// Copyright 2015-2020 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/* STM32H5 port. The F1 reference port implements CMD52/CMD53 by hand in
   sdio.c; on H5 stm32h5xx_hal_sdio.c already does that, so this file is only
   the SDMMC1 low level init plus a thin mapping onto the HAL_SDIO API. */

#include <stdio.h>
#include <stdint.h>
#include <stddef.h>

#include "stm32h5xx_hal.h"

#include "sdio_host_log.h"
#include "sdio_host_error.h"
#include "sdio_driver_port.h"
#include "platform_os.h"
#include "sdio.h"

static const char TAG[] = "sdio_driver";

/* Referenced by sdio_host_log.h; raise to 3 for the CCCR dump during bring-up */
int sdio_debugLevel = 3;

SDIO_HandleTypeDef hsdio1;

static volatile uint8_t esp_int_flag;

/* CMD53 byte mode carries a 9-bit count, so a single transfer tops out at 512 */
#define SDIO_PORT_MAX_BYTE_XFER 512U

/**
  * @brief Slave raised its function-1 interrupt (D1 held low).
  *
  * SDMMC_STA_SDIOIT tracks the D1 level and the HAL does not clear it, so the
  * source has to be masked here or the ISR re-enters until the slave releases
  * D1. sdio_driver_wait_int() re-arms it.
  */
static void EspIoInterruptCallback(SDIO_HandleTypeDef *hsdio, uint32_t func)
{
    (void)func;
    __HAL_SDIO_DISABLE_IT(hsdio, SDMMC_IT_SDIOIT);
    esp_int_flag = 1U;
}

void HAL_SDIO_MspInit(SDIO_HandleTypeDef *hsdio)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

    if (hsdio->Instance != SDMMC1) {
        return;
    }

    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SDMMC1;
    PeriphClkInitStruct.Sdmmc1ClockSelection = RCC_SDMMC1CLKSOURCE_PLL1Q;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
        SDIO_LOGE(TAG, "SDMMC1 kernel clock config failed");
        return;
    }

    __HAL_RCC_SDMMC1_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /**SDMMC1 GPIO Configuration
    PB2      ------> SDMMC1_CMD
    PB13     ------> SDMMC1_D0
    PC9      ------> SDMMC1_D1
    PC10     ------> SDMMC1_D2
    PC11     ------> SDMMC1_D3
    PC12     ------> SDMMC1_CK
    */
    GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF12_SDMMC1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    HAL_NVIC_SetPriority(SDMMC1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(SDMMC1_IRQn);
}

void HAL_SDIO_MspDeInit(SDIO_HandleTypeDef *hsdio)
{
    if (hsdio->Instance != SDMMC1) {
        return;
    }

    __HAL_RCC_SDMMC1_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_2 | GPIO_PIN_13);
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12);
    HAL_NVIC_DisableIRQ(SDMMC1_IRQn);
}

void SdioPortIrqHandler(void)
{
    HAL_SDIO_IRQHandler(&hsdio1);
}

sdio_err_t sdio_driver_init(void)
{
    uint32_t kernel_clk;

    kernel_clk = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SDMMC1);
    if (kernel_clk == 0U) {
        SDIO_LOGE(TAG, "SDMMC1 kernel clock is 0");
        return FAILURE;
    }

    hsdio1.Instance = SDMMC1;
    hsdio1.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
    hsdio1.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
    hsdio1.Init.BusWide = (SDIO_PORT_BUS_WIDTH == HAL_SDIO_4_WIRES_MODE)
                          ? SDMMC_BUS_WIDE_4B : SDMMC_BUS_WIDE_1B;
    hsdio1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
    /* SDMMC_CK = kernel_clk / (2 * ClockDiv); ClockDiv is 10 bits */
    hsdio1.Init.ClockDiv = kernel_clk / (2U * SDIO_PORT_CLOCK_HZ);
    if (hsdio1.Init.ClockDiv > 0x3FFU) {
        hsdio1.Init.ClockDiv = 0x3FFU;
    }

    SDIO_LOGI(TAG, "[Clock] kernel=%luHz, div=%lu, SDMMC_CK=%luHz",
              (unsigned long)kernel_clk, (unsigned long)hsdio1.Init.ClockDiv,
              (unsigned long)(hsdio1.Init.ClockDiv ? kernel_clk / (2U * hsdio1.Init.ClockDiv)
                                                   : kernel_clk));

    /* Runs the CMD0/CMD5/CMD3/CMD7 enumeration and sets the CCCR bus width */
    if (HAL_SDIO_Init(&hsdio1) != HAL_OK) {
        SDIO_LOGE(TAG, "HAL_SDIO_Init failed, err=0x%08lx",
                  (unsigned long)HAL_SDIO_GetError(&hsdio1));
        return FAILURE;
    }

    if (HAL_SDIO_RegisterIOFunctionCallback(&hsdio1, SDIO_PORT_FUNCTION,
                                            EspIoInterruptCallback) != HAL_OK) {
        return FAILURE;
    }

    if (HAL_SDIO_EnableIOFunction(&hsdio1, SDIO_PORT_FUNCTION) != HAL_OK) {
        SDIO_LOGE(TAG, "enable IO function 1 failed");
        return FAILURE;
    }

    if (HAL_SDIO_SetBlockSize(&hsdio1, SDIO_PORT_FUNCTION, SDIO_PORT_BLOCK_SIZE) != HAL_OK) {
        SDIO_LOGE(TAG, "set block size failed");
        return FAILURE;
    }

    if (HAL_SDIO_EnableIOFunctionInterrupt(&hsdio1, SDIO_PORT_FUNCTION) != HAL_OK) {
        SDIO_LOGE(TAG, "enable IO function 1 interrupt failed");
        return FAILURE;
    }

    return SDIO_SUCCESS;
}

sdio_err_t sdio_driver_read_byte(uint32_t function, uint32_t reg, uint8_t *out_byte)
{
    HAL_SDIO_DirectCmd_TypeDef cmd52;

    cmd52.Reg_Addr = reg;
    cmd52.ReadAfterWrite = 0U;
    cmd52.IOFunctionNbr = function & 0x7U;

    if (HAL_SDIO_ReadDirect(&hsdio1, &cmd52, out_byte) != HAL_OK) {
        SDIO_LOGE(TAG, "CMD52 read error, func %lu reg 0x%lx",
                  (unsigned long)function, (unsigned long)reg);
        return FAILURE;
    }
    return SDIO_SUCCESS;
}

sdio_err_t sdio_driver_write_byte(uint32_t function, uint32_t reg, uint8_t in_byte,
                                  uint8_t *out_byte)
{
    HAL_SDIO_DirectCmd_TypeDef cmd52;

    cmd52.Reg_Addr = reg;
    cmd52.ReadAfterWrite = 0U;
    cmd52.IOFunctionNbr = function & 0x7U;

    if (HAL_SDIO_WriteDirect(&hsdio1, &cmd52, in_byte) != HAL_OK) {
        SDIO_LOGE(TAG, "CMD52 write error, func %lu reg 0x%lx",
                  (unsigned long)function, (unsigned long)reg);
        return FAILURE;
    }

    /* HAL_SDIO_WriteDirect discards the read-after-write payload, so when the
       caller wants it back issue a plain CMD52 read instead. */
    if (out_byte != NULL) {
        return sdio_driver_read_byte(function, reg, out_byte);
    }
    return SDIO_SUCCESS;
}

static sdio_err_t SdioTransfer(uint32_t function, uint32_t addr, void *buffer,
                               uint32_t len, uint32_t block_mode, uint8_t is_write)
{
    HAL_SDIO_ExtendedCmd_TypeDef cmd53;
    HAL_StatusTypeDef status;

    if ((buffer == NULL) || (len == 0U)) {
        return ERR_INVALID_ARG;
    }
    if (((uintptr_t)buffer & 3U) != 0U) {
        SDIO_LOGE(TAG, "buffer must be 4-byte aligned");
        return ERR_INVALID_ARG;
    }
    if ((block_mode == HAL_SDIO_MODE_BYTE) && (len > SDIO_PORT_MAX_BYTE_XFER)) {
        SDIO_LOGE(TAG, "byte mode transfer too long: %lu", (unsigned long)len);
        return ERR_INVALID_ARG;
    }
    if ((block_mode == HAL_SDIO_MODE_BLOCK) && ((len % SDIO_PORT_BLOCK_SIZE) != 0U)) {
        SDIO_LOGE(TAG, "block mode length %lu not a multiple of %u",
                  (unsigned long)len, (unsigned int)SDIO_PORT_BLOCK_SIZE);
        return ERR_INVALID_ARG;
    }

    cmd53.Reg_Addr = addr;
    cmd53.OpCode = HAL_SDIO_OP_CODE_AUTO_INC;
    cmd53.Block_Mode = block_mode;
    cmd53.IOFunctionNbr = function & 0x7U;

    if (is_write != 0U) {
        status = HAL_SDIO_WriteExtended(&hsdio1, &cmd53, (uint8_t *)buffer, len, 1000U);
    } else {
        status = HAL_SDIO_ReadExtended(&hsdio1, &cmd53, (uint8_t *)buffer, len, 1000U);
    }

    if (status != HAL_OK) {
        SDIO_LOGE(TAG, "CMD53 %s error, addr 0x%lx count %lu, err 0x%08lx",
                  (is_write != 0U) ? "write" : "read", (unsigned long)addr,
                  (unsigned long)len, (unsigned long)HAL_SDIO_GetError(&hsdio1));
        return FAILURE;
    }
    return SDIO_SUCCESS;
}

sdio_err_t sdio_driver_read_bytes(uint32_t function, uint32_t addr, void *buffer, uint32_t len)
{
    return SdioTransfer(function, addr, buffer, len, HAL_SDIO_MODE_BYTE, 0U);
}

sdio_err_t sdio_driver_write_bytes(uint32_t function, uint32_t addr, void *buffer, uint32_t len)
{
    return SdioTransfer(function, addr, buffer, len, HAL_SDIO_MODE_BYTE, 1U);
}

sdio_err_t sdio_driver_read_blocks(uint32_t function, uint32_t addr, void *buffer, uint32_t len)
{
    return SdioTransfer(function, addr, buffer, len, HAL_SDIO_MODE_BLOCK, 0U);
}

sdio_err_t sdio_driver_write_blocks(uint32_t function, uint32_t addr, void *buffer, uint32_t len)
{
    return SdioTransfer(function, addr, buffer, len, HAL_SDIO_MODE_BLOCK, 1U);
}

sdio_err_t sdio_driver_wait_int(uint32_t timeout)
{
    uint32_t start = HAL_GetTick();

    esp_int_flag = 0U;
    __HAL_SDIO_CLEAR_FLAG(&hsdio1, SDMMC_FLAG_SDIOIT);
    __HAL_SDIO_ENABLE_IT(&hsdio1, SDMMC_IT_SDIOIT);
    __SDMMC_OPERATION_ENABLE(hsdio1.Instance);

    while (esp_int_flag == 0U) {
        if ((HAL_GetTick() - start) > timeout) {
            __HAL_SDIO_DISABLE_IT(&hsdio1, SDMMC_IT_SDIOIT);
            return ERR_TIMEOUT;
        }
    }
    return SDIO_SUCCESS;
}
