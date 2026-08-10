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

/* CCCR Bus Interface Control (function 0, register 0x07) */
#define SDIO_CCCR_BUS_IF_CTRL   0x07U
#define SDIO_CCCR_BUS_WIDTH_MSK 0x03U
#define SDIO_CCCR_BUS_WIDTH_4B  0x02U

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

/**
  * @brief Select PLL1Q as the SDMMC1 kernel clock.
  *
  * HAL_RCC_OscConfig() only enables the PLL1 P output, so PLL1Q is still off
  * after SystemClock_Config() and the kernel clock frequency reads back as 0.
  * HAL_RCCEx_PeriphCLKConfig() is what turns PLL1Q on, so it has to run before
  * the frequency is queried - i.e. before HAL_SDIO_Init(), not from MspInit.
  */
static HAL_StatusTypeDef SdioKernelClockConfig(void)
{
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SDMMC1;
    PeriphClkInitStruct.Sdmmc1ClockSelection = RCC_SDMMC1CLKSOURCE_PLL1Q;

    return HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);
}

void HAL_SDIO_MspInit(SDIO_HandleTypeDef *hsdio)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if (hsdio->Instance != SDMMC1) {
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

/**
  * @brief Re-run the first two enumeration steps and report each one.
  *
  * Distinguishes "slave is silent" (CMD5 timeout - no SDIO AT firmware, or a
  * wiring/pull-up fault) from "slave answered but a later step failed".
  */
static void SdioProbeSlave(void)
{
    uint32_t err;
    uint32_t resp4 = 0U;

    if (SDMMC_GetPowerState(hsdio1.Instance) == 0U) {
        SDIO_LOGE(TAG, "probe: SDMMC1 power state is off");
        return;
    }

    err = SDMMC_CmdGoIdleState(hsdio1.Instance);
    SDIO_LOGE(TAG, "probe: CMD0 err=0x%08lx", (unsigned long)err);

    err = SDMMC_CmdSendOperationcondition(hsdio1.Instance, 0U, &resp4);
    SDIO_LOGE(TAG, "probe: CMD5 err=0x%08lx resp=0x%08lx nbr_of_func=%lu",
              (unsigned long)err, (unsigned long)resp4,
              (unsigned long)((resp4 & 0x70000000U) >> 28U));

    if (err != 0U) {
        SDIO_LOGE(TAG, "probe: slave did not answer CMD5 - check that the ESP is "
                       "running SDIO AT firmware, the CMD/D0-D3 pull-ups, and GND");
    } else if (((resp4 & 0x70000000U) >> 28U) == 0U) {
        SDIO_LOGE(TAG, "probe: CMD5 answered but reports 0 IO functions");
    } else {
        SDIO_LOGE(TAG, "probe: CMD5 OK - failure is later in enumeration (CMD3/CMD7)");
    }
}

#if (SDIO_PORT_BUS_WIDTH == HAL_SDIO_4_WIRES_MODE)
/**
  * @brief Widen the link to 4 bits once enumeration has completed.
  *
  * HAL_SDIO_Init() compares hsdio->Init.BusWide (an SDMMC_BUS_WIDE_* register
  * value) against HAL_SDIO_4_WIRES_MODE (1), so it always writes 1-bit into
  * CCCR 0x07. Enumeration therefore runs in 1-bit and the switch is done here:
  * CMD52 travels on CMD only, so it is unaffected by the width, and CLKCR is
  * moved to 4-bit only after the slave has acknowledged the new setting.
  */
static sdio_err_t SdioSetBusWidth4(void)
{
    uint8_t val;

    if (sdio_driver_read_byte(0U, SDIO_CCCR_BUS_IF_CTRL, &val) != SDIO_SUCCESS) {
        SDIO_LOGE(TAG, "CCCR 0x07 read failed");
        return FAILURE;
    }

    val = (uint8_t)((val & ~SDIO_CCCR_BUS_WIDTH_MSK) | SDIO_CCCR_BUS_WIDTH_4B);
    if (sdio_driver_write_byte(0U, SDIO_CCCR_BUS_IF_CTRL, val, &val) != SDIO_SUCCESS) {
        SDIO_LOGE(TAG, "CCCR 0x07 write failed");
        return FAILURE;
    }

    if ((val & SDIO_CCCR_BUS_WIDTH_MSK) != SDIO_CCCR_BUS_WIDTH_4B) {
        SDIO_LOGE(TAG, "slave refused 4-bit, CCCR 0x07 = 0x%02x", (unsigned int)val);
        return FAILURE;
    }

    MODIFY_REG(hsdio1.Instance->CLKCR, SDMMC_CLKCR_WIDBUS, SDMMC_BUS_WIDE_4B);
    hsdio1.Init.BusWide = SDMMC_BUS_WIDE_4B;

    SDIO_LOGI(TAG, "bus width switched to 4-bit");
    return SDIO_SUCCESS;
}
#endif /* SDIO_PORT_BUS_WIDTH == HAL_SDIO_4_WIRES_MODE */

sdio_err_t sdio_driver_init(void)
{
    uint32_t kernel_clk;

    if (SdioKernelClockConfig() != HAL_OK) {
        SDIO_LOGE(TAG, "SDMMC1 kernel clock config failed");
        return FAILURE;
    }

    kernel_clk = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SDMMC1);
    if (kernel_clk == 0U) {
        SDIO_LOGE(TAG, "SDMMC1 kernel clock is 0");
        return FAILURE;
    }

    hsdio1.Instance = SDMMC1;
    hsdio1.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
    hsdio1.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
    /* Enumerate in 1-bit; SdioSetBusWidth4() widens the link afterwards */
    hsdio1.Init.BusWide = SDMMC_BUS_WIDE_1B;
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
        /* SDIO_InitCard() returns HAL_ERROR without recording an ErrorCode, so
           re-probe CMD0/CMD5 here to say which half of the link is at fault. */
        SDIO_LOGE(TAG, "HAL_SDIO_Init failed, err=0x%08lx",
                  (unsigned long)HAL_SDIO_GetError(&hsdio1));
        SdioProbeSlave();
        return FAILURE;
    }

#if (SDIO_PORT_BUS_WIDTH == HAL_SDIO_4_WIRES_MODE)
    if (SdioSetBusWidth4() != SDIO_SUCCESS) {
        return FAILURE;
    }
#endif

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
