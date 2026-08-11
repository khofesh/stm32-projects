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

/* ESP-AT can take several seconds to boot and enable its SDIO slave, while the
   STM32 is ready in milliseconds. SDIO_InitCard() sends CMD5 exactly once, so
   enumeration is retried here instead of losing that race on every boot. */
#define SDIO_INIT_RETRY_COUNT   60U
#define SDIO_INIT_RETRY_MS      500U

/* CCCR Bus Interface Control (function 0, register 0x07) */
#define SDIO_CCCR_BUS_IF_CTRL   0x07U
#define SDIO_CCCR_BUS_WIDTH_MSK 0x03U
#define SDIO_CCCR_BUS_WIDTH_4B  0x02U
#define SDIO_OCR_3V3_WINDOW     0x00FF8000U

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
    GPIO_InitStruct.Speed = SDIO_PORT_GPIO_SPEED;
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

/* R4 carries no CRC and its command-index field is all ones, so a correctly
   framed response always reads back RESPCMD = 0x3F. Any other value means the
   CPSM latched the frame at the wrong bit position. */
#define SDIO_R4_RESPCMD         0x3FU
/* OCR bits 6:0 are reserved and always read 0 in a valid R4 */
#define SDIO_R4_OCR_RSVD_MSK    0x0000007FU

/* Every cell of the probe matrix repeats CMD5 this many times. A link that
   answers some of the time is intermittent - a resistive joint or a marginal
   jumper - which no single-shot test can tell apart from a dead one. */
#define SDIO_PROBE_REPEAT       8U

static const uint32_t sdio_probe_speed[] = {
    GPIO_SPEED_FREQ_VERY_HIGH,
    GPIO_SPEED_FREQ_HIGH,
    GPIO_SPEED_FREQ_MEDIUM,
    GPIO_SPEED_FREQ_LOW,
};
static const char *const sdio_probe_speed_name[] = { "vhigh", "high", "med", "low" };
#define SDIO_PROBE_SPEED_COUNT  (sizeof(sdio_probe_speed) / sizeof(sdio_probe_speed[0]))

/**
  * @brief Re-mux the six SDMMC nets with a different output slew rate.
  */
static void SdioSetPadSpeed(uint32_t speed)
{
    GPIO_InitTypeDef gpio = {0};

    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = speed;
    gpio.Alternate = GPIO_AF12_SDMMC1;

    gpio.Pin = GPIO_PIN_2 | GPIO_PIN_13;
    HAL_GPIO_Init(GPIOB, &gpio);
    gpio.Pin = GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12;
    HAL_GPIO_Init(GPIOC, &gpio);
}

/**
  * @brief Frame-alignment verdict on one R4.
  *
  * The host's shift register always clocks exactly 32 times, so it cannot lose
  * bits: a response that arrives short means the slave advanced its output
  * early, i.e. it saw CK edges the host never issued. RESPCMD says whether the
  * drift had already started by the index field (0x3F = still aligned there),
  * and the trailing reserved ones shifted into the bottom of RESP1 say how many
  * clocks were lost by the end.
  *
  * The NF field is not usable as a verdict - it is one of the first bits to be
  * pushed out, which is why a glitching bus reads as "0 IO functions".
  */
static void SdioReportAlignment(uint32_t resp4, uint32_t respcmd)
{
    uint32_t k;

    if ((respcmd == SDIO_R4_RESPCMD) && ((resp4 & SDIO_R4_OCR_RSVD_MSK) == 0U)) {
        SDIO_LOGE(TAG, "    frame aligned, nf=%lu, OCR=0x%06lx",
                  (unsigned long)((resp4 & 0x70000000U) >> 28U),
                  (unsigned long)(resp4 & 0x00FFFFFFU));
        return;
    }

    for (k = 1U; k < 8U; k++) {
        if (((resp4 & ((1UL << k) - 1UL)) == ((1UL << k) - 1UL)) &&
            (((resp4 >> k) & SDIO_R4_OCR_RSVD_MSK) == 0U)) {
            SDIO_LOGE(TAG, "    slave saw %lu extra CK edge(s) during the "
                           "response (resp = R4<<%lu); OCR=0x%06lx recovered, "
                           "NF lost off the top, index field %s",
                      (unsigned long)k, (unsigned long)k,
                      (unsigned long)((resp4 >> k) & 0x00FFFFFFU),
                      (respcmd == SDIO_R4_RESPCMD) ? "still intact (drift began "
                                                     "in the payload)"
                                                   : "already short (drift began "
                                                     "before the payload)");
            return;
        }
    }
    SDIO_LOGE(TAG, "    corrupt - no single clock count fits the frame");
}

/**
  * @brief One matrix cell: SDIO_PROBE_REPEAT x CMD5 at a fixed slew and CLKDIV.
  */
static void SdioProbeCmd5(uint32_t clkdiv, uint32_t arg, const char *speed_name)
{
    uint32_t i;
    uint32_t resp4 = 0U;
    uint32_t first = 0U;
    uint32_t last = 0U;
    uint32_t respcmd = 0U;
    uint32_t answered = 0U;
    uint32_t varies = 0U;

    MODIFY_REG(hsdio1.Instance->CLKCR, SDMMC_CLKCR_CLKDIV, clkdiv);
    HAL_Delay(2U);
    (void)SDMMC_CmdGoIdleState(hsdio1.Instance);
    HAL_Delay(2U);

    for (i = 0U; i < SDIO_PROBE_REPEAT; i++) {
        resp4 = 0U;
        if (SDMMC_CmdSendOperationcondition(hsdio1.Instance, arg, &resp4) == 0U) {
            if (answered == 0U) {
                first = resp4;
                respcmd = hsdio1.Instance->RESPCMD & 0x3FU;
            } else if (resp4 != first) {
                varies = 1U;
            }
            last = resp4;
            answered++;
        }
        HAL_Delay(1U);
    }

    SDIO_LOGE(TAG, "  pads=%-5s div=%-4lu arg=0x%08lx answered=%lu/%u "
                   "respcmd=0x%02lx resp=0x%08lx",
              speed_name, (unsigned long)clkdiv, (unsigned long)arg,
              (unsigned long)answered, (unsigned int)SDIO_PROBE_REPEAT,
              (unsigned long)respcmd, (unsigned long)first);

    /* With a voltage window in the argument the card leaves busy part way
       through the batch and legitimately answers differently, so a changing
       response is only a fault indication when the argument is 0. */
    if (varies != 0U) {
        SDIO_LOGE(TAG, "    last=0x%08lx - response changed during the batch%s",
                  (unsigned long)last,
                  (arg != 0U) ? " (expected: card leaving busy)"
                              : " (with arg=0 this is a fault - unstable link)");
    }

    if (answered != 0U) {
        SdioReportAlignment(first, respcmd);
    }
}

/**
  * @brief Sweep pad slew rate x bus clock, repeating CMD5 in every cell.
  *
  * Three outcomes are worth separating and none of them is visible from a
  * single CMD5: nothing ever answers (slave silent, or a dead net), some tries
  * answer (intermittent connection - a resistive joint, not a missing one), or
  * every try answers with a shifted frame (bus timing).
  */
static void SdioProbeSlave(void)
{
    GPIO_InitTypeDef gpio = {0};
    uint32_t err;
    uint32_t speed;
    uint32_t base_div = hsdio1.Instance->CLKCR & SDMMC_CLKCR_CLKDIV;
    GPIO_PinState cmd_level;

    if (SDMMC_GetPowerState(hsdio1.Instance) == 0U) {
        SDIO_LOGE(TAG, "probe: SDMMC1 power state is off");
        return;
    }

    err = SDMMC_CmdGoIdleState(hsdio1.Instance);
    SDIO_LOGE(TAG, "probe: CMD0 err=0x%08lx", (unsigned long)err);
    SDIO_LOGE(TAG, "probe: CMD5 matrix, slew rate x clock (div 0x3ff = slowest)");

    for (speed = 0U; speed < SDIO_PROBE_SPEED_COUNT; speed++) {
        SdioSetPadSpeed(sdio_probe_speed[speed]);
        SdioProbeCmd5(base_div, 0U, sdio_probe_speed_name[speed]);
        SdioProbeCmd5(0x3FFU, 0U, sdio_probe_speed_name[speed]);
    }

    SdioSetPadSpeed(SDIO_PORT_GPIO_SPEED);
    SdioProbeCmd5(base_div, SDIO_OCR_3V3_WINDOW, "cfg");
    MODIFY_REG(hsdio1.Instance->CLKCR, SDMMC_CLKCR_CLKDIV, base_div);

    gpio.Pin = GPIO_PIN_2;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &gpio);
    cmd_level = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2);
    SDIO_LOGE(TAG, "probe: CMD pin level after timeout: %s",
              (cmd_level == GPIO_PIN_SET) ? "high" : "low");

    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Speed = SDIO_PORT_GPIO_SPEED;
    gpio.Alternate = GPIO_AF12_SDMMC1;
    HAL_GPIO_Init(GPIOB, &gpio);

    SDIO_LOGE(TAG, "probe: 'answered=0/8' everywhere is a silent slave or a dead "
                   "net; a mix of 0/8 and 8/8 is an intermittent joint; 8/8 with "
                   "a constant edge count on every row is CMD-to-CK coupling, "
                   "which no host setting can fix - separate CK from CMD, and "
                   "filter CK at the slave input");
}

#if (SDIO_PORT_PIN_DIAG != 0)
typedef struct {
    const char   *name;
    GPIO_TypeDef *port;
    uint16_t      pin;
    uint8_t       has_pullup;   /* the board fits a 10k pull-up on this net */
} SdioPinDesc;

static const SdioPinDesc sdio_pins[] = {
    { "CMD(PB2)",  GPIOB, GPIO_PIN_2,  1U },
    { "D0(PB13)",  GPIOB, GPIO_PIN_13, 1U },
    { "D1(PC9)",   GPIOC, GPIO_PIN_9,  1U },
    { "D2(PC10)",  GPIOC, GPIO_PIN_10, 1U },
    { "D3(PC11)",  GPIOC, GPIO_PIN_11, 1U },
    { "CK(PC12)",  GPIOC, GPIO_PIN_12, 0U },
};
#define SDIO_PIN_COUNT (sizeof(sdio_pins) / sizeof(sdio_pins[0]))

/* Control: LD1, nothing external on the net. If this one also fails to read
   back what the output driver puts on it, the diagnostic itself is wrong. */
static const SdioPinDesc sdio_ctrl_pin = { "CTRL(PA5)", GPIOA, GPIO_PIN_5, 0U };

static void SdioPinMode(const SdioPinDesc *p, uint32_t mode, uint32_t pull)
{
    GPIO_InitTypeDef gpio = {0};

    gpio.Pin = p->pin;
    gpio.Mode = mode;
    gpio.Pull = pull;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(p->port, &gpio);
}

static GPIO_PinState SdioPinRead(const SdioPinDesc *p, uint32_t pull)
{
    SdioPinMode(p, GPIO_MODE_INPUT, pull);
    HAL_Delay(2U);
    return HAL_GPIO_ReadPin(p->port, p->pin);
}

/**
  * @brief Electrical check of the six SDMMC nets, before any AF is muxed on.
  *
  * Two passes, neither of which needs a scope:
  *
  * 1. Pull-up census. Read each pin with the internal pull-down engaged (~40k);
  *    an external 10k to 3V3 wins, so high means "the pull-up, and therefore the
  *    wire it sits on, is really there". Low means the net is floating - a
  *    missing pull-up or an open jumper - which is exactly what makes CMD5 time
  *    out. CK carries no pull-up, so it is only driven, not censused.
  * 2. Short matrix. Drive one pin low while the other five are inputs with
  *    internal pull-ups; anything that follows shares copper with it. Catches
  *    shorted or swapped jumpers, and a pin that cannot be pulled low by its own
  *    output driver is shorted to 3V3.
  */
static uint32_t SdioPinDiag(void)
{
    uint32_t i;
    uint32_t j;
    uint32_t floating = 0U;
    uint32_t stuck = 0U;

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* Validate the method itself on a net with nothing attached */
    SdioPinMode(&sdio_ctrl_pin, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
    HAL_Delay(2U);
    i = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5) == GPIO_PIN_RESET) ? 1U : 0U;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
    HAL_Delay(2U);
    j = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5) == GPIO_PIN_SET) ? 1U : 0U;
    SDIO_LOGE(TAG, "pin diag: control %s drive0=%s drive1=%s%s", sdio_ctrl_pin.name,
              (i != 0U) ? "ok" : "FAIL", (j != 0U) ? "ok" : "FAIL",
              ((i & j) != 0U) ? "" : " - readback method is broken, ignore the rest");
    SdioPinMode(&sdio_ctrl_pin, GPIO_MODE_INPUT, GPIO_NOPULL);

    SDIO_LOGE(TAG, "pin diag: pull-up census (expect 'high' on CMD and D0-D3)");
    for (i = 0U; i < SDIO_PIN_COUNT; i++) {
        GPIO_PinState with_pd;
        GPIO_PinState with_pu;

        if (sdio_pins[i].has_pullup == 0U) {
            continue;
        }

        with_pd = SdioPinRead(&sdio_pins[i], GPIO_PULLDOWN);
        with_pu = SdioPinRead(&sdio_pins[i], GPIO_PULLUP);

        if (with_pd == GPIO_PIN_SET) {
            /* A hard short to 3V3 reads the same here; the matrix separates them */
            SDIO_LOGE(TAG, "  %s: high  (something pulls this net up)",
                      sdio_pins[i].name);
        } else if (with_pu == GPIO_PIN_SET) {
            floating++;
            SDIO_LOGE(TAG, "  %s: FLOAT (no external pull-up - open wire or "
                           "missing resistor)", sdio_pins[i].name);
        } else {
            SDIO_LOGE(TAG, "  %s: LOW   (net shorted to GND)", sdio_pins[i].name);
        }
    }

    SDIO_LOGE(TAG, "pin diag: short matrix");
    for (i = 0U; i < SDIO_PIN_COUNT; i++) {
        for (j = 0U; j < SDIO_PIN_COUNT; j++) {
            if (j != i) {
                SdioPinMode(&sdio_pins[j], GPIO_MODE_INPUT, GPIO_PULLUP);
            }
        }

        SdioPinMode(&sdio_pins[i], GPIO_MODE_OUTPUT_PP, GPIO_NOPULL);
        HAL_GPIO_WritePin(sdio_pins[i].port, sdio_pins[i].pin, GPIO_PIN_RESET);
        HAL_Delay(2U);

        if (HAL_GPIO_ReadPin(sdio_pins[i].port, sdio_pins[i].pin) != GPIO_PIN_RESET) {
            uint32_t pos = (uint32_t)(31 - __CLZ((uint32_t)sdio_pins[i].pin));

            stuck++;
            /* Separate "the pad is really being driven and loses" from "the
               config write never landed" (GTZC/SECCFGR, missing clock, LCKR) */
            SDIO_LOGE(TAG, "  %s: driven low but reads high - MODER=%lu ODR=%lu "
                           "OTYPER=%lu PUPDR=%lu",
                      sdio_pins[i].name,
                      (unsigned long)((sdio_pins[i].port->MODER >> (2U * pos)) & 3U),
                      (unsigned long)((sdio_pins[i].port->ODR >> pos) & 1U),
                      (unsigned long)((sdio_pins[i].port->OTYPER >> pos) & 1U),
                      (unsigned long)((sdio_pins[i].port->PUPDR >> (2U * pos)) & 3U));
        }

        for (j = 0U; j < SDIO_PIN_COUNT; j++) {
            if ((j != i) &&
                (HAL_GPIO_ReadPin(sdio_pins[j].port, sdio_pins[j].pin) == GPIO_PIN_RESET)) {
                SDIO_LOGE(TAG, "  %s shorted to %s", sdio_pins[i].name, sdio_pins[j].name);
            }
        }

        HAL_GPIO_WritePin(sdio_pins[i].port, sdio_pins[i].pin, GPIO_PIN_SET);
        HAL_Delay(2U);
    }

    /* Leave everything as a plain input; HAL_SDIO_MspInit() muxes the AF back on */
    for (i = 0U; i < SDIO_PIN_COUNT; i++) {
        SdioPinMode(&sdio_pins[i], GPIO_MODE_INPUT, GPIO_NOPULL);
    }

    if (stuck != 0U) {
        /* MODER=1/ODR=0/OTYPER=0 in the lines above means the pad really is a
           push-pull output driving low, so only a low impedance path to 3V3 can
           hold the net high - a bridged or mis-wired pull-up, not a 10k one.
           The host cannot assert CMD in this state, which is the CMD5 timeout. */
        SDIO_LOGE(TAG, "pin diag: %lu net(s) tied hard to 3V3 - the host cannot "
                       "drive them at all; rework the pull-ups before blaming "
                       "the slave", (unsigned long)stuck);
    } else if (floating != 0U) {
        SDIO_LOGE(TAG, "pin diag: %lu net(s) floating - fix the wiring before "
                       "reading anything into the CMD5 timeout",
                  (unsigned long)floating);
    } else {
        SDIO_LOGE(TAG, "pin diag: wiring looks sane from the host side");
    }

    return stuck + floating;
}
#endif /* SDIO_PORT_PIN_DIAG */

#if (SDIO_PORT_CK_TOGGLE_MS != 0)
/**
  * @brief Square wave on CK and CMD, slow enough for a meter or logic probe.
  *
  * Proves the host really drives PC12/PB2 and that the far end sees them,
  * which the SDMMC at 400 kHz cannot demonstrate without an analyser.
  */
static void SdioToggleCk(void)
{
    GPIO_InitTypeDef gpio = {0};
    uint32_t end;

    __HAL_RCC_GPIOC_CLK_ENABLE();
    gpio.Pin = GPIO_PIN_12;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &gpio);

    SDIO_LOGE(TAG, "toggling CK(PC12) at 2 Hz for %u ms - measure at ESP32-C6 "
                   "GPIO19", (unsigned int)SDIO_PORT_CK_TOGGLE_MS);

    end = HAL_GetTick() + SDIO_PORT_CK_TOGGLE_MS;
    while (HAL_GetTick() < end) {
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_12);
        HAL_Delay(250U);
    }
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET);

    __HAL_RCC_GPIOB_CLK_ENABLE();
    gpio.Pin = GPIO_PIN_2;
    HAL_GPIO_Init(GPIOB, &gpio);

    SDIO_LOGE(TAG, "toggling CMD(PB2) at 2 Hz for %u ms - measure at ESP32-C6 "
                   "GPIO18", (unsigned int)SDIO_PORT_CK_TOGGLE_MS);

    end = HAL_GetTick() + SDIO_PORT_CK_TOGGLE_MS;
    while (HAL_GetTick() < end) {
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_2);
        HAL_Delay(250U);
    }
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
}
#endif /* SDIO_PORT_CK_TOGGLE_MS */

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
    uint32_t retry;
    HAL_StatusTypeDef status = HAL_ERROR;

#if (SDIO_PORT_PIN_DIAG != 0)
    /* No point clocking the bus when the host cannot even assert CMD */
    if (SdioPinDiag() != 0U) {
        return FAILURE;
    }
#endif
#if (SDIO_PORT_CK_TOGGLE_MS != 0)
    SdioToggleCk();
#endif

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

    /* Runs the CMD0/CMD5/CMD3/CMD7 enumeration and sets the CCCR bus width.
       A failed attempt leaves State back at RESET, so re-calling it is a clean
       restart of the whole sequence. */
    for (retry = 0U; retry < SDIO_INIT_RETRY_COUNT; retry++) {
        status = HAL_SDIO_Init(&hsdio1);
        if (status == HAL_OK) {
            break;
        }
        HAL_Delay(SDIO_INIT_RETRY_MS);
    }

    if (status != HAL_OK) {
        /* SDIO_InitCard() returns HAL_ERROR without recording an ErrorCode, so
           re-probe CMD0/CMD5 here to say which half of the link is at fault. */
        SDIO_LOGE(TAG, "HAL_SDIO_Init failed after %lu attempts, err=0x%08lx",
                  (unsigned long)SDIO_INIT_RETRY_COUNT,
                  (unsigned long)HAL_SDIO_GetError(&hsdio1));
        SdioProbeSlave();
        return FAILURE;
    }

    if (retry > 0U) {
        SDIO_LOGI(TAG, "slave enumerated after %lu retries (%lu ms)",
                  (unsigned long)retry,
                  (unsigned long)(retry * SDIO_INIT_RETRY_MS));
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
        uint32_t err = HAL_SDIO_GetError(&hsdio1);
        SDIO_LOGE(TAG, "CMD53 %s error, addr 0x%lx count %lu, err 0x%08lx%s%s%s%s%s",
                  (is_write != 0U) ? "write" : "read", (unsigned long)addr,
                  (unsigned long)len, (unsigned long)err,
                  ((err & HAL_SDIO_ERROR_DATA_CRC_FAIL) != 0U) ? " DATA_CRC_FAIL" : "",
                  ((err & HAL_SDIO_ERROR_DATA_TIMEOUT) != 0U) ? " DATA_TIMEOUT" : "",
                  ((err & HAL_SDIO_ERROR_TX_UNDERRUN) != 0U) ? " TX_UNDERRUN" : "",
                  ((err & HAL_SDIO_ERROR_RX_OVERRUN) != 0U) ? " RX_OVERRUN" : "",
                  ((err & HAL_SDIO_ERROR_TIMEOUT) != 0U) ? " TIMEOUT" : "");
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
