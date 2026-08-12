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

/* Set while a transfer failure is an expected consequence of the slave being
   told to reboot, so the teardown does not print faults that are not faults */
static uint8_t sdio_quiet_errors;

/* CMD53 byte mode carries a 9-bit count, so a single transfer tops out at 512 */
#define SDIO_PORT_MAX_BYTE_XFER 512U
#define SDIO_CMD53_RETRY_COUNT  32U
/* ...but a data phase that never starts is not worth 32 of them. CRC failures
   are frames lost in flight and retry well; HAL_SDIO_ERROR_TIMEOUT means the
   slave answered R5 and then sent nothing, which it will do again. Burning the
   full count on it left the console dead for 32 s at a stretch. */
#define SDIO_CMD53_TIMEOUT_RETRY 3U

/* ESP-AT can take several seconds to boot and enable its SDIO slave, while the
   STM32 is ready in milliseconds. SDIO_InitCard() sends CMD5 exactly once, so
   enumeration is retried here instead of losing that race on every boot. */
#define SDIO_INIT_RETRY_COUNT   60U
#define SDIO_INIT_RETRY_MS      500U
/* A re-enumeration after AT+RESTORE is a different bet: the slave was alive a
   moment ago and only has to finish one reboot, so a full 30 s of retries here
   just leaves the console mute. Give up sooner and let the caller decide
   whether to wait longer or fall back to a hardware reset. */
#define SDIO_REINIT_RETRY_COUNT 20U

/* CCCR Bus Interface Control (function 0, register 0x07) */
#define SDIO_CCCR_BUS_IF_CTRL   0x07U
#define SDIO_CCCR_BUS_WIDTH_MSK 0x03U
#define SDIO_CCCR_BUS_WIDTH_1B  0x00U
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

#if (SDIO_PORT_SLOW_KERNEL_CLK != 0)
    /* CLKDIV tops out at 1023, so a 250 MHz kernel clock cannot go below
       122 kHz - and the link is only known to carry a frame at 25 kHz. PLL2R at
       50 MHz moves the whole range down by five: div=1023 is 24 kHz, div=312 is
       80 kHz. Same CSI source as PLL1, so no extra oscillator is needed:
       4 MHz / 1 = 4 MHz in, x100 = 400 MHz VCO, / 8 = 50 MHz. */
    PeriphClkInitStruct.Sdmmc1ClockSelection = RCC_SDMMC1CLKSOURCE_PLL2R;
    PeriphClkInitStruct.PLL2.PLL2Source   = RCC_PLL2_SOURCE_CSI;
    PeriphClkInitStruct.PLL2.PLL2M        = 1;
    PeriphClkInitStruct.PLL2.PLL2N        = 100;
    PeriphClkInitStruct.PLL2.PLL2P        = 2;
    PeriphClkInitStruct.PLL2.PLL2Q        = 2;
    PeriphClkInitStruct.PLL2.PLL2R        = 8;
    PeriphClkInitStruct.PLL2.PLL2RGE      = RCC_PLL2_VCIRANGE_2;
    PeriphClkInitStruct.PLL2.PLL2VCOSEL   = RCC_PLL2_VCORANGE_WIDE;
    PeriphClkInitStruct.PLL2.PLL2FRACN    = 0;
    PeriphClkInitStruct.PLL2.PLL2ClockOut = RCC_PLL2_DIVR;
#else
    PeriphClkInitStruct.Sdmmc1ClockSelection = RCC_SDMMC1CLKSOURCE_PLL1Q;
#endif

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
    GPIO_InitStruct.Pin = GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = SDIO_PORT_GPIO_SPEED;
    GPIO_InitStruct.Alternate = GPIO_AF12_SDMMC1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* CMD alone may be open-drain - see SDIO_PORT_CMD_OPEN_DRAIN */
    GPIO_InitStruct.Pin = GPIO_PIN_2;
#if (SDIO_PORT_CMD_OPEN_DRAIN != 0)
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
#endif
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
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

/**
  * @brief SDMMC1 interrupt entry.
  *
  * SDMMC_STA_SDIOIT follows the D1 level and has no write-1-to-clear: masking
  * the source is the only way out of the handler. HAL_SDIO_IRQHandler() does
  * that from the IO-function callback, but SDIO_IOFunction_IRQHandler() only
  * reaches the callback while IOInterruptNbr == 1. HAL_SDIO_DeInit() never
  * resets that counter, so the second HAL_SDIO_EnableIOFunctionInterrupt() -
  * the one in the re-enumeration after AT+RESTORE - takes it to 2, and from
  * then on every interrupt goes down a path that first issues a CMD52 from
  * inside the ISR. Against a slave that is still rebooting that read fails, the
  * handler returns without masking anything, and the level re-enters it
  * immediately: the main loop never runs again. Measured on the bench as a
  * permanent hang with PC inside this vector (IPSR 95).
  *
  * Every transfer this port issues is polled, so SDIOIT is the only reason this
  * interrupt is enabled at all and it is answered here rather than in the HAL.
  */
void SdioPortIrqHandler(void)
{
    if (__HAL_SDIO_GET_FLAG(&hsdio1, SDMMC_FLAG_SDIOIT) != 0U) {
        __HAL_SDIO_DISABLE_IT(&hsdio1, SDMMC_IT_SDIOIT);
        esp_int_flag = 1U;
        return;
    }

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

    gpio.Pin = GPIO_PIN_13;
    HAL_GPIO_Init(GPIOB, &gpio);

    gpio.Pin = GPIO_PIN_2;
#if (SDIO_PORT_CMD_OPEN_DRAIN != 0)
    gpio.Mode = GPIO_MODE_AF_OD;
#endif
    HAL_GPIO_Init(GPIOB, &gpio);

    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pin = GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12;
    HAL_GPIO_Init(GPIOC, &gpio);
}

/**
  * @brief CMD0 then CMD5 driven by hand, with the SDMMC out of the picture.
  *
  * Every host-side explanation for the timeout has been ruled out with the CPSM
  * in the loop, which still leaves three things it could be hiding: the edge the
  * frame is driven on, the 64-clock window the CPSM gives up after, and whether
  * the CPSM releases CMD in time for the slave to answer at all. Bit-banging
  * removes all three. The clock is ~25 kHz and the response window is 200 clocks
  * instead of 64, so anything the slave sends will be caught.
  *
  * A response here with none from the CPSM is a host configuration bug. Silence
  * here too means the slave is not decoding the frame, and no host setting will
  * fix that.
  */
/* Half-period in core cycles at 250 MHz; 5000 = 25 kHz, 312 = 400 kHz */
static uint32_t sdio_bb_half_cycles = 5000U;

/* Idle clocks sent with CMD high before CMD0. The SD spec wants at least 74
   after power-up; the question is whether the slave insists on them. */
static uint32_t sdio_bb_idle_clocks = 80U;

/* Result of the same probe with the idle clocks suppressed */
volatile uint32_t sdio_bb_noidle = 0U;

/* Enumeration outcome, as a magic value so it is recognisable in a raw memory
   dump even if the symbol table and the running image disagree on layout. */
#define SDIO_MARK_ENUM_OK    0xC0FFEE01U
#define SDIO_MARK_DRIVER_OK  0xC0FFEE02U
#define SDIO_MARK_ENUM_FAIL  0xDEAD0001U
volatile uint32_t sdio_enum_mark = 0U;

/* How far SdioIdentifyCardSlow() got on its last run: 1 entered, 2 CMD0 ok,
   3 inquiry CMD5 ok, 4 slave reported ready, 5 CMD3 ok, 6 CMD7 ok. */
volatile uint32_t sdio_ident_step = 0U;
volatile uint32_t sdio_ident_resp4 = 0U;
volatile uint32_t sdio_enum_retries = 0U;

/* Mirrored out of the log so the result can be read over SWD when the VCP is
   taken: [0] 0=not run 1=answered 2=silent, [1] clocks to the start bit,
   [2]/[3] the R4 bytes. Not static - it has to survive into the map file. */
volatile uint32_t sdio_bb_result[4] = {0U, 0U, 0U, 0U};

/* Answer count per CLKCR phase combination, same reason. Index is
   bit0 = NEGEDGE, bit1 = HWFC_EN. */
volatile uint32_t sdio_phase_result[4] = {0U, 0U, 0U, 0U};

/* Bit-banged CMD5 answered (1) or not (0) at 25/50/100/200/400 kHz. Finds the
   clock the interconnect actually stops carrying, which the CPSM cannot reach:
   its slowest cell is 122 kHz off a 250 MHz kernel clock. */
volatile uint32_t sdio_bb_freq_khz[8] = {0U};
volatile uint32_t sdio_bb_freq_ok[8] = {0U};

static void SdioBbDelay(void)
{
    uint32_t start = DWT->CYCCNT;

    while ((DWT->CYCCNT - start) < sdio_bb_half_cycles) {
    }
}

static uint8_t SdioCrc7(const uint8_t *data, uint32_t len)
{
    uint8_t crc = 0U;
    uint32_t i;
    uint32_t j;

    for (i = 0U; i < len; i++) {
        uint8_t b = data[i];

        for (j = 0U; j < 8U; j++) {
            crc = (uint8_t)(crc << 1);
            if (((b ^ crc) & 0x80U) != 0U) {
                crc ^= 0x09U;
            }
            b = (uint8_t)(b << 1);
        }
    }

    return (uint8_t)(crc & 0x7FU);
}

/* CMD is driven while CK is low, so the slave latches it on the rising edge -
   the "send at negedge, sample at posedge" default the ESP slave documents. */
static void SdioBbClock(uint32_t cmd_bit)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, (cmd_bit != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    SdioBbDelay();
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET);
    SdioBbDelay();
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET);
}

static void SdioBbSendFrame(uint8_t index, uint32_t arg)
{
    uint8_t frame[6];
    uint32_t i;
    uint32_t j;

    frame[0] = (uint8_t)(0x40U | (index & 0x3FU));
    frame[1] = (uint8_t)(arg >> 24);
    frame[2] = (uint8_t)(arg >> 16);
    frame[3] = (uint8_t)(arg >> 8);
    frame[4] = (uint8_t)arg;
    frame[5] = (uint8_t)((SdioCrc7(frame, 5U) << 1) | 1U);

    for (i = 0U; i < 6U; i++) {
        for (j = 0U; j < 8U; j++) {
            SdioBbClock((uint32_t)(frame[i] >> (7U - j)) & 1U);
        }
    }
}

static void SdioBitBangProbe(void)
{
    GPIO_InitTypeDef gpio = {0};
    uint32_t i;
    uint32_t start_clk = 0U;
    uint32_t got_start = 0U;
    uint8_t  resp[6] = {0};

    /* Free-running cycle counter for the sub-millisecond delays */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    if ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0U) {
        SDIO_LOGE(TAG, "bitbang: no cycle counter, skipped");
        return;
    }

    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Pin   = GPIO_PIN_12;
    HAL_GPIO_Init(GPIOC, &gpio);
    gpio.Pin   = GPIO_PIN_2;
    HAL_GPIO_Init(GPIOB, &gpio);

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET);

    /* Idle clocks with CMD high, as the card expects before CMD0 */
    for (i = 0U; i < sdio_bb_idle_clocks; i++) {
        SdioBbClock(1U);
    }

    SdioBbSendFrame(0U, 0U);          /* CMD0 - no response expected */
    for (i = 0U; i < 16U; i++) {
        SdioBbClock(1U);
    }

    SdioBbSendFrame(5U, 0U);          /* CMD5 - R4 */

    /* Release CMD to the pull-up so the slave can drive it */
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_NOPULL;
    gpio.Pin  = GPIO_PIN_2;
    HAL_GPIO_Init(GPIOB, &gpio);

    for (i = 0U; i < 200U; i++) {
        SdioBbDelay();
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET);
        SdioBbDelay();
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2) == GPIO_PIN_RESET) {
            got_start = 1U;
            start_clk = i;
        }
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET);
        if (got_start != 0U) {
            break;
        }
    }

    if (got_start != 0U) {
        /* Start bit already consumed; take the remaining 47 of the R4 frame */
        for (i = 0U; i < 47U; i++) {
            SdioBbDelay();
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET);
            SdioBbDelay();
            if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2) != GPIO_PIN_RESET) {
                resp[(i + 1U) / 8U] |= (uint8_t)(0x80U >> ((i + 1U) % 8U));
            }
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET);
        }

        sdio_bb_result[0] = 1U;
        sdio_bb_result[1] = start_clk;
        sdio_bb_result[2] = ((uint32_t)resp[0] << 24) | ((uint32_t)resp[1] << 16) |
                            ((uint32_t)resp[2] << 8) | (uint32_t)resp[3];
        sdio_bb_result[3] = ((uint32_t)resp[4] << 8) | (uint32_t)resp[5];

        SDIO_LOGE(TAG, "bitbang: slave answered CMD5 after %lu clocks - "
                       "%02x %02x %02x %02x %02x %02x",
                  (unsigned long)start_clk, resp[0], resp[1], resp[2],
                  resp[3], resp[4], resp[5]);
        SDIO_LOGE(TAG, "  the slave decodes a hand-built frame, so the fault is "
                       "in the SDMMC setup, not the link");
    } else {
        sdio_bb_result[0] = 2U;
        SDIO_LOGE(TAG, "bitbang: no start bit in 200 clocks after CMD5");
        SDIO_LOGE(TAG, "  the slave ignores a frame built by hand at 25 kHz with "
                       "a 200-clock window, so it is not decoding CMD at all - "
                       "stop tuning the host");
    }

    SdioSetPadSpeed(SDIO_PORT_GPIO_SPEED);
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

static uint32_t sdio_probe_answered;

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

    sdio_probe_answered = answered;

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
#if (SDIO_PORT_ESP_RST_BOOT_MS != 0)
/**
  * @brief Pulse the ESP32-C6 EN pin, then wait for esp-at to come up.
  *
  * Puts the slave in a known state at every host start, which is what makes
  * enumeration repeatable: the software CCCR reset only reaches a slave still
  * willing to decode CMD52, this reaches one that is not.
  */
static void SdioResetSlaveHw(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();

    gpio.Pin   = SDIO_PORT_ESP_RST_PIN;
    gpio.Mode  = GPIO_MODE_OUTPUT_OD;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(SDIO_PORT_ESP_RST_PORT, &gpio);

    HAL_GPIO_WritePin(SDIO_PORT_ESP_RST_PORT, SDIO_PORT_ESP_RST_PIN, GPIO_PIN_RESET);
    HAL_Delay(SDIO_PORT_ESP_RST_LOW_MS);
    /* Open-drain high is a release, not a drive - the devkit pull-up takes it */
    HAL_GPIO_WritePin(SDIO_PORT_ESP_RST_PORT, SDIO_PORT_ESP_RST_PIN, GPIO_PIN_SET);

    SDIO_LOGI(TAG, "slave held in reset %u ms, waiting %u ms for esp-at",
              (unsigned int)SDIO_PORT_ESP_RST_LOW_MS,
              (unsigned int)SDIO_PORT_ESP_RST_BOOT_MS);
    HAL_Delay(SDIO_PORT_ESP_RST_BOOT_MS);
}
#endif /* SDIO_PORT_ESP_RST_BOOT_MS */

/**
  * @brief CMD52 write of the CCCR RES bit - the only reset an I/O card honours.
  *
  * CMD0 is a memory-card command and an I/O-only slave is free to ignore it, so
  * a host that restarts while the slave keeps running meets a card still holding
  * the RCA and enabled function from the previous session, quietly ignoring
  * CMD5. No amount of retrying breaks that loop; a write of RES to CCCR 0x06
  * does, and it is harmless on a slave that is already idle.
  *
  * The response is deliberately not checked: a wedged slave will not send one,
  * and it acts on the write either way.
  */
static void SdioResetSlaveIo(void)
{
    SDMMC_CmdInitTypeDef cmd = {0};
    uint32_t tickstart;

    if (SDMMC_GetPowerState(hsdio1.Instance) == 0U) {
        return;
    }

    /* R/W=1, function 0, RAW=0, address 0x06 (I/O Abort), data 0x08 (RES) */
    cmd.Argument         = 0x80000C08U;
    cmd.CmdIndex         = SDMMC_CMD_SDMMC_RW_DIRECT;
    cmd.Response         = SDMMC_RESPONSE_SHORT;
    cmd.WaitForInterrupt = SDMMC_WAIT_NO;
    cmd.CPSM             = SDMMC_CPSM_ENABLE;
    (void)SDMMC_SendCommand(hsdio1.Instance, &cmd);

    tickstart = HAL_GetTick();
    while (((hsdio1.Instance->STA & (SDMMC_FLAG_CMDREND | SDMMC_FLAG_CTIMEOUT |
                                     SDMMC_FLAG_CCRCFAIL)) == 0U) &&
           ((HAL_GetTick() - tickstart) < 10U)) {
    }
    __SDMMC_CLEAR_FLAG(hsdio1.Instance, SDMMC_STATIC_CMD_FLAGS);

    /* The slave needs a moment to fall back to the pre-init state */
    HAL_Delay(20U);
}

/**
  * @brief Does the CPSM actually put a frame on CMD, and does CK really run?
  *
  * Everything else in the probe only says "no response", which cannot tell a
  * silent slave from a host that never transmitted. The AF does not disable the
  * input path, so IDR still shows what the pad is doing: kick a CMD0 off
  * without waiting for it, then sample both pads flat out for longer than the
  * 48-bit frame lasts. CK is the control - it free-runs, so a run of all-high
  * there means the sampling itself is broken.
  */
static void SdioProbeLineActivity(void)
{
    SDMMC_CmdInitTypeDef cmd = {0};
    uint32_t i;
    uint32_t cmd_low = 0U;
    uint32_t ck_low = 0U;
    uint32_t d3_low = 0U;

    cmd.Argument         = 0U;
    cmd.CmdIndex         = SDMMC_CMD_GO_IDLE_STATE;
    cmd.Response         = SDMMC_RESPONSE_NO;
    cmd.WaitForInterrupt = SDMMC_WAIT_NO;
    cmd.CPSM             = SDMMC_CPSM_ENABLE;
    (void)SDMMC_SendCommand(hsdio1.Instance, &cmd);

    /* ~20k reads of two IDRs outlasts a 48-bit frame at 400 kHz by a wide margin */
    for (i = 0U; i < 20000U; i++) {
        if ((GPIOB->IDR & GPIO_PIN_2) == 0U) {
            cmd_low++;
        }
        if ((GPIOC->IDR & GPIO_PIN_12) == 0U) {
            ck_low++;
        }
        /* DAT3 is CS at CMD0 time: low selects SPI mode, and an SDIO slave in
           SPI mode ignores CMD5 forever. It must stay high through the frame. */
        if ((GPIOC->IDR & GPIO_PIN_11) == 0U) {
            d3_low++;
        }
    }

    __SDMMC_CLEAR_FLAG(hsdio1.Instance, SDMMC_STATIC_CMD_FLAGS);

    SDIO_LOGE(TAG, "probe: line activity during CMD0 - CMD low %lu/%u, "
                   "CK low %lu/%u, D3 low %lu/%u", (unsigned long)cmd_low, 20000U,
              (unsigned long)ck_low, 20000U, (unsigned long)d3_low, 20000U);

    if (d3_low != 0U) {
        SDIO_LOGE(TAG, "  D3/CS went low during CMD0 - that latches the slave "
                       "into SPI mode, where CMD5 is never answered. D3 must "
                       "stay high until enumeration is done");
    }

    if (ck_low == 0U) {
        SDIO_LOGE(TAG, "  CK never sampled low - the clock is not running, so "
                       "nothing below this line means anything");
    } else if (cmd_low == 0U) {
        SDIO_LOGE(TAG, "  CMD never left the pull-up while CK ran - the CPSM is "
                       "not driving PB2, so the slave has nothing to answer");
    } else {
        SDIO_LOGE(TAG, "  host transmits: CK toggles and CMD is pulled low "
                       "inside the frame, so the timeout is at the slave");
    }
}

/**
  * @brief CMD5 at each CLKCR sampling phase, with everything else held still.
  *
  * The slave answers a bit-banged CMD5 with NCR=2, the spec minimum, so the CPSM
  * has no slack: half a cycle of error in where it turns CMD around or samples
  * the reply and it misses the start bit and reports a plain timeout. NEGEDGE
  * moves exactly that phase, and HWFC_EN is swept with it because it also gates
  * when the CPSM releases the line.
  */
static void SdioProbeClkPhase(uint32_t base_div)
{
    static const char *const phase_name[4] = {
        "posedge no-hwfc", "negedge no-hwfc", "posedge hwfc", "negedge hwfc"
    };
    uint32_t i;

    SDIO_LOGE(TAG, "probe: CLKCR sampling phase");

    for (i = 0U; i < 4U; i++) {
        uint32_t clkcr = hsdio1.Instance->CLKCR;

        clkcr &= ~(SDMMC_CLKCR_NEGEDGE | SDMMC_CLKCR_HWFC_EN);
        if ((i & 1U) != 0U) {
            clkcr |= SDMMC_CLKCR_NEGEDGE;
        }
        if ((i & 2U) != 0U) {
            clkcr |= SDMMC_CLKCR_HWFC_EN;
        }
        hsdio1.Instance->CLKCR = clkcr;
        HAL_Delay(2U);

        SdioProbeCmd5(base_div, 0U, phase_name[i]);
        sdio_phase_result[i] = sdio_probe_answered;
    }

    /* Back to the configured phase so nothing downstream inherits the sweep */
    MODIFY_REG(hsdio1.Instance->CLKCR,
               SDMMC_CLKCR_NEGEDGE | SDMMC_CLKCR_HWFC_EN,
               hsdio1.Init.ClockEdge | hsdio1.Init.HardwareFlowControl);
}

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
    SdioProbeLineActivity();
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

    SdioProbeClkPhase(base_div);

    /* Does the slave insist on the 74 idle clocks the spec asks for after
       power-up? The HAL sends CMD0 straight after powering the CPSM on, so if
       it does, the CPSM can never enumerate no matter how it is tuned. */
    sdio_bb_idle_clocks = 0U;
    sdio_bb_result[0] = 0U;
    SdioBitBangProbe();
    sdio_bb_noidle = sdio_bb_result[0];
    sdio_bb_idle_clocks = 80U;
    sdio_bb_result[0] = 0U;
    SdioBitBangProbe();
    SDIO_LOGE(TAG, "  bitbang idle-clock A/B: without=%s with=%s",
              (sdio_bb_noidle == 1U) ? "answered" : "silent",
              (sdio_bb_result[0] == 1U) ? "answered" : "silent");

    /* Where does the interconnect actually stop carrying a frame? */
    {
        static const uint32_t half[5] = { 5000U, 2500U, 1250U, 625U, 312U };
        static const uint32_t khz[5]  = { 25U, 50U, 100U, 200U, 400U };
        uint32_t f;

        for (f = 0U; f < 5U; f++) {
            sdio_bb_half_cycles = half[f];
            sdio_bb_result[0] = 0U;
            SdioBitBangProbe();
            sdio_bb_freq_khz[f] = khz[f];
            sdio_bb_freq_ok[f] = (sdio_bb_result[0] == 1U) ? 1U : 0U;
            SDIO_LOGE(TAG, "  bitbang %lu kHz: %s", (unsigned long)khz[f],
                      (sdio_bb_result[0] == 1U) ? "answered" : "silent");
        }
        sdio_bb_half_cycles = 5000U;
    }

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
    /* One net at a time, so whoever is watching the slave can tell which host
       pin a given ESP32-C6 pad is actually wired to. Expected far end:
       CK->GPIO19, CMD->GPIO18, D0->GPIO20, D1->GPIO21, D2->GPIO22, D3->GPIO23 */
    static const struct {
        const char   *name;
        const char   *far_end;
        GPIO_TypeDef *port;
        uint16_t      pin;
    } sweep[] = {
        { "CK(PC12)",  "GPIO19", GPIOC, GPIO_PIN_12 },
        { "CMD(PB2)",  "GPIO18", GPIOB, GPIO_PIN_2  },
        { "D0(PB13)",  "GPIO20", GPIOB, GPIO_PIN_13 },
        { "D1(PC9)",   "GPIO21", GPIOC, GPIO_PIN_9  },
        { "D2(PC10)",  "GPIO22", GPIOC, GPIO_PIN_10 },
        { "D3(PC11)",  "GPIO23", GPIOC, GPIO_PIN_11 },
    };
    GPIO_InitTypeDef gpio = {0};
    uint32_t i;
    uint32_t end;

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;

    for (i = 0U; i < (sizeof(sweep) / sizeof(sweep[0])); i++) {
        gpio.Pin = sweep[i].pin;
        HAL_GPIO_Init(sweep[i].port, &gpio);
        HAL_GPIO_WritePin(sweep[i].port, sweep[i].pin, GPIO_PIN_SET);

        SDIO_LOGE(TAG, "sweep: %s alone at 2 Hz for %u ms - expect it at "
                       "ESP32-C6 %s", sweep[i].name,
                  (unsigned int)SDIO_PORT_CK_TOGGLE_MS, sweep[i].far_end);

        end = HAL_GetTick() + SDIO_PORT_CK_TOGGLE_MS;
        while (HAL_GetTick() < end) {
            HAL_GPIO_TogglePin(sweep[i].port, sweep[i].pin);
            HAL_Delay(250U);
        }

        /* Release the net (pull-ups take it high) and leave a quiet gap so the
           next window is unambiguous */
        HAL_GPIO_WritePin(sweep[i].port, sweep[i].pin, GPIO_PIN_SET);
        gpio.Mode = GPIO_MODE_INPUT;
        HAL_GPIO_Init(sweep[i].port, &gpio);
        gpio.Mode = GPIO_MODE_OUTPUT_PP;
        HAL_Delay(1500U);
    }
}
#endif /* SDIO_PORT_CK_TOGGLE_MS */

/**
  * @brief Agree the data bus width with the slave once enumeration has completed.
  *
  * HAL_SDIO_Init() compares hsdio->Init.BusWide (an SDMMC_BUS_WIDE_* register
  * value) against HAL_SDIO_4_WIRES_MODE (1), so what it writes into CCCR 0x07
  * has nothing to do with the width actually asked for. The setting is made
  * here instead: CMD52 travels on CMD only, so it is unaffected by the width,
  * and CLKCR is moved only after the slave has acknowledged the new value.
  *
  * The 1-bit case is written out too rather than assumed. A slave that was left
  * in 4-bit by a previous host session keeps that setting until it is reset, and
  * a host that has meanwhile dropped to 1-bit would then read nothing but data
  * CRC failures.
  */
static sdio_err_t SdioSetBusWidth(void)
{
#if (SDIO_PORT_BUS_WIDTH == HAL_SDIO_4_WIRES_MODE)
    const uint8_t want = SDIO_CCCR_BUS_WIDTH_4B;
    const uint32_t widbus = SDMMC_BUS_WIDE_4B;
#else
    const uint8_t want = SDIO_CCCR_BUS_WIDTH_1B;
    const uint32_t widbus = SDMMC_BUS_WIDE_1B;
#endif
    uint8_t val;

    if (sdio_driver_read_byte(0U, SDIO_CCCR_BUS_IF_CTRL, &val) != SDIO_SUCCESS) {
        SDIO_LOGE(TAG, "CCCR 0x07 read failed");
        return FAILURE;
    }

    val = (uint8_t)((val & ~SDIO_CCCR_BUS_WIDTH_MSK) | want);
    if (sdio_driver_write_byte(0U, SDIO_CCCR_BUS_IF_CTRL, val, &val) != SDIO_SUCCESS) {
        SDIO_LOGE(TAG, "CCCR 0x07 write failed");
        return FAILURE;
    }

    if ((val & SDIO_CCCR_BUS_WIDTH_MSK) != want) {
        SDIO_LOGE(TAG, "slave refused %u-bit, CCCR 0x07 = 0x%02x",
                  (want == SDIO_CCCR_BUS_WIDTH_4B) ? 4U : 1U, (unsigned int)val);
        return FAILURE;
    }

    MODIFY_REG(hsdio1.Instance->CLKCR, SDMMC_CLKCR_WIDBUS, widbus);
    hsdio1.Init.BusWide = widbus;

    SDIO_LOGI(TAG, "bus width set to %u-bit",
              (want == SDIO_CCCR_BUS_WIDTH_4B) ? 4U : 1U);
    return SDIO_SUCCESS;
}

/* CMD5 argument: 3.2-3.3V window, no 1.8V switch request */
#define SDIO_IDENT_OCR_VDD_32_33  (1UL << 20U)

/* Per-command retries inside identification. The link drops frames at a low
   rate; retrying the command is far cheaper than restarting enumeration. */
#define SDIO_IDENT_CMD_RETRY      32U

/**
  * @brief Replacement for the HAL identification step, run at the port's clock.
  *
  * HAL_SDIO_Init() recomputes Init.ClockDiv from its private SDIO_INIT_FREQ
  * (400 kHz) and enumerates there, throwing away hsdio1.Init.ClockDiv - it only
  * restores it after identification succeeds. This link does not carry a frame
  * at 400 kHz: the bit-banged probe reads a valid R4 at 25 kHz and nothing at
  * 400 kHz, which is why every clock, slew and phase sweep changed nothing. They
  * were all outside the window the HAL re-initialises.
  *
  * SDIO_IdentifyCard is a documented hook and the HAL only defaults it when it
  * is NULL, so this runs the same CMD0/CMD5/CMD3/CMD7 sequence with the clock
  * put back to SDIO_PORT_CLOCK_HZ first, plus the 74 idle cycles the slave turns
  * out to insist on - a bit-banged CMD5 with them is answered, without them it
  * is not.
  */
static HAL_StatusTypeDef SdioIdentifyCardSlow(SDIO_HandleTypeDef *hsdio)
{
    uint32_t errorstate;
    uint32_t resp4 = 0U;
    uint32_t timeout;
    uint16_t rca = 1U;

    sdio_ident_step = 1U;

    /* Undo the HAL's 400 kHz and give the slave its power-up clocks at ours */
    MODIFY_REG(hsdio->Instance->CLKCR, SDMMC_CLKCR_CLKDIV, hsdio->Init.ClockDiv);
    HAL_Delay(1U + (74U * 1000U / SDIO_PORT_CLOCK_HZ) + 1U);

    for (timeout = 0U; timeout < SDIO_IDENT_CMD_RETRY; timeout++) {
        if (SDMMC_CmdGoIdleState(hsdio->Instance) == HAL_SDIO_ERROR_NONE) {
            break;
        }
        HAL_Delay(2U);
    }

    if (timeout >= SDIO_IDENT_CMD_RETRY) {
        return HAL_ERROR;
    }

    sdio_ident_step = 2U;

    if (SDMMC_GetPowerState(hsdio->Instance) == 0U) {
        return HAL_ERROR;
    }

    /* Inquiry CMD5 - arg 0 never sets C, it only reports the OCR.

       Retried here rather than upwards: a single timed-out command used to fail
       the whole HAL_SDIO_Init(), which tears the card back down to RESET and
       re-runs CMD0 from scratch. On a link that drops the occasional frame that
       turns a recoverable hiccup into a total failure, and the step markers show
       exactly that - runs that failed at this command had already seen a ready
       R4 from an earlier attempt in the same batch. */
    for (timeout = 0U; timeout < SDIO_IDENT_CMD_RETRY; timeout++) {
        if (SDMMC_CmdSendOperationcondition(hsdio->Instance, 0U, &resp4) ==
            HAL_SDIO_ERROR_NONE) {
            break;
        }
        HAL_Delay(2U);
        (void)SDMMC_CmdGoIdleState(hsdio->Instance);
        HAL_Delay(2U);
    }

    if (timeout >= SDIO_IDENT_CMD_RETRY) {
        return HAL_ERROR;
    }

    sdio_ident_step = 3U;
    sdio_ident_resp4 = resp4;

    if (((resp4 & 0x70000000U) >> 28U) == 0U) {
        return HAL_ERROR;      /* no I/O function - not an SDIO card */
    }

    /* Now poll with the voltage window until the slave reports ready. The HAL
       asks once and gives up, which loses the race whenever the slave is still
       initialising. */
    for (timeout = 0U; timeout < 1000U; timeout++) {
        if (SDMMC_CmdSendOperationcondition(hsdio->Instance,
                                            SDIO_IDENT_OCR_VDD_32_33,
                                            &resp4) == HAL_SDIO_ERROR_NONE) {
            if ((resp4 & 0x80000000U) != 0U) {
                break;
            }
        }
        HAL_Delay(1U);
    }

    if ((resp4 & 0x80000000U) == 0U) {
        return HAL_ERROR;
    }

    sdio_ident_step = 4U;
    sdio_ident_resp4 = resp4;

    /* CMD0 before CMD3 makes CMD3 illegal for a while; the HAL retries and so
       does this. */
    timeout = 0U;
    do {
        errorstate = SDMMC_CmdSetRelAdd(hsdio->Instance, &rca);
        timeout++;
        HAL_Delay(1U);
    } while ((errorstate != HAL_SDIO_ERROR_NONE) && (timeout < 1000U));

    if (errorstate != HAL_SDIO_ERROR_NONE) {
        return HAL_ERROR;
    }

    sdio_ident_step = 5U;

    for (timeout = 0U; timeout < SDIO_IDENT_CMD_RETRY; timeout++) {
        if (SDMMC_CmdSelDesel(hsdio->Instance, ((uint32_t)rca) << 16U) ==
            HAL_SDIO_ERROR_NONE) {
            break;
        }
        HAL_Delay(2U);
    }

    if (timeout >= SDIO_IDENT_CMD_RETRY) {
        return HAL_ERROR;
    }

    sdio_ident_step = 6U;
    return HAL_OK;
}

static sdio_err_t SdioDriverInit(uint8_t reset_slave)
{
    uint32_t kernel_clk;
    uint32_t retry;
    uint32_t init_retry_count = (reset_slave != 0U) ? SDIO_INIT_RETRY_COUNT
                                                    : SDIO_REINIT_RETRY_COUNT;
    HAL_StatusTypeDef status = HAL_ERROR;

    if (hsdio1.Instance == SDMMC1) {
        HAL_NVIC_DisableIRQ(SDMMC1_IRQn);
        (void)HAL_SDIO_DeInit(&hsdio1);
    }
    esp_int_flag = 0U;
    /* Whatever the caller was suppressing, a failure to re-enumerate is real */
    sdio_quiet_errors = 0U;

    /* HAL_SDIO_DeInit() resets State and ErrorCode and nothing else, so the
       IO-interrupt bookkeeping survives into the next enumeration: the counter
       would reach 2 on the second EnableIOFunctionInterrupt() and put the ISR
       on the CMD52-from-interrupt path for good. See SdioPortIrqHandler(). */
    hsdio1.IOInterruptNbr = 0U;
    hsdio1.IOFunctionMask = 0U;

    if (reset_slave != 0U) {
#if (SDIO_PORT_ESP_RST_BOOT_MS != 0)
        /* Before anything touches the bus, so the slave is always freshly booted */
        SdioResetSlaveHw();
#endif
    }

    /* The sweep runs first on cold boot: it is the only test that says where a
       wire actually lands, and a net the census rejects is exactly when that
       matters most. Skip it on reinit after AT+RESTORE; the slave is rebooting
       itself and the host must not remux or drive the SDIO pins as GPIOs. */
    if (reset_slave != 0U) {
#if (SDIO_PORT_CK_TOGGLE_MS != 0)
        SdioToggleCk();
#endif
#if (SDIO_PORT_PIN_DIAG != 0)
        /* No point clocking the bus when the host cannot even assert CMD */
        if (SdioPinDiag() != 0U) {
            return FAILURE;
        }
#endif
    }

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
    /* Claim the identification step before the HAL can default it */
    hsdio1.SDIO_IdentifyCard = SdioIdentifyCardSlow;

    for (retry = 0U; retry < init_retry_count; retry++) {
        status = HAL_SDIO_Init(&hsdio1);
        if (status == HAL_OK) {
            break;
        }
        SdioResetSlaveIo();
        HAL_Delay(SDIO_INIT_RETRY_MS);
    }

    sdio_enum_retries = retry;
    sdio_enum_mark = (status == HAL_OK) ? SDIO_MARK_ENUM_OK : SDIO_MARK_ENUM_FAIL;

    if (status != HAL_OK) {
        /* SDIO_InitCard() returns HAL_ERROR without recording an ErrorCode, so
           re-probe CMD0/CMD5 here to say which half of the link is at fault. */
        SDIO_LOGE(TAG, "HAL_SDIO_Init failed after %lu attempts, err=0x%08lx",
                  (unsigned long)init_retry_count,
                  (unsigned long)HAL_SDIO_GetError(&hsdio1));
        if (reset_slave != 0U) {
            SdioProbeSlave();
        }
        return FAILURE;
    }

    if (retry > 0U) {
        SDIO_LOGI(TAG, "slave enumerated after %lu retries (%lu ms)",
                  (unsigned long)retry,
                  (unsigned long)(retry * SDIO_INIT_RETRY_MS));
    }

    /* Everything below is a CMD52 on the same link that loses the occasional
       frame during identification, and a single miss here used to fail the whole
       init even though enumeration had succeeded. Same answer as there: retry
       the command rather than the sequence. */
    for (retry = 0U; retry < SDIO_IDENT_CMD_RETRY; retry++) {
        if (SdioSetBusWidth() == SDIO_SUCCESS) {
            break;
        }
        HAL_Delay(2U);
    }

    if (retry >= SDIO_IDENT_CMD_RETRY) {
        SDIO_LOGE(TAG, "set bus width failed");
        return FAILURE;
    }

    if (HAL_SDIO_RegisterIOFunctionCallback(&hsdio1, SDIO_PORT_FUNCTION,
                                            EspIoInterruptCallback) != HAL_OK) {
        return FAILURE;
    }

    for (retry = 0U; retry < SDIO_IDENT_CMD_RETRY; retry++) {
        if (HAL_SDIO_EnableIOFunction(&hsdio1, SDIO_PORT_FUNCTION) == HAL_OK) {
            break;
        }
        HAL_Delay(2U);
    }

    if (retry >= SDIO_IDENT_CMD_RETRY) {
        SDIO_LOGE(TAG, "enable IO function 1 failed");
        return FAILURE;
    }

    /* esp_slave_init_io() programs the CCCR/FBR block size with CMD52. Do not
       duplicate it here with HAL_SDIO_SetBlockSize(): that HAL path writes the
       same two FBR bytes with CMD53 byte mode, which is exactly the transfer
       type this port avoids for short register access during recovery. */
    hsdio1.block_size = SDIO_PORT_BLOCK_SIZE;

    for (retry = 0U; retry < SDIO_IDENT_CMD_RETRY; retry++) {
        if (HAL_SDIO_EnableIOFunctionInterrupt(&hsdio1, SDIO_PORT_FUNCTION) == HAL_OK) {
            break;
        }
        HAL_Delay(2U);
    }

    if (retry >= SDIO_IDENT_CMD_RETRY) {
        SDIO_LOGE(TAG, "enable IO function 1 interrupt failed");
        return FAILURE;
    }

    sdio_enum_mark = SDIO_MARK_DRIVER_OK;
    return SDIO_SUCCESS;
}

sdio_err_t sdio_driver_init(void)
{
    return SdioDriverInit(1U);
}

void sdio_driver_set_quiet(uint8_t quiet)
{
    sdio_quiet_errors = quiet;
}

sdio_err_t sdio_driver_reinit(void)
{
    return SdioDriverInit(0U);
}

static sdio_err_t SdioReadByte(uint32_t function, uint32_t reg, uint8_t *out_byte,
                               uint8_t log_error)
{
    HAL_SDIO_DirectCmd_TypeDef cmd52;

    cmd52.Reg_Addr = reg;
    cmd52.ReadAfterWrite = 0U;
    cmd52.IOFunctionNbr = function & 0x7U;

    if (HAL_SDIO_ReadDirect(&hsdio1, &cmd52, out_byte) != HAL_OK) {
        if (log_error != 0U) {
            SDIO_LOGE(TAG, "CMD52 read error, func %lu reg 0x%lx",
                      (unsigned long)function, (unsigned long)reg);
        }
        return FAILURE;
    }
    return SDIO_SUCCESS;
}

sdio_err_t sdio_driver_read_byte(uint32_t function, uint32_t reg, uint8_t *out_byte)
{
    return SdioReadByte(function, reg, out_byte, 1U);
}

static sdio_err_t SdioWriteByte(uint32_t function, uint32_t reg, uint8_t in_byte,
                                uint8_t log_error)
{
    HAL_SDIO_DirectCmd_TypeDef cmd52;

    cmd52.Reg_Addr = reg;
    cmd52.ReadAfterWrite = 0U;
    cmd52.IOFunctionNbr = function & 0x7U;

    if (HAL_SDIO_WriteDirect(&hsdio1, &cmd52, in_byte) != HAL_OK) {
        if (log_error != 0U) {
            SDIO_LOGE(TAG, "CMD52 write error, func %lu reg 0x%lx",
                      (unsigned long)function, (unsigned long)reg);
        }
        return FAILURE;
    }
    return SDIO_SUCCESS;
}

sdio_err_t sdio_driver_write_byte(uint32_t function, uint32_t reg, uint8_t in_byte,
                                  uint8_t *out_byte)
{
    if (SdioWriteByte(function, reg, in_byte, 1U) != SDIO_SUCCESS) {
        return FAILURE;
    }

    /* HAL_SDIO_WriteDirect discards the read-after-write payload, so when the
       caller wants it back issue a plain CMD52 read instead. */
    if (out_byte != NULL) {
        return sdio_driver_read_byte(function, reg, out_byte);
    }
    return SDIO_SUCCESS;
}

static HAL_StatusTypeDef SdioByteReadBlockMode(uint32_t function, uint32_t addr,
                                               uint8_t *buf, uint32_t len,
                                               uint32_t timeout_ms);
static uint32_t SdioCmd53RetryableError(uint32_t err);

/**
  * @brief Put both ends of a failed CMD53 back to a known state.
  *
  * A data phase that ends in DCRCFAIL/DTIMEOUT leaves the slave's function-1
  * data path mid-transfer: it keeps waiting for the bytes it never got, and
  * answers the next CMD53 with COM_CRC_ERROR in R5 rather than the transfer.
  * That is the cascade in the log - one DATA_CRC_FAIL followed by an unbroken
  * run of err=0x1000 - and retrying the command alone can never clear it.
  *
  * CCCR 0x06 (I/O Abort, written by HAL_SDIO_AbortIOFunction) is the defined way
  * out: it aborts the function's transfer and frees the bus. The host side needs
  * the same treatment, hence the FIFO reset and the flag clear.
  */
/**
  * @brief CCCR I/O Abort carrying CMDSTOP: stops the slave and the DPSM at once.
  *
  * A data phase that ends without DATAEND leaves DPSMACT set - the DPSM is still
  * waiting for the bytes that never came - and while it is set, DCTRL and DLEN
  * are read-only. Every transfer after it therefore inherits the stale
  * programming and times out in turn, which is the "DCOUNT=508" (512 minus the
  * four bytes that did arrive) in the failure logs: a 4-byte register read that
  * was never able to load its own length. Rewriting DCTRL, pulsing FIFORST and
  * clearing the flags do not touch it, and neither does a plain CMD52.
  *
  * CMDSTOP is what does: it tells the CPSM this command terminates the data
  * transfer, so the DPSM is torn down with it. The frame it rides on is the
  * CCCR 0x06 write that HAL_SDIO_AbortIOFunction() would have sent anyway, so
  * the slave gets its I/O abort in the same breath. The response is not checked
  * - a slave wedged mid-transfer may not send one, and it acts on the write
  * either way.
  */
/* The abort frame itself can be lost - the slave is mid-transfer and the log
   shows it answering with CTIMEOUT (STA bit 2) while DPSMACT stays set. One
   shot is then not enough, and a DPSM left active makes DCTRL and DLEN
   read-only for every transfer that follows, so the link never comes back. */
#define SDIO_ABORT_ATTEMPTS 3U

static void SdioAbortDataPath(uint32_t function)
{
    SDMMC_CmdInitTypeDef cmd = {0};
    uint32_t tickstart;
    uint32_t attempt;

    for (attempt = 0U; attempt < SDIO_ABORT_ATTEMPTS; attempt++) {
        __SDMMC_CMDSTOP_ENABLE(hsdio1.Instance);
        __SDMMC_CMDTRANS_DISABLE(hsdio1.Instance);

        /* R/W=1, function 0, RAW=0, address 0x06 (I/O Abort), data = function no. */
        cmd.Argument         = 0x80000C00U | (function & 0x7U);
        cmd.CmdIndex         = SDMMC_CMD_SDMMC_RW_DIRECT;
        cmd.Response         = SDMMC_RESPONSE_SHORT;
        cmd.WaitForInterrupt = SDMMC_WAIT_NO;
        cmd.CPSM             = SDMMC_CPSM_ENABLE;
        (void)SDMMC_SendCommand(hsdio1.Instance, &cmd);

        tickstart = HAL_GetTick();
        while (((hsdio1.Instance->STA & (SDMMC_FLAG_CMDREND | SDMMC_FLAG_CTIMEOUT |
                                        SDMMC_FLAG_CCRCFAIL)) == 0U) &&
               ((HAL_GetTick() - tickstart) < 20U)) {
        }

        __SDMMC_CMDSTOP_DISABLE(hsdio1.Instance);

        tickstart = HAL_GetTick();
        while (((hsdio1.Instance->STA & SDMMC_FLAG_DPSMACT) != 0U) &&
               ((HAL_GetTick() - tickstart) < 20U)) {
        }

        /* Retrying with the response flags still latched would make the wait
           above fall straight through on the next pass */
        __SDMMC_CLEAR_FLAG(hsdio1.Instance, SDMMC_STATIC_CMD_FLAGS);

        if ((hsdio1.Instance->STA & SDMMC_FLAG_DPSMACT) == 0U) {
            return;
        }
    }

    SDIO_LOGE(TAG, "DPSM still active after %u aborts, STA=0x%08lx DCOUNT=%lu",
              (unsigned int)SDIO_ABORT_ATTEMPTS,
              (unsigned long)hsdio1.Instance->STA,
              (unsigned long)hsdio1.Instance->DCOUNT);
}

static void SdioRecoverAfterCmd53(uint32_t function)
{
    /* Data path first: nothing below can be reprogrammed until DPSMACT drops */
    SdioAbortDataPath(function);

    /* Now DCTRL is writable again. DTEN and the block size go with the rewrite;
       SDIOEN is the only bit worth keeping. */
    hsdio1.Instance->DCTRL = ((hsdio1.Instance->DCTRL & SDMMC_DCTRL_SDIOEN) != 0U)
                             ? SDMMC_DCTRL_SDIOEN : 0U;

    /* FIFORST is not self-clearing: left asserted it holds the receive FIFO in
       reset, so the next transfer's DCOUNT never moves and every following
       CMD53 times out. One failed transfer would otherwise take the link down
       for good. */
    MODIFY_REG(hsdio1.Instance->DCTRL, SDMMC_DCTRL_FIFORST, SDMMC_DCTRL_FIFORST);
    CLEAR_BIT(hsdio1.Instance->DCTRL, SDMMC_DCTRL_FIFORST);
    __SDMMC_CMDTRANS_DISABLE(hsdio1.Instance);
    __HAL_SDIO_CLEAR_FLAG(&hsdio1, SDMMC_STATIC_FLAGS);
    __HAL_SDIO_CLEAR_FLAG(&hsdio1, SDMMC_STATIC_DATA_FLAGS);

    hsdio1.State = HAL_SDIO_STATE_READY;
    hsdio1.Context = SDIO_CONTEXT_NONE;
}

/* A register access is at most one word; anything longer is packet payload and
   has to stay on CMD53. */
#define SDIO_CMD52_REG_XFER_MAX 4U
/* ...and only a register may take it at all. In the packet window at
   ESP_SLAVE_CMD53_END_ADDR the address *is* the transfer length, so splitting a
   read into four CMD52s does not read four bytes of one packet - it asks the
   slave for four different lengths and destroys the queue. Every function-1
   register this driver touches (0x44, 0x50, 0x58, 0x60, 0xd4) is below 0x1000;
   the window is at 0x1f4xx-0x1f800. */
#define SDIO_CMD52_FALLBACK_MAX_ADDR 0x1000U
/* Attempts per byte. Kept short - each miss costs a CMD52 timeout and a log
   line, and this path only runs on a register. */
#define SDIO_CMD52_BYTE_RETRY     4U

/**
  * @brief Read a slave register a byte at a time with CMD52.
  *
  * Function 1 registers 0x44 (TOKEN_RDATA) and 0x48 answer a CMD53 with a data
  * phase the host's CRC rejects - measured 0/20 good, while 0x40, 0x4c-0x60 are
  * 20/20 and CMD52 returns the same bytes on every attempt. Without the token
  * count the host never believes the slave has a free buffer, so every AT
  * command typed at the console ends in "buffer is not enough: 0, 1 required".
  *
  * CMD52 carries its byte in the response on the CMD line and has no data
  * phase at all, which is why it is unaffected.
  */
static sdio_err_t SdioReadBytesCmd52(uint32_t function, uint32_t addr,
                                     uint8_t *buf, uint32_t len)
{
    uint32_t i;
    uint32_t retry;

    for (i = 0U; i < len; i++) {
        /* The link drops the occasional frame, and this is the last line of
           defence: a single missed CMD52 here fails the whole word read, which
           for the length register at 0x60 aborts the receive drain. */
        for (retry = 0U; retry < SDIO_CMD52_BYTE_RETRY; retry++) {
            if (SdioReadByte(function, addr + i, &buf[i], 0U) == SDIO_SUCCESS) {
                break;
            }
            HAL_Delay(1U);
        }

        if (retry >= SDIO_CMD52_BYTE_RETRY) {
            SDIO_LOGE(TAG, "CMD52 read error, func %lu reg 0x%lx",
                      (unsigned long)function, (unsigned long)(addr + i));
            return FAILURE;
        }
    }
    return SDIO_SUCCESS;
}

static sdio_err_t SdioWriteBytesCmd52(uint32_t function, uint32_t addr,
                                      const uint8_t *buf, uint32_t len)
{
    uint32_t i;
    uint32_t retry;

    for (i = 0U; i < len; i++) {
        for (retry = 0U; retry < SDIO_CMD52_BYTE_RETRY; retry++) {
            if (SdioWriteByte(function, addr + i, buf[i], 0U) == SDIO_SUCCESS) {
                break;
            }
            HAL_Delay(1U);
        }

        if (retry >= SDIO_CMD52_BYTE_RETRY) {
            SDIO_LOGE(TAG, "CMD52 write error, func %lu reg 0x%lx",
                      (unsigned long)function, (unsigned long)(addr + i));
            return FAILURE;
        }
    }
    return SDIO_SUCCESS;
}

/**
  * @brief Wall-clock budget for one CMD53, from what the transfer must take.
  *
  * A flat second is both too long and too short: at SDIO_PORT_CLOCK_HZ a
  * 512-byte block needs ~164 ms, so a stall costs six times what it should to
  * notice, while a full 4 kB packet legitimately needs more than a second and
  * would be failed while it was still running. Three times the theoretical
  * time, plus a fixed floor for the command and turnaround.
  */
static uint32_t SdioXferTimeoutMs(uint32_t len)
{
    /* 8 clocks per byte on one data line; a 4-bit bus only finishes sooner */
    uint32_t ms = ((len * 8U) / (SDIO_PORT_CLOCK_HZ / 1000U));

    /* Six times, not three. The slave does not stream a multi-block read at the
       bus rate - it pauses between blocks - and a 1024-byte read that needs
       328 ms of clock was being failed at 1033 ms with the data still coming.
       The cost of the wider budget is only how long a genuinely dead transfer
       takes to notice, and that one is retried at most three times. */
    return 100U + (ms * 6U);
}

static sdio_err_t SdioTransfer(uint32_t function, uint32_t addr, void *buffer,
                               uint32_t len, uint32_t block_mode, uint8_t is_write)
{
    HAL_SDIO_ExtendedCmd_TypeDef cmd53;
    HAL_StatusTypeDef status;
    uint32_t retry;
    uint32_t err = HAL_SDIO_ERROR_NONE;

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

    const uint8_t cmd52_reg_xfer = ((block_mode == HAL_SDIO_MODE_BYTE) &&
                                    (len <= SDIO_CMD52_REG_XFER_MAX) &&
                                    (addr < SDIO_CMD52_FALLBACK_MAX_ADDR)) ? 1U : 0U;

    const uint32_t timeout_ms = SdioXferTimeoutMs(len);

    if (cmd52_reg_xfer != 0U) {
        if (is_write != 0U) {
            return SdioWriteBytesCmd52(function, addr, (const uint8_t *)buffer, len);
        }
        return SdioReadBytesCmd52(function, addr, (uint8_t *)buffer, len);
    }

    /* The whole transfer runs with SDMMC1 masked at the NVIC.
     *
     * HAL_SDIO_EnableIOFunctionInterrupt() leaves SDMMC_IT_SDIOIT enabled for
     * good, and STA.SDIOIT simply tracks D1 - so the slave raising its
     * interrupt part way through a data phase, which is exactly what it does
     * when the next packet lands, runs HAL_SDIO_IRQHandler(). That handler
     * reads STA and acts on DATAEND *without checking whether the DATAEND
     * interrupt was ever enabled*: it clears the flag, forces State to READY
     * and disables CMDTRANS. The polled loop inside HAL_SDIO_ReadExtended() /
     * WriteExtended() is then waiting for a flag that has already been taken
     * from it, and gives up with HAL_SDIO_ERROR_TIMEOUT (0x80000000) on a
     * transfer whose data actually moved - no CRC error, no DTIMEOUT. From
     * there the recovery aborts a function that was never stuck and the link
     * stops carrying anything at all.
     *
     * Masking the IRQ loses no interrupt: SDIOIT is a level, not an edge, so
     * it is still asserted when the mask is lifted. */
    HAL_NVIC_DisableIRQ(SDMMC1_IRQn);

    for (retry = 0U; retry < SDIO_CMD53_RETRY_COUNT; retry++) {
        if ((is_write == 0U) && (block_mode == HAL_SDIO_MODE_BYTE)) {
            status = SdioByteReadBlockMode(function, addr, (uint8_t *)buffer, len, timeout_ms);
        } else if (is_write != 0U) {
            status = HAL_SDIO_WriteExtended(&hsdio1, &cmd53, (uint8_t *)buffer, len, timeout_ms);
        } else {
            status = HAL_SDIO_ReadExtended(&hsdio1, &cmd53, (uint8_t *)buffer, len, timeout_ms);
        }

        if (status == HAL_OK) {
            HAL_NVIC_EnableIRQ(SDMMC1_IRQn);
            return SDIO_SUCCESS;
        }

        /* Latched before the recovery below, whose CMD52 resets ErrorCode.
           DCOUNT goes with it: on a read that ends in HAL_SDIO_ERROR_TIMEOUT it
           says how much of the transfer never arrived, which is the difference
           between a slave that stopped early and one that never started. */
        err = HAL_SDIO_GetError(&hsdio1);
        SDIO_LOGD(TAG, "CMD53 attempt %lu failed, STA=0x%08lx DCOUNT=%lu",
                  (unsigned long)retry, (unsigned long)hsdio1.Instance->STA,
                  (unsigned long)hsdio1.Instance->DCOUNT);
        SdioRecoverAfterCmd53(function);

        if ((SdioCmd53RetryableError(err) == 0U) ||
            ((retry + 1U) >= SDIO_CMD53_RETRY_COUNT) ||
            (((err & HAL_SDIO_ERROR_TIMEOUT) != 0U) &&
             ((retry + 1U) >= SDIO_CMD53_TIMEOUT_RETRY))) {
            break;
        }
        HAL_Delay(1U);
    }

    HAL_NVIC_EnableIRQ(SDMMC1_IRQn);

    if (sdio_quiet_errors != 0U) {
        /* The caller knows the slave is going away - an AT+RESTORE reboot -
           so a failed transfer here is the expected outcome, not a fault. */
        return FAILURE;
    }

    {
        SDIO_LOGE(TAG, "CMD53 %s error, addr 0x%lx count %lu, err 0x%08lx%s%s%s%s%s%s%s%s",
                  (is_write != 0U) ? "write" : "read", (unsigned long)addr,
                  (unsigned long)len, (unsigned long)err,
                  ((err & SDMMC_ERROR_CMD_CRC_FAIL) != 0U) ? " CMD_CRC_FAIL" : "",
                  ((err & SDMMC_ERROR_CMD_RSP_TIMEOUT) != 0U) ? " CMD_RSP_TIMEOUT" : "",
                  ((err & SDMMC_ERROR_COM_CRC_FAILED) != 0U) ? " R5_COM_CRC" : "",
                  ((err & HAL_SDIO_ERROR_DATA_CRC_FAIL) != 0U) ? " DATA_CRC_FAIL" : "",
                  ((err & HAL_SDIO_ERROR_DATA_TIMEOUT) != 0U) ? " DATA_TIMEOUT" : "",
                  ((err & HAL_SDIO_ERROR_TX_UNDERRUN) != 0U) ? " TX_UNDERRUN" : "",
                  ((err & HAL_SDIO_ERROR_RX_OVERRUN) != 0U) ? " RX_OVERRUN" : "",
                  ((err & HAL_SDIO_ERROR_TIMEOUT) != 0U) ? " TIMEOUT" : "");
        return FAILURE;
    }
}

static uint32_t SdioCmd53RetryableError(uint32_t err)
{
    return ((err & (SDMMC_ERROR_CMD_CRC_FAIL |
                   SDMMC_ERROR_COM_CRC_FAILED |
                   SDMMC_ERROR_CMD_RSP_TIMEOUT |
                   HAL_SDIO_ERROR_DATA_CRC_FAIL |
                   HAL_SDIO_ERROR_DATA_TIMEOUT |
                   HAL_SDIO_ERROR_TIMEOUT)) != 0U) ? 1U : 0U;
}

static uint32_t SdioCmd53Arg(uint32_t function, uint32_t addr, uint32_t len,
                             uint32_t block_mode, uint8_t is_write)
{
    uint32_t arg = (is_write != 0U) ? (1UL << 31U) : 0U;

    arg |= (function & 0x7U) << 28U;
    arg |= (block_mode == HAL_SDIO_MODE_BLOCK) ? (1UL << 27U) : 0U;
    arg |= 1UL << 26U;
    arg |= (addr & 0x1FFFFU) << 9U;
    arg |= len & 0x1FFU;

    return arg;
}

static uint32_t SdioResp5Error(uint32_t r5)
{
    if ((r5 & SDMMC_SDIO_R5_OUT_OF_RANGE) != 0U) {
        return SDMMC_ERROR_ADDR_OUT_OF_RANGE;
    }
    if ((r5 & SDMMC_SDIO_R5_INVALID_FUNCTION_NUMBER) != 0U) {
        return SDMMC_ERROR_INVALID_PARAMETER;
    }
    if ((r5 & SDMMC_SDIO_R5_ILLEGAL_CMD) != 0U) {
        return SDMMC_ERROR_ILLEGAL_CMD;
    }
    if ((r5 & SDMMC_SDIO_R5_COM_CRC_FAILED) != 0U) {
        return SDMMC_ERROR_COM_CRC_FAILED;
    }
    if ((r5 & (SDMMC_SDIO_R5_ERROR | SDMMC_SDIO_R5_GENERAL_UNKNOWN_ERROR)) != 0U) {
        return SDMMC_ERROR_GENERAL_UNKNOWN_ERR;
    }
    return SDMMC_ERROR_NONE;
}

static uint32_t SdioWaitResp5(uint32_t *out_resp)
{
    uint32_t sta;
    uint32_t count = SDMMC_CMDTIMEOUT * (SystemCoreClock / 8U / 1000U);

    do {
        if (count-- == 0U) {
            return SDMMC_ERROR_TIMEOUT;
        }
        sta = hsdio1.Instance->STA;
    } while (((sta & (SDMMC_FLAG_CCRCFAIL | SDMMC_FLAG_CMDREND | SDMMC_FLAG_CTIMEOUT)) == 0U) ||
             ((sta & SDMMC_FLAG_CMDACT) != 0U));

    if ((sta & SDMMC_FLAG_CTIMEOUT) != 0U) {
        __HAL_SDIO_CLEAR_FLAG(&hsdio1, SDMMC_FLAG_CTIMEOUT);
        return SDMMC_ERROR_CMD_RSP_TIMEOUT;
    }

    if (out_resp != NULL) {
        *out_resp = SDMMC_GetResponse(hsdio1.Instance, SDMMC_RESP1);
    }

    if (SDMMC_GetCommandResponse(hsdio1.Instance) != SDMMC_CMD_SDMMC_RW_EXTENDED) {
        if ((sta & SDMMC_FLAG_CCRCFAIL) != 0U) {
            __HAL_SDIO_CLEAR_FLAG(&hsdio1, SDMMC_FLAG_CCRCFAIL);
        }
        return SDMMC_ERROR_CMD_CRC_FAIL;
    }

    if ((sta & SDMMC_FLAG_CCRCFAIL) != 0U) {
        uint32_t r5 = (out_resp != NULL) ? *out_resp : SDMMC_GetResponse(hsdio1.Instance, SDMMC_RESP1);
        uint32_t r5_err = SdioResp5Error(r5);

        __HAL_SDIO_CLEAR_FLAG(&hsdio1, SDMMC_FLAG_CCRCFAIL);
        if (r5_err != SDMMC_ERROR_NONE) {
            return r5_err;
        }
    }

    __HAL_SDIO_CLEAR_FLAG(&hsdio1, SDMMC_STATIC_CMD_FLAGS);
    return SdioResp5Error((out_resp != NULL) ? *out_resp : SDMMC_GetResponse(hsdio1.Instance, SDMMC_RESP1));
}

static HAL_StatusTypeDef SdioByteReadBlockMode(uint32_t function, uint32_t addr,
                                               uint8_t *buf, uint32_t len,
                                               uint32_t timeout_ms)
{
    SDMMC_DataInitTypeDef data = {0};
    SDMMC_CmdInitTypeDef cmd = {0};
    uint32_t err;
    uint32_t resp = 0U;
    uint32_t remain = len;
    uint32_t start = HAL_GetTick();

    hsdio1.ErrorCode = HAL_SDIO_ERROR_NONE;
    /* Full write, so FIFORST left over from an aborted transfer goes with it */
    hsdio1.Instance->DCTRL = (hsdio1.Instance->DCTRL & SDMMC_DCTRL_SDIOEN) ? SDMMC_DCTRL_SDIOEN : 0U;

    data.DataTimeOut = SDMMC_DATATIMEOUT;
    data.DataLength = len;
    data.DataBlockSize = SDMMC_DATABLOCK_SIZE_1B;
    data.TransferDir = SDMMC_TRANSFER_DIR_TO_SDMMC;
    data.TransferMode = SDMMC_TRANSFER_MODE_SDIO;
    data.DPSM = SDMMC_DPSM_DISABLE;
    (void)SDMMC_ConfigData(hsdio1.Instance, &data);
    __SDMMC_CMDTRANS_ENABLE(hsdio1.Instance);

    cmd.Argument = SdioCmd53Arg(function, addr, len, HAL_SDIO_MODE_BYTE, 0U);
    cmd.CmdIndex = SDMMC_CMD_SDMMC_RW_EXTENDED;
    cmd.Response = SDMMC_RESPONSE_SHORT;
    cmd.WaitForInterrupt = SDMMC_WAIT_NO;
    cmd.CPSM = SDMMC_CPSM_ENABLE;
    (void)SDMMC_SendCommand(hsdio1.Instance, &cmd);

    err = SdioWaitResp5(&resp);
    if (err != SDMMC_ERROR_NONE) {
        hsdio1.ErrorCode |= err;
        MODIFY_REG(hsdio1.Instance->DCTRL, SDMMC_DCTRL_FIFORST, SDMMC_DCTRL_FIFORST);
        __HAL_SDIO_CLEAR_FLAG(&hsdio1, SDMMC_STATIC_FLAGS);
        __HAL_SDIO_CLEAR_FLAG(&hsdio1, SDMMC_STATIC_DATA_FLAGS);
        __SDMMC_CMDTRANS_DISABLE(hsdio1.Instance);
        return HAL_ERROR;
    }

    while ((hsdio1.Instance->STA & (SDMMC_FLAG_RXOVERR | SDMMC_FLAG_DCRCFAIL |
                                    SDMMC_FLAG_DTIMEOUT | SDMMC_FLAG_DATAEND)) == 0U) {
        if (((hsdio1.Instance->STA & SDMMC_FLAG_RXFIFOE) == 0U) && (remain > 0U)) {
            uint32_t v = SDMMC_ReadFIFO(hsdio1.Instance);
            uint32_t i;

            for (i = 0U; (i < 4U) && (remain > 0U); i++) {
                *buf++ = (uint8_t)(v >> (i * 8U));
                remain--;
            }
        }
        if ((HAL_GetTick() - start) >= timeout_ms) {
            SDIO_LOGD(TAG, "byte read timeout addr 0x%lx len %lu STA=0x%08lx DCTRL=0x%08lx DCOUNT=%lu resp=0x%08lx",
                      (unsigned long)addr, (unsigned long)len,
                      (unsigned long)hsdio1.Instance->STA,
                      (unsigned long)hsdio1.Instance->DCTRL,
                      (unsigned long)hsdio1.Instance->DCOUNT,
                      (unsigned long)resp);
            hsdio1.ErrorCode |= HAL_SDIO_ERROR_TIMEOUT;
            __HAL_SDIO_CLEAR_FLAG(&hsdio1, SDMMC_STATIC_FLAGS);
            __SDMMC_CMDTRANS_DISABLE(hsdio1.Instance);
            return HAL_TIMEOUT;
        }
    }

    __SDMMC_CMDTRANS_DISABLE(hsdio1.Instance);

    if ((hsdio1.Instance->STA & SDMMC_FLAG_DTIMEOUT) != 0U) {
        hsdio1.ErrorCode |= HAL_SDIO_ERROR_DATA_TIMEOUT;
    } else if ((hsdio1.Instance->STA & SDMMC_FLAG_DCRCFAIL) != 0U) {
        hsdio1.ErrorCode |= HAL_SDIO_ERROR_DATA_CRC_FAIL;
    } else if ((hsdio1.Instance->STA & SDMMC_FLAG_RXOVERR) != 0U) {
        hsdio1.ErrorCode |= HAL_SDIO_ERROR_RX_OVERRUN;
    }

    __HAL_SDIO_CLEAR_FLAG(&hsdio1, SDMMC_STATIC_FLAGS);
    return (hsdio1.ErrorCode == HAL_SDIO_ERROR_NONE) ? HAL_OK : HAL_ERROR;
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
