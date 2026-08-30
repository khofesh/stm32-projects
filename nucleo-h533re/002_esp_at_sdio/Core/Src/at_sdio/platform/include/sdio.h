/**
  ******************************************************************************
  * @file    sdio.h
  * @brief   STM32H5 SDMMC1-in-SDIO-mode port for the ESP-AT SDIO host.
  *
  * The STM32H5 HAL ships a full SDIO driver (stm32h5xx_hal_sdio.c) providing
  * CMD52/CMD53, so unlike the STM32F1 reference port there is no hand written
  * SDIO register driver here - this header only exposes the handle and the
  * board level configuration.
  ******************************************************************************
  */
#ifndef __SDIO_H
#define __SDIO_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "stm32h5xx_hal.h"

/* Bring-up configuration ----------------------------------------------------*/
/* Enumeration always runs at 1 wire; SDIO_PORT_BUS_WIDTH only selects what the
   link is widened to afterwards. It runs at SDIO_PORT_CLOCK_HZ rather than the
   400 kHz the HAL would pick - see SdioIdentifyCardSlow() in the .c.
   HAL_SDIO_Init() writes the slave CCCR bus width from Init.BusWide compared
   against HAL_SDIO_4_WIRES_MODE (1), but Init.BusWide holds an SDMMC_BUS_WIDE_*
   register value, so the comparison never matches and the slave would be left
   in 1-bit mode - sdio_driver_init() writes CCCR 0x07 itself instead.

   Held at 1 wire, as the esp-at example advises. No hardware change is needed
   to switch: D1-D3 keep their pull-ups and stay wired, the SDMMC just never
   drives them.

   4 wires is what the CMD53 data phase fails on. CMD52 travels on CMD alone and
   is 100% reliable on this link, while every CMD53 - the first one after
   enumeration included - comes back DATA_CRC_FAIL, then COM_CRC_FAILED once the
   slave's data path is out of step. That is a data-line fault, and D2 is the
   suspect: it can only be PC10 on this package, and the NUCLEO-H533RE assigns
   PC10 to USB_FS_PWR_EN, so there is board circuitry sitting on the net (see
   ERROR.md). 1 wire uses CK, CMD, D0 and D1 only, so D2 never carries data.

   Before going back to 4 wires, prove PC10 swings rail to rail at the morpho
   pin with the SDMMC muxed off. */
#define SDIO_PORT_BUS_WIDTH   HAL_SDIO_1_WIRE_MODE
#define SDIO_PORT_CLOCK_HZ    25000U

/* Drop the SDMMC1 kernel clock from PLL1Q (250 MHz) to PLL2R (50 MHz).

   CLKDIV is 10 bits, so off 250 MHz the bus bottoms out at 122 kHz. This link
   does not carry a frame reliably anywhere near there: a bit-banged CMD5 is
   answered at 25 kHz and ignored at 400 kHz. 50 MHz makes 25 kHz reachable
   (ClockDiv 1000), and SdioIdentifyCardSlow() is what stops the HAL putting
   400 kHz back for the identification sequence.

   This is a crutch, not a destination: needing 25 kHz means the interconnect is
   the real problem. Shorten the jumpers and give each signal a ground return,
   then walk SDIO_PORT_CLOCK_HZ back up and set this to 0 once 400 kHz holds. */
#define SDIO_PORT_SLOW_KERNEL_CLK  1

/* Pad slew rate for CK/CMD/D0-D3. CubeMX and the ST SDMMC examples use
   VERY_HIGH, but on this wiring it is the worst setting: the measured CMD5
   matrix (see docs/ERROR.md) is 0/8 answers at VERY_HIGH, 3 lost CK edges at
   HIGH and MEDIUM, and 2 at LOW. Slower pads glitch the slave's clock less, and
   LOW is electrically ample at the 400 kHz bring-up clock. Raise it only after
   the interconnect is fixed and the clock has been increased.
   SdioProbeSlave() sweeps all four at runtime, so re-measure before changing. */
#define SDIO_PORT_GPIO_SPEED  GPIO_SPEED_FREQ_LOW

/* Bring-up diagnostics, both run before HAL_SDIO_Init() muxes the AF on.

   SDIO_PORT_PIN_DIAG: pull-up census plus a pin-to-pin short matrix on the six
   SDMMC nets. Costs ~100 ms and needs no instruments; set to 0 once the link is
   up. SDIO_PORT_CK_TOGGLE_MS: sweep the six nets one at a time, each a 2 Hz GPIO
   square wave for that many milliseconds, so continuity to the matching
   ESP32-C6 pad can be checked with a meter or by reading GPIO_IN on the slave.
   0 disables it - leave it disabled unless something is actually watching the
   far end, since it only delays enumeration. */
#define SDIO_PORT_PIN_DIAG      1
#define SDIO_PORT_CK_TOGGLE_MS  0U

/* Drive CMD open-drain instead of push-pull.

   Kept because it is cheap to reach for, but measured to be unnecessary: it was
   tried against the theory that the CPSM was still driving CMD when the slave
   answered at NCR=2, and it changed nothing. Enumeration is 8/8 with plain
   push-pull once the clock and the per-command retries are right. Turn it on
   only if CMD contention is ever actually demonstrated. */
#define SDIO_PORT_CMD_OPEN_DRAIN  0

/* Slave reset line: PB15 (CN10-26, free on this board) to the ESP32-C6 EN pin.
   Without it the slave keeps whatever state the previous host session left it
   in, and an I/O-only card may ignore the CMD0 that opens enumeration, so a
   host restart alone cannot recover the link.

   Open-drain: the devkit already pulls EN up and the USB bridge drives it low
   for its own auto-reset, so the host must only ever pull down, never push up.
   A host reset leaves the pad high-Z and the module boots on its own.

   EN carries ~10k/1uF on the devkit, so it needs a long low to discharge and
   ~10 ms to rise again; BOOT_MS then covers ROM plus esp-at reaching
   sdio_slave_start, measured at ~1.05 s. 0 disables the whole thing. */
#define SDIO_PORT_ESP_RST_PORT     GPIOB
#define SDIO_PORT_ESP_RST_PIN      GPIO_PIN_15
#define SDIO_PORT_ESP_RST_LOW_MS   20U
#define SDIO_PORT_ESP_RST_BOOT_MS  1500U

/* Function 1 is the ESP slave data function */
#define SDIO_PORT_FUNCTION    HAL_SDIO_FUNCTION_1
#define SDIO_PORT_BLOCK_SIZE  HAL_SDIO_DATA_BLOCK_SIZE_512BYTE

extern SDIO_HandleTypeDef hsdio1;

/**
  * @brief Dispatch SDMMC1 interrupts to the SDIO HAL.
  *        Call from SDMMC1_IRQHandler() in stm32h5xx_it.c.
  */
void SdioPortIrqHandler(void);

#ifdef __cplusplus
}
#endif

#endif /* __SDIO_H */
