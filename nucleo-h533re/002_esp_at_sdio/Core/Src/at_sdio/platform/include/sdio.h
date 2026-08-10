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
/* Enumeration always runs at 1 wire / 400 kHz; SDIO_PORT_BUS_WIDTH only selects
   what the link is widened to afterwards.
   HAL_SDIO_Init() writes the slave CCCR bus width from Init.BusWide compared
   against HAL_SDIO_4_WIRES_MODE (1), but Init.BusWide holds an SDMMC_BUS_WIDE_*
   register value, so the comparison never matches and the slave would be left
   in 1-bit mode - sdio_driver_init() writes CCCR 0x07 itself instead.

   Held at 1 wire for bring-up, as the esp-at example advises. No hardware
   change is needed to switch: D1-D3 keep their pull-ups and stay wired, the
   SDMMC just never drives them. 1 wire also keeps D2 off PC10, which the
   NUCLEO-H533RE assigns to USB_FS_PWR_EN - see ERROR.md. */
#define SDIO_PORT_BUS_WIDTH   HAL_SDIO_4_WIRES_MODE
#define SDIO_PORT_CLOCK_HZ    400000U

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
