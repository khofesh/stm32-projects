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
/* Start at 1 wire / 400 kHz. Move to 4 wires and a higher clock only once the
   CCCR reads in the ESP-AT log come back non-zero.
   Note for 4 wires: HAL_SDIO_Init() writes the slave CCCR bus width from
   Init.BusWide compared against HAL_SDIO_4_WIRES_MODE (1), but Init.BusWide
   holds an SDMMC_BUS_WIDE_* register value, so the comparison never matches
   and the slave is left in 1-bit mode. Write CCCR 0x07 bit1 yourself with
   HAL_SDIO_WriteDirect() after init when switching to 4 wires. */
#define SDIO_PORT_BUS_WIDTH   HAL_SDIO_1_WIRE_MODE
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
