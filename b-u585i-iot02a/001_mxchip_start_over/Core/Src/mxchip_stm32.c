/*
 * mxchip_stm32.c
 *
 *  Created on: Oct 4, 2025
 *      Author: fahmad
 */

#include "mxchip_stm32.h"
#include "mx_wifi_io.h"

/* External UART handle for debug output */
extern UART_HandleTypeDef huart1;
/* External SPI handle for MXCHIP communication */
extern SPI_HandleTypeDef hspi2;


