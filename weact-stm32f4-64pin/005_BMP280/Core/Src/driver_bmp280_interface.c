/**
 * Copyright (c) 2015 - present LibDriver All rights reserved
 *
 * The MIT License (MIT)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * @file      driver_bmp280_interface.h
 * @brief     driver bmp280 interface header file
 * @version   1.0.0
 * @author    Shifeng Li
 * @date      2024-01-15
 *
 * <h3>history</h3>
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2024/01/15  <td>1.0      <td>Shifeng Li  <td>first upload
 * </table>
 */

#ifndef DRIVER_BMP280_INTERFACE_H
#define DRIVER_BMP280_INTERFACE_H

#include "driver_bmp280.h"
#include "driver_bmp280_interface.h"
#include "main.h"
#include <stdarg.h>
#include <stdio.h>

extern SPI_HandleTypeDef hspi1;
extern UART_HandleTypeDef huart1;

// CS pin for BMP280
#define BMP280_CS_GPIO_Port GPIOA
#define BMP280_CS_Pin GPIO_PIN_4

// CS pin control macros
#define BMP280_CS_LOW()        HAL_GPIO_WritePin(BMP280_CS_GPIO_Port, BMP280_CS_Pin, GPIO_PIN_RESET)
#define BMP280_CS_HIGH()       HAL_GPIO_WritePin(BMP280_CS_GPIO_Port, BMP280_CS_Pin, GPIO_PIN_SET)


#ifdef __cplusplus
extern "C"{
#endif

/**
 * @defgroup bmp280_interface_driver bmp280 interface driver function
 * @brief    bmp280 interface driver modules
 * @ingroup  bmp280_driver
 * @{
 */

/**
 * @brief  interface iic bus init
 * @return status code
 *         - 0 success
 *         - 1 iic init failed
 * @note   none
 */
uint8_t bmp280_interface_iic_init(void)
{
	return 1; // not implemented
}

/**
 * @brief  interface iic bus deinit
 * @return status code
 *         - 0 success
 *         - 1 iic deinit failed
 * @note   none
 */
uint8_t bmp280_interface_iic_deinit(void);

/**
 * @brief      interface iic bus read
 * @param[in]  addr iic device write address
 * @param[in]  reg iic register address
 * @param[out] *buf pointer to a data buffer
 * @param[in]  len length of the data buffer
 * @return     status code
 *             - 0 success
 *             - 1 read failed
 * @note       none
 */
uint8_t bmp280_interface_iic_read(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len);

/**
 * @brief     interface iic bus write
 * @param[in] addr iic device write address
 * @param[in] reg iic register address
 * @param[in] *buf pointer to a data buffer
 * @param[in] len length of the data buffer
 * @return    status code
 *            - 0 success
 *            - 1 write failed
 * @note      none
 */
uint8_t bmp280_interface_iic_write(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len);

/**
 * @brief  interface spi bus init
 * @return status code
 *         - 0 success
 *         - 1 spi init failed
 * @note   none
 */
uint8_t bmp280_interface_spi_init(void)
{
	// CS pin is already enabled in main.c

	BMP280_CS_HIGH();

	return 0;
}

/**
 * @brief  interface spi bus deinit
 * @return status code
 *         - 0 success
 *         - 1 spi deinit failed
 * @note   none
 */
uint8_t bmp280_interface_spi_deinit(void)
{
	HAL_GPIO_DeInit(BMP280_CS_GPIO_Port, BMP280_CS_Pin);

	return 0;
}

/**
 * @brief      interface spi bus read
 * @param[in]  reg register address
 * @param[out] *buf pointer to a data buffer
 * @param[in]  len length of data buffer
 * @return     status code
 *             - 0 success
 *             - 1 read failed
 * @note       none
 */
uint8_t bmp280_interface_spi_read(uint8_t reg, uint8_t *buf, uint16_t len)
{
	HAL_StatusTypeDef status;
	// set MSB for read operation
	uint8_t tx_reg = reg | 0x80;

	BMP280_CS_LOW();

	status = HAL_SPI_Transmit(&hspi1, &tx_reg, 1, HAL_MAX_DELAY);
	if (status != HAL_OK)
	{
		BMP280_CS_HIGH();
		return 1;
	}

	status = HAL_SPI_Receive(&hspi1, buf, len, HAL_MAX_DELAY);
	if (status != HAL_OK)
	{
		BMP280_CS_HIGH();
		return 1;
	}

	BMP280_CS_HIGH();

	return 0;
}

/**
 * @brief     interface spi bus write
 * @param[in] reg register address
 * @param[in] *buf pointer to a data buffer
 * @param[in] len length of data buffer
 * @return    status code
 *            - 0 success
 *            - 1 write failed
 * @note      none
 */
uint8_t bmp280_interface_spi_write(uint8_t reg, uint8_t *buf, uint16_t len)
{
	HAL_StatusTypeDef status;
	// clear MSB for write operation
	uint8_t tx_reg = reg & 0x7F;

	BMP280_CS_LOW();

	status = HAL_SPI_Transmit(&hspi1, &tx_reg, 1, HAL_MAX_DELAY);
	if (status != HAL_OK)
	{
		BMP280_CS_HIGH();
		return 1;
	}

	status = HAL_SPI_Receive(&hspi1, buf, len, HAL_MAX_DELAY);
	if (status != HAL_OK)
	{
		BMP280_CS_HIGH();
		return 1;
	}

	BMP280_CS_HIGH();

	return 0;
}

/**
 * @brief     interface delay ms
 * @param[in] ms time
 * @note      none
 */
void bmp280_interface_delay_ms(uint32_t ms)
{
	HAL_Delay(ms);
}

/**
 * @brief     interface print format data
 * @param[in] fmt format data
 * @note      none
 */
void bmp280_interface_debug_print(const char *const fmt, ...)
{
	int strSize = 256;
	char str[strSize];
	uint16_t len;
	va_list args;

    memset((char *)str, 0, sizeof(char) * 256);
    va_start(args, fmt);
    vsnprintf((char *)str, 255, (char const *)fmt, args);
    va_end(args);

    len = strlen((char *)str);
    HAL_UART_Transmit(&huart1, (uint8_t *)str, len, 0xFFFF);
}

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif
