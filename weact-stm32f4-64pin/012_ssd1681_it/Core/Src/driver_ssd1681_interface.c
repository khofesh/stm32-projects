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
 * @file      driver_ssd1681_interface_template.c
 * @brief     driver ssd1681 interface template source file
 * @version   1.0.0
 * @author    Shifeng Li
 * @date      2022-08-30
 *
 * <h3>history</h3>
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2022/08/30  <td>1.0      <td>Shifeng Li  <td>first upload
 * </table>
 */

#include "driver_ssd1681_interface.h"

#include <stdarg.h>
#include <stdio.h>

extern SPI_HandleTypeDef hspi1;
extern UART_HandleTypeDef huart1;

// GPIO pin for SSD1681
#define SSD1681_BUSY_GPIO_PORT GPIOA
#define SSD1681_BUSY_GPIO_PIN  GPIO_PIN_0

#define SSD1681_RESET_GPIO_PORT    GPIOA
#define SSD1681_RESET_GPIO_PIN     GPIO_PIN_1

#define SSD1681_DC_GPIO_PORT       GPIOA
#define SSD1681_DC_GPIO_PIN        GPIO_PIN_2

#define SSD1681_CS_GPIO_PORT       GPIOA
#define SSD1681_CS_GPIO_PIN        GPIO_PIN_4

// SPI operation states
typedef enum {
    SPI_STATE_READY = 0,
    SPI_STATE_BUSY_TX,
    SPI_STATE_BUSY_RX,
    SPI_STATE_ERROR
} spi_state_t;

// SPI operation completion flags
static volatile spi_state_t spi_state = SPI_STATE_READY;
static volatile uint8_t spi_tx_complete = 0;
static volatile uint8_t spi_rx_complete = 0;
static volatile uint8_t spi_error_flag = 0;

// timeout for SPI operations
#define SPI_TIMEOUT_MS 1000

/**
 * @brief  Wait for SPI operation to complete
 * @param  timeout_ms Timeout in milliseconds
 * @return status code
 *         - 0 success
 *         - 1 timeout or error
 */
static uint8_t ssd1681_wait_for_spi_complete(uint32_t timeout_ms)
{
	uint32_t start_time = HAL_GetTick();

	while ((HAL_GetTick() - start_time) < timeout_ms)
	{
		if (spi_error_flag)
		{
			spi_error_flag = 0;
			spi_state = SPI_STATE_READY;
			return 1;
		}

		if (spi_state == SPI_STATE_BUSY_TX && spi_tx_complete)
		{
			spi_tx_complete = 0;
			spi_state = SPI_STATE_READY;
			return 0;
		}
		else if (spi_state == SPI_STATE_BUSY_RX && spi_rx_complete)
		{
			spi_rx_complete = 0;
			spi_state = SPI_STATE_READY;
			return 0;
		}

		HAL_Delay(1);
	}

	spi_state = SPI_STATE_READY;
	return 1;
}

/**
 * @brief  interface spi bus init
 * @return status code
 *         - 0 success
 *         - 1 spi init failed
 * @note   none
 */
uint8_t ssd1681_interface_spi_init(void)
{
    return 0;
}

/**
 * @brief  interface spi bus deinit
 * @return status code
 *         - 0 success
 *         - 1 spi deinit failed
 * @note   none
 */
uint8_t ssd1681_interface_spi_deinit(void)
{
    return 0;
}

/**
 * @brief     interface spi bus write
 * @param[in] *buf pointer to a data buffer
 * @param[in] len length of data buffer
 * @return    status code
 *            - 0 success
 *            - 1 write failed
 * @note      none
 */
uint8_t ssd1681_interface_spi_write_cmd(uint8_t *buf, uint16_t len)
{
    HAL_StatusTypeDef status;

    // check if SPI busy
    if (spi_state != SPI_STATE_READY)
    {
    	return 1;
    }

    // CS low
    HAL_GPIO_WritePin(SSD1681_CS_GPIO_PORT, SSD1681_CS_GPIO_PIN, GPIO_PIN_RESET);

    // set state to busy transmitting
    spi_state = SPI_STATE_BUSY_TX;
    spi_tx_complete = 0;
    spi_error_flag = 0;

    // start dma transmission
    status = HAL_SPI_Transmit_DMA(&hspi1, buf, len);

    if (status != HAL_OK)
    {
    	// CS high
    	HAL_GPIO_WritePin(SSD1681_CS_GPIO_PORT, SSD1681_CS_GPIO_PIN, GPIO_PIN_SET);
    	spi_state = SPI_STATE_READY;
    	return 1;
    }

    // wait for transmission to complete
    if (ssd1681_wait_for_spi_complete(SPI_TIMEOUT_MS) != 0)
    {
    	// abort dma transmission on timeout/error
    	HAL_SPI_Abort(&hspi1);
    	HAL_GPIO_WritePin(SSD1681_CS_GPIO_PORT, SSD1681_CS_GPIO_PIN, GPIO_PIN_SET);
    	return 1;
    }

    // CS high
    HAL_GPIO_WritePin(SSD1681_CS_GPIO_PORT, SSD1681_CS_GPIO_PIN, GPIO_PIN_SET);

    return 0;
}

/**
 * @brief      interface spi bus read
 * @param[out] *buf pointer to a data buffer
 * @param[in]  len length of data buffer
 * @return     status code
 *             - 0 success
 *             - 1 read failed
 * @note       none
 */
uint8_t ssd1681_interface_spi_read_cmd(uint8_t *buf, uint16_t len)
{
    HAL_StatusTypeDef status;

    if (spi_state != SPI_STATE_READY)
    {
    	return 1;
    }

    HAL_GPIO_WritePin(SSD1681_CS_GPIO_PORT, SSD1681_CS_GPIO_PIN, GPIO_PIN_RESET);

    spi_state = SPI_STATE_BUSY_RX;
    spi_rx_complete = 0;
    spi_error_flag = 0;

    status = HAL_SPI_Receive_DMA(&hspi1, buf, len);

    if (status != HAL_OK)
    {
    	//  CS high
    	HAL_GPIO_WritePin(SSD1681_CS_GPIO_PORT, SSD1681_CS_GPIO_PIN, GPIO_PIN_SET);
    	spi_state = SPI_STATE_READY;
    	return 1;
    }

    if (ssd1681_wait_for_spi_complete(SPI_TIMEOUT_MS) != 0)
    {
        // abort dma reception on timeout/error
        HAL_SPI_Abort(&hspi1);
        HAL_GPIO_WritePin(SSD1681_CS_GPIO_PORT, SSD1681_CS_GPIO_PIN, GPIO_PIN_SET);
        return 1;
    }

    HAL_GPIO_WritePin(SSD1681_CS_GPIO_PORT, SSD1681_CS_GPIO_PIN, GPIO_PIN_SET);

    return 0;
}

/**
 * @brief     interface delay ms
 * @param[in] ms time
 * @note      none
 */
void ssd1681_interface_delay_ms(uint32_t ms)
{
	HAL_Delay(ms);
}

/**
 * @brief     interface print format data
 * @param[in] fmt format data
 * @note      none
 */
void ssd1681_interface_debug_print(const char *const fmt, ...)
{
	char str[256];
	uint16_t len;
	va_list args;

    memset((char *)str, 0, sizeof(char) * 256);
    va_start(args, fmt);
    vsnprintf((char *)str, 255, (char const *)fmt, args);
    va_end(args);

    len = strlen((char *)str);
    HAL_UART_Transmit(&huart1, (uint8_t *)str, len, 1000);
}

/**
 * @brief  interface command && data gpio init
 * @return status code
 *         - 0 success
 *         - 1 gpio init failed
 * @note   none
 */
uint8_t ssd1681_interface_spi_cmd_data_gpio_init(void)
{
	HAL_GPIO_WritePin(SSD1681_DC_GPIO_PORT, SSD1681_DC_GPIO_PIN, GPIO_PIN_RESET);

	return 0;
}

/**
 * @brief  interface command && data gpio deinit
 * @return status code
 *         - 0 success
 *         - 1 gpio deinit failed
 * @note   none
 */
uint8_t ssd1681_interface_spi_cmd_data_gpio_deinit(void)
{
	HAL_GPIO_DeInit(SSD1681_DC_GPIO_PORT, SSD1681_DC_GPIO_PIN);
	return 0;
}

/**
 * @brief     interface command && data gpio write
 * @param[in] value written value
 * @return    status code
 *            - 0 success
 *            - 1 gpio write failed
 * @note      none
 */
uint8_t ssd1681_interface_spi_cmd_data_gpio_write(uint8_t value)
{
    GPIO_PinState state = (value == 0) ? GPIO_PIN_RESET : GPIO_PIN_SET;
    HAL_GPIO_WritePin(SSD1681_DC_GPIO_PORT, SSD1681_DC_GPIO_PIN, state);

    return 0;
}

/**
 * @brief  interface reset gpio init
 * @return status code
 *         - 0 success
 *         - 1 gpio init failed
 * @note   none
 */
uint8_t ssd1681_interface_reset_gpio_init(void)
{
	HAL_GPIO_WritePin(SSD1681_RESET_GPIO_PORT, SSD1681_RESET_GPIO_PIN, GPIO_PIN_SET);

	return 0;
}

/**
 * @brief  interface reset gpio deinit
 * @return status code
 *         - 0 success
 *         - 1 gpio deinit failed
 * @note   none
 */
uint8_t ssd1681_interface_reset_gpio_deinit(void)
{
    HAL_GPIO_DeInit(SSD1681_RESET_GPIO_PORT, SSD1681_RESET_GPIO_PIN);
    return 0;
}

/**
 * @brief     interface reset gpio write
 * @param[in] value written value
 * @return    status code
 *            - 0 success
 *            - 1 gpio write failed
 * @note      none
 */
uint8_t ssd1681_interface_reset_gpio_write(uint8_t value)
{
    GPIO_PinState state = (value == 0) ? GPIO_PIN_RESET : GPIO_PIN_SET;
    HAL_GPIO_WritePin(SSD1681_RESET_GPIO_PORT, SSD1681_RESET_GPIO_PIN, state);
    return 0;
}

/**
 * @brief  interface busy gpio init
 * @return status code
 *         - 0 success
 *         - 1 gpio init failed
 * @note   none
 */
uint8_t ssd1681_interface_busy_gpio_init(void)
{
    return 0;
}

/**
 * @brief  interface busy gpio deinit
 * @return status code
 *         - 0 success
 *         - 1 gpio deinit failed
 * @note   none
 */
uint8_t ssd1681_interface_busy_gpio_deinit(void)
{
    HAL_GPIO_DeInit(SSD1681_BUSY_GPIO_PORT, SSD1681_BUSY_GPIO_PIN);
    return 0;
}

/**
 * @brief      interface busy gpio read
 * @param[out] *value pointer to a value buffer
 * @return     status code
 *             - 0 success
 *             - 1 gpio read failed
 * @note       none
 */
uint8_t ssd1681_interface_busy_gpio_read(uint8_t *value)
{
    if (value == NULL)
    {
    	return 1;
    }

    *value = (HAL_GPIO_ReadPin(SSD1681_BUSY_GPIO_PORT, SSD1681_BUSY_GPIO_PIN) == GPIO_PIN_SET) ? 1 : 0;
    return 0;
}

/**
 * @brief  Initialize CS gpio for manual control
 * @return status code
 *         - 0 success
 *         - 1 gpio init failed
 * @note   This function is added for manual CS control
 */
uint8_t ssd1681_interface_cs_gpio_init(void)
{
    HAL_GPIO_WritePin(SSD1681_CS_GPIO_PORT, SSD1681_CS_GPIO_PIN, GPIO_PIN_SET);

    return 0;
}

/**
 * @brief  SPI Transmit Complete Callback
 * @param  hspi pointer to a SPI_HandleTypeDef structure
 * @note   This function should be called from HAL_SPI_TxCpltCallback
 */
void ssd1681_spi_tx_complete_callback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI1)
    {
        spi_tx_complete = 1;
    }
}

/**
 * @brief  SPI Receive Complete Callback
 * @param  hspi pointer to a SPI_HandleTypeDef structure
 * @note   This function should be called from HAL_SPI_RxCpltCallback
 */
void ssd1681_spi_rx_complete_callback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI1)
    {
        spi_rx_complete = 1;
    }
}

/**
 * @brief  SPI Error Callback
 * @param  hspi pointer to a SPI_HandleTypeDef structure
 * @note   This function should be called from HAL_SPI_ErrorCallback
 */
void ssd1681_spi_error_callback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI1)
    {
        spi_error_flag = 1;
    }
}

