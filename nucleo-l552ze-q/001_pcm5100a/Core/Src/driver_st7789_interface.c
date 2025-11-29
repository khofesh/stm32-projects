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
 * @file      driver_st7789_interface.c
 * @brief     driver st7789 interface source file for NUCLEO-L552ZE-Q
 * @version   1.0.0
 * @author    Modified for NUCLEO-L552ZE-Q
 * @date      2025-11-29
 *
 * Pin Connections:
 * - SCK (SPI_SCK) -> D13 (PA5) - SPI1_SCK
 * - SDA (SPI_MOSI) -> D11 (PA7) - SPI1_MOSI
 * - RES (Reset) -> D9 (PC7) - GPIO
 * - DC (Data/Command) -> D9 (PC7) - GPIO
 * - BLK (Backlight) -> 3.3V (always on) or D8 (PA9) for PWM control
 * - VCC -> 3.3V
 * - GND -> GND
 *
 * Note: No CS pin - display is always selected
 *
 * USART for debug (optional):
 * - TX -> PA2 (or your configured UART TX)
 * - RX -> PA3 (or your configured UART RX)
 */

#include "driver_st7789_interface.h"
#include "main.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

extern SPI_HandleTypeDef hspi1;

#define ST7789_RST_PIN      GPIO_PIN_15     // PD15 (D9) - Reset pin
#define ST7789_RST_PORT     GPIOD
#define ST7789_DC_PIN       GPIO_PIN_12     // PF12 (D8) - Data/Command pin
#define ST7789_DC_PORT      GPIOF


/**
 * @brief  interface spi bus init
 * @return status code
 *         - 0 success
 *         - 1 spi init failed
 * @note   none
 */
uint8_t st7789_interface_spi_init(void)
{
    if (HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_READY)
    {
    	return 0;
    }
    else
    {
    	return 1;
    }
}

/**
 * @brief  interface spi bus deinit
 * @return status code
 *         - 0 success
 *         - 1 spi deinit failed
 * @note   none
 */
uint8_t st7789_interface_spi_deinit(void)
{
    if (HAL_SPI_DeInit(&hspi1) == HAL_OK)
    {
    	return 0;
    }
    else
    {
    	return 1;
    }
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
uint8_t st7789_interface_spi_write_cmd(uint8_t *buf, uint16_t len)
{
    HAL_StatusTypeDef status;

    status = HAL_SPI_Transmit(&hspi1, buf, len, HAL_MAX_DELAY);

    if (status == HAL_OK)
    {
        return 0;
    }
    else
    {
        return 1;
    }
}

/**
 * @brief     interface delay ms
 * @param[in] ms time
 * @note      none
 */
void st7789_interface_delay_ms(uint32_t ms)
{
	HAL_Delay(ms);
}

/**
 * @brief     interface print format data
 * @param[in] fmt format data
 * @note      none
 */
void st7789_interface_debug_print(const char *const fmt, ...)
{
    char str[256];
    va_list args;

    /* Format the string */
    va_start(args, fmt);
    vsnprintf(str, sizeof(str), fmt, args);
    va_end(args);

    printf(str);
}

/**
 * @brief  interface command && data gpio init
 * @return status code
 *         - 0 success
 *         - 1 gpio init failed
 * @note   none
 */
uint8_t st7789_interface_cmd_data_gpio_init(void)
{
	HAL_GPIO_WritePin(ST7789_DC_PORT, ST7789_DC_PIN, GPIO_PIN_SET);
	return 0;
}

/**
 * @brief  interface command && data gpio deinit
 * @return status code
 *         - 0 success
 *         - 1 gpio deinit failed
 * @note   none
 */
uint8_t st7789_interface_cmd_data_gpio_deinit(void)
{
	HAL_GPIO_WritePin(ST7789_DC_PORT, ST7789_DC_PIN, GPIO_PIN_RESET);
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
uint8_t st7789_interface_cmd_data_gpio_write(uint8_t value)
{
    if (value == 0)
    {
        /* Command mode - DC low */
        HAL_GPIO_WritePin(ST7789_DC_PORT, ST7789_DC_PIN, GPIO_PIN_RESET);
    }
    else
    {
        /* Data mode - DC high */
        HAL_GPIO_WritePin(ST7789_DC_PORT, ST7789_DC_PIN, GPIO_PIN_SET);
    }
    return 0;
}

/**
 * @brief  interface reset gpio init
 * @return status code
 *         - 0 success
 *         - 1 gpio init failed
 * @note   none
 */
uint8_t st7789_interface_reset_gpio_init(void)
{
	HAL_GPIO_WritePin(ST7789_RST_PORT, ST7789_RST_PIN, GPIO_PIN_SET);
    return 0;
}

/**
 * @brief  interface reset gpio deinit
 * @return status code
 *         - 0 success
 *         - 1 gpio deinit failed
 * @note   none
 */
uint8_t st7789_interface_reset_gpio_deinit(void)
{
	HAL_GPIO_WritePin(ST7789_RST_PORT, ST7789_RST_PIN, GPIO_PIN_RESET);
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
uint8_t st7789_interface_reset_gpio_write(uint8_t value)
{
    if (value == 0)
    {
        /* Reset active - RST low */
        HAL_GPIO_WritePin(ST7789_RST_PORT, ST7789_RST_PIN, GPIO_PIN_RESET);
    }
    else
    {
        /* Reset inactive - RST high */
        HAL_GPIO_WritePin(ST7789_RST_PORT, ST7789_RST_PIN, GPIO_PIN_SET);
    }
    return 0;
}
