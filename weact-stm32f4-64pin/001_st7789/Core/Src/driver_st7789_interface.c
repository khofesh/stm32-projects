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
 * @brief     driver st7789 interface source file for STM32F446RE
 * @version   1.0.0
 * @author    Modified for STM32F446RE
 * @date      2025-07-13
 *
 * Pin Connections:
 * - SCL (SPI_SCK) -> PA5
 * - SDA (SPI_MOSI) -> PA7
 * - CS (SPI_NSS) -> PA4
 * - RST (Reset) -> PA2
 * - DC (Data/Command) -> PA1
 * - BL (Backlight) -> VCC (always on)
 *
 * USART for debug:
 * - TX -> PA9
 * - RX -> PA10
 */

#include "driver_st7789_interface.h"
#include "main.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

/* External SPI and UART handles - defined in main.c */
extern SPI_HandleTypeDef hspi1;
extern UART_HandleTypeDef huart1;

/* GPIO Pin Definitions - Match your main.h definitions */
#define ST7789_CS_PIN       GPIO_PIN_4      // PA4
#define ST7789_CS_PORT      GPIOA
#define ST7789_RST_PIN      LCD_RST_Pin     // PA2 (use your defined name)
#define ST7789_RST_PORT     GPIOA
#define ST7789_DC_PIN       LCD_DC_Pin      // PA1 (use your defined name)
#define ST7789_DC_PORT      GPIOA

/* Private functions */
static void st7789_cs_low(void);
static void st7789_cs_high(void);

/**
 * @brief  interface spi bus init
 * @return status code
 *         - 0 success
 *         - 1 spi init failed
 * @note   SPI1 should be already initialized in main.c
 */
uint8_t st7789_interface_spi_init(void)
{
    /* SPI1 is already initialized in main.c via MX_SPI1_Init() */
    /* just check if it's ready */
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

    /* Pull CS low to start transmission */
    st7789_cs_low();

    /* Transmit data via SPI */
    status = HAL_SPI_Transmit(&hspi1, buf, len, HAL_MAX_DELAY);

    /* Pull CS high to end transmission */
    st7789_cs_high();

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
 * @param[in] ms time in milliseconds
 * @note      none
 */
void st7789_interface_delay_ms(uint32_t ms)
{
    HAL_Delay(ms);
}

/**
 * @brief     interface print format data
 * @param[in] fmt format data
 * @note      Output via UART1
 */
void st7789_interface_debug_print(const char *const fmt, ...)
{
    char str[256];
    va_list args;

    /* Format the string */
    va_start(args, fmt);
    vsnprintf(str, sizeof(str), fmt, args);
    va_end(args);

    /* Send via UART */
    HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), HAL_MAX_DELAY);
}

/**
 * @brief  interface command && data gpio init
 * @return status code
 *         - 0 success
 *         - 1 gpio init failed
 * @note   DC pin (PA1) should be already configured in main.c
 */
uint8_t st7789_interface_cmd_data_gpio_init(void)
{
    /* GPIO should be already initialized in main.c via MX_GPIO_Init() */
    /* Set DC pin to output high (data mode) by default */
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
    /* Set DC pin to low before deinit */
    HAL_GPIO_WritePin(ST7789_DC_PORT, ST7789_DC_PIN, GPIO_PIN_RESET);
    return 0;
}

/**
 * @brief     interface command && data gpio write
 * @param[in] value written value (0 = command, 1 = data)
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
 * @note   RST pin (PA2) should be already configured in main.c
 */
uint8_t st7789_interface_reset_gpio_init(void)
{
    /* GPIO should be already initialized in main.c via MX_GPIO_Init() */
    /* Set reset pin high (not in reset) by default */
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
    /* Set reset pin to low before deinit */
    HAL_GPIO_WritePin(ST7789_RST_PORT, ST7789_RST_PIN, GPIO_PIN_RESET);
    return 0;
}

/**
 * @brief     interface reset gpio write
 * @param[in] value written value (0 = reset active, 1 = reset inactive)
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

/**
 * @brief  Pull CS pin low (select device)
 * @note   Private function
 */
static void st7789_cs_low(void)
{
    HAL_GPIO_WritePin(ST7789_CS_PORT, ST7789_CS_PIN, GPIO_PIN_RESET);
}

/**
 * @brief  Pull CS pin high (deselect device)
 * @note   Private function
 */
static void st7789_cs_high(void)
{
    HAL_GPIO_WritePin(ST7789_CS_PORT, ST7789_CS_PIN, GPIO_PIN_SET);
}
