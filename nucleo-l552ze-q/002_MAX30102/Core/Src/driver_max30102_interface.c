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
 * @file      driver_max30102_interface_template.c
 * @brief     driver max30102 interface template source file
 * @version   1.0.0
 * @author    Shifeng Li
 * @date      2021-11-13
 *
 * <h3>history</h3>
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2021/11/13  <td>1.0      <td>Shifeng Li  <td>first upload
 * </table>
 */

#include "driver_max30102_interface.h"
#include "driver_max30102.h"
#include "main.h"
#include <stdarg.h>
#include <stdio.h>

extern I2C_HandleTypeDef hi2c1;

typedef enum {
	I2C_STATE_READY = 2,
	I2C_STATE_BUSY_TX,
	I2C_STATE_BUSY_RX,
	I2C_STATE_ERROR
} i2c_state_t;

static volatile i2c_state_t i2c_state = I2C_STATE_READY;
static volatile uint8_t i2c_operation_complete = 0;
static volatile uint8_t i2c_error_occurred = 0;

/**
 * @brief  interface iic bus init
 * @return status code
 *         - 0 success
 *         - 1 iic init failed
 * @note   none
 */
uint8_t max30102_interface_iic_init(void)
{
    return 0;
}

/**
 * @brief  interface iic bus deinit
 * @return status code
 *         - 0 success
 *         - 1 iic deinit failed
 * @note   none
 */
uint8_t max30102_interface_iic_deinit(void)
{
    return 0;
}

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
uint8_t max30102_interface_iic_read(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len)
{
	uint32_t timeout = 1000;
	uint32_t start_time = HAL_GetTick();

	uint8_t device_addr = addr << 1;

	while (i2c_state != I2C_STATE_READY)
	{
		if ((HAL_GetTick() - start_time) >= timeout)
		{
			return 1;
		}
		HAL_Delay(1);
	}

	i2c_operation_complete = 0;
	i2c_error_occurred = 0;
	i2c_state = I2C_STATE_BUSY_RX;

	if (HAL_I2C_Master_Receive_IT(&hi2c1, device_addr, buf, len) != HAL_OK)
	{
		i2c_state = I2C_STATE_READY;
		return 1;
	}

	start_time = HAL_GetTick();
	while (!i2c_operation_complete && !i2c_error_occurred)
	{
		if ((HAL_GetTick() - start_time) >= timeout)
		{
			i2c_state = I2C_STATE_READY;
			return 1;
		}
		HAL_Delay(1);
	}

    if (i2c_error_occurred) {
        i2c_state = I2C_STATE_READY;
        return 1;
    }

    i2c_state = I2C_STATE_READY;

    return 0;
}

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
uint8_t max30102_interface_iic_write(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len)
{
	uint32_t timeout = 1000;
	uint32_t start_time = HAL_GetTick();

	uint8_t device_addr = addr << 1;

	while (i2c_state != I2C_STATE_READY)
	{
		if ((HAL_GetTick() - start_time) >= timeout)
		{
			return 1;
		}
		HAL_Delay(1);
	}

	i2c_operation_complete = 0;
	i2c_error_occurred = 0;
	i2c_state = I2C_STATE_BUSY_TX;

	if (HAL_I2C_Master_Transmit_IT(&hi2c1, device_addr, buf, len) != HAL_OK)
	{
		i2c_state = I2C_STATE_READY;
		return 1;
	}

	start_time = HAL_GetTick();
	while (!i2c_operation_complete && !i2c_error_occurred)
	{
		if ((HAL_GetTick() - start_time) >= timeout)
		{
			i2c_state = I2C_STATE_READY;
			return 1;
		}
		HAL_Delay(1);
	}

    if (i2c_error_occurred) {
        i2c_state = I2C_STATE_READY;
        return 1;
    }

    i2c_state = I2C_STATE_READY;

    return 0;
}

/**
 * @brief     interface delay ms
 * @param[in] ms time
 * @note      none
 */
void max30102_interface_delay_ms(uint32_t ms)
{
	HAL_Delay(ms);
}

/**
 * @brief     interface print format data
 * @param[in] fmt format data
 * @note      none
 */
void max30102_interface_debug_print(const char *const fmt, ...)
{
    char str[256];
    va_list args;
    
    va_start(args, fmt);
    vsnprintf(str, sizeof(str), fmt, args);
    va_end(args);

    printf(str);
}

/**
 * @brief     interface receive callback
 * @param[in] type irq type
 * @note      none
 */
void max30102_interface_receive_callback(uint8_t type)
{
    switch (type)
    {
        case MAX30102_INTERRUPT_STATUS_FIFO_FULL :
        {
            max30102_interface_debug_print("max30102: irq fifo full.\n");
            
            break;
        }
        case MAX30102_INTERRUPT_STATUS_PPG_RDY :
        {
            max30102_interface_debug_print("max30102: irq ppg rdy.\n");
            
            break;
        }
        case MAX30102_INTERRUPT_STATUS_ALC_OVF :
        {
            max30102_interface_debug_print("max30102: irq alc ovf.\n");
            
            break;
        }
        case MAX30102_INTERRUPT_STATUS_PWR_RDY :
        {
            max30102_interface_debug_print("max30102: irq pwr rdy.\n");
            
            break;
        }
        case MAX30102_INTERRUPT_STATUS_DIE_TEMP_RDY :
        {
            max30102_interface_debug_print("max30102: irq die temp rdy.\n");
            
            break;
        }
        default :
        {
            max30102_interface_debug_print("max30102: unknown code.\n");
            
            break;
        }
    }
}

/**
 * @brief  I2C Master Tx Transfer completed callback.
 * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
 *                the configuration information for the specified I2C.
 * @retval None
 */
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	if (hi2c->Instance == I2C1)
	{
		i2c_operation_complete = 1;
	}
}

/**
 * @brief  I2C Master Rx Transfer completed callback.
 * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
 *                the configuration information for the specified I2C.
 * @retval None
 */
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1) {
        i2c_operation_complete = 1;
    }
}

/**
 * @brief  I2C error callback.
 * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
 *                the configuration information for the specified I2C.
 * @retval None
 */
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1) {
        i2c_error_occurred = 1;
        i2c_state = I2C_STATE_ERROR;
    }
}

