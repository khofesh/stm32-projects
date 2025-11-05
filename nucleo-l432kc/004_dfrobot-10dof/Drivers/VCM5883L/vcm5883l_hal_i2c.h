/*
 * vcm5883l_hal_i2c.h
 *
 *  Created on: Nov 5, 2025
 *      Author: fahmad
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32l4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#define VCM5883L_I2C_TIMEOUT  100 // milliseconds

typedef I2C_HandleTypeDef vcm5883l_i2c_handle_t;

bool vcm5883l_i2c_is_device_ready(vcm5883l_i2c_handle_t *hi2c, uint8_t dev_addr);

bool vcm5883l_i2c_write(vcm5883l_i2c_handle_t *hi2c, uint8_t dev_addr,
                                       uint8_t *data, uint16_t size);

bool vcm5883l_i2c_read(vcm5883l_i2c_handle_t *hi2c, uint8_t dev_addr,
                                      uint8_t *data, uint16_t size);

bool vcm5883l_i2c_mem_read(vcm5883l_i2c_handle_t *hi2c, uint8_t dev_addr,
                                          uint8_t reg_addr, uint8_t *data, uint16_t size);

bool vcm5883l_i2c_mem_write(vcm5883l_i2c_handle_t *hi2c, uint8_t dev_addr,
                                           uint8_t reg_addr, uint8_t *data, uint16_t size);

#ifdef __cplusplus
}
#endif
