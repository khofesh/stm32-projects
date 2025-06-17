/*
 * bme280_stm32.h
 *
 *  Created on: Jun 17, 2025
 *      Author: fahmad
 */

#pragma once

#include <string.h>
#include "bme280.h"
#include "main.h"

int8_t user_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);
int8_t user_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr);
void user_delay_us(uint32_t period, void *intf_ptr);
int8_t bme280_interface_init(struct bme280_dev *dev, uint8_t intf);
int8_t bme280_init_sensor(struct bme280_dev *dev);
int8_t bme280_read_sensor_data(struct bme280_data *comp_data, struct bme280_dev *dev);
