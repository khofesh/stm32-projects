/*
 * dfrobot_bloodoxygen.h
 *
 *  Created on: Nov 29, 2025
 *      Author: fahmad
 */

#pragma once

#include "main.h"

#define DFROBOT_I2C_ADDR        0x57

// Register addresses
#define DFROBOT_REG_SPO2_HR     0x0C    // SpO2 and Heart Rate (8 bytes)
#define DFROBOT_REG_TEMP        0x14    // Temperature (2 bytes)
#define DFROBOT_REG_CONTROL     0x20    // Control register (2 bytes)

#define DFROBOT_REG_DEVICE_ID   0x04    // Device ID (2 bytes) = 0x0020
#define DFROBOT_REG_SAVE_CFG    0x1A    // Save configuration (2 bytes)
#define DFROBOT_REG_BAUD_RATE   0x06    // UART baud rate config (2 bytes)

// Control commands
#define DFROBOT_CMD_START       0x0001  // Start collection
#define DFROBOT_CMD_STOP        0x0002  // Stop collection

typedef struct {
    int16_t spo2;       // Oxygen saturation (%)
    int32_t heartbeat;  // Heart rate (BPM)
} dfrobot_data_t;

uint8_t dfrobot_init(I2C_HandleTypeDef *hi2c);
uint8_t dfrobot_start_collection(I2C_HandleTypeDef *hi2c);
uint8_t dfrobot_stop_collection(I2C_HandleTypeDef *hi2c);
uint8_t dfrobot_read_data(I2C_HandleTypeDef *hi2c, dfrobot_data_t *data);
float dfrobot_read_temperature(I2C_HandleTypeDef *hi2c);
