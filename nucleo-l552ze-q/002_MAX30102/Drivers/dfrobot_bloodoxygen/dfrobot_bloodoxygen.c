/*
 * dfrobot_bloodoxygen.c
 *
 *  Created on: Nov 29, 2025
 *      Author: fahmad
 */


#include "dfrobot_bloodoxygen.h"
#include <stdio.h>

extern volatile I2C_State_t i2c_state;
extern volatile HAL_StatusTypeDef i2c_result;

#define I2C_TIMEOUT_MS      1000

static uint8_t dfrobot_write_reg(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *data, uint8_t len)
{
    uint8_t buffer[32];
    buffer[0] = reg;

    for (uint8_t i = 0; i < len; i++) {
        buffer[i + 1] = data[i];
    }

    // Set state to busy
    i2c_state = I2C_STATE_BUSY_TX;
    i2c_result = HAL_BUSY;

    // Start interrupt-based transmission
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit_IT(hi2c, DFROBOT_I2C_ADDR << 1, buffer, len + 1);

    if (status != HAL_OK) {
        printf("Write start failed: status=%d\n", status);
        i2c_state = I2C_STATE_READY;
        return 1;
    }

    // Wait for completion with timeout
    uint32_t timeout = HAL_GetTick() + I2C_TIMEOUT_MS;
    while (i2c_state == I2C_STATE_BUSY_TX) {
        if (HAL_GetTick() > timeout) {
            printf("Write timeout\n");
            i2c_state = I2C_STATE_READY;
            return 1;
        }
    }

    if (i2c_state == I2C_STATE_ERROR || i2c_result != HAL_OK) {
        printf("Write failed: error=0x%08lX\n", hi2c->ErrorCode);
        i2c_state = I2C_STATE_READY;
        return 1;
    }

    return 0;
}

static uint8_t dfrobot_read_reg(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *data, uint8_t len)
{
    HAL_StatusTypeDef status;

    // Write register address using interrupt mode
    i2c_state = I2C_STATE_BUSY_TX;
    i2c_result = HAL_BUSY;

    status = HAL_I2C_Master_Transmit_IT(hi2c, DFROBOT_I2C_ADDR << 1, &reg, 1);
    if (status != HAL_OK) {
        printf("Read reg write start failed: status=%d\n", status);
        i2c_state = I2C_STATE_READY;
        return 1;
    }

    // Wait for TX completion
    uint32_t timeout = HAL_GetTick() + I2C_TIMEOUT_MS;
    while (i2c_state == I2C_STATE_BUSY_TX) {
        if (HAL_GetTick() > timeout) {
            printf("Read reg write timeout\n");
            i2c_state = I2C_STATE_READY;
            return 1;
        }
    }

    if (i2c_state == I2C_STATE_ERROR || i2c_result != HAL_OK) {
        printf("Read reg write failed\n");
        i2c_state = I2C_STATE_READY;
        return 1;
    }

    // Read data using interrupt mode
    i2c_state = I2C_STATE_BUSY_RX;
    i2c_result = HAL_BUSY;

    status = HAL_I2C_Master_Receive_IT(hi2c, DFROBOT_I2C_ADDR << 1, data, len);
    if (status != HAL_OK) {
        printf("Read reg receive start failed: status=%d\n", status);
        i2c_state = I2C_STATE_READY;
        return 1;
    }

    // Wait for RX completion
    timeout = HAL_GetTick() + I2C_TIMEOUT_MS;
    while (i2c_state == I2C_STATE_BUSY_RX) {
        if (HAL_GetTick() > timeout) {
            printf("Read reg receive timeout\n");
            i2c_state = I2C_STATE_READY;
            return 1;
        }
    }

    if (i2c_state == I2C_STATE_ERROR || i2c_result != HAL_OK) {
        printf("Read reg receive failed\n");
        i2c_state = I2C_STATE_READY;
        return 1;
    }

    return 0;
}

uint8_t dfrobot_init(I2C_HandleTypeDef *hi2c)
{
    printf("DFRobot Blood Oxygen Sensor Init...\n");

    // Check if device is present
    HAL_StatusTypeDef status = HAL_I2C_IsDeviceReady(hi2c, DFROBOT_I2C_ADDR << 1, 3, 1000);
    if (status != HAL_OK) {
        printf("Device not found!\n");
        return 1;
    }

    printf("Device found at 0x%02X\n", DFROBOT_I2C_ADDR);

    // Start data collection
    if (dfrobot_start_collection(hi2c) != 0) {
        printf("Failed to start collection\n");
        return 1;
    }

    printf("Init successful!\n");
    return 0;
}

uint8_t dfrobot_start_collection(I2C_HandleTypeDef *hi2c)
{
    uint8_t cmd[2] = {0x00, 0x01};  // Start command
    return dfrobot_write_reg(hi2c, DFROBOT_REG_CONTROL, cmd, 2);
}

uint8_t dfrobot_stop_collection(I2C_HandleTypeDef *hi2c)
{
    uint8_t cmd[2] = {0x00, 0x02};  // Stop command
    return dfrobot_write_reg(hi2c, DFROBOT_REG_CONTROL, cmd, 2);
}

uint8_t dfrobot_read_data(I2C_HandleTypeDef *hi2c, dfrobot_data_t *data)
{
    uint8_t rbuf[8];

    if (dfrobot_read_reg(hi2c, DFROBOT_REG_SPO2_HR, rbuf, 8) != 0) {
        return 1;
    }

    // Parse SpO2 (1 byte at offset 0)
    data->spo2 = rbuf[0];
    if (data->spo2 == 0) {
        data->spo2 = -1;  // Invalid reading
    }

    // Parse heart rate (4 bytes at offset 2-5, big-endian)
    data->heartbeat = ((uint32_t)rbuf[2] << 24) |
                      ((uint32_t)rbuf[3] << 16) |
                      ((uint32_t)rbuf[4] << 8) |
                      ((uint32_t)rbuf[5]);

    if (data->heartbeat == 0) {
        data->heartbeat = -1;  // Invalid reading
    }

    return 0;
}

float dfrobot_read_temperature(I2C_HandleTypeDef *hi2c)
{
    uint8_t temp_buf[2];

    if (dfrobot_read_reg(hi2c, DFROBOT_REG_TEMP, temp_buf, 2) != 0) {
        return -999.0f;  // Error value
    }

    float temperature = temp_buf[0] * 1.0f + temp_buf[1] / 100.0f;
    return temperature;
}
