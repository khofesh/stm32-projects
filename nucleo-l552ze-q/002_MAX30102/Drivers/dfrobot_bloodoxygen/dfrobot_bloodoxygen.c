/*
 * dfrobot_bloodoxygen.c
 *
 *  Created on: Nov 29, 2025
 *      Author: fahmad
 */


#include "dfrobot_bloodoxygen.h"
#include <stdio.h>

static uint8_t dfrobot_write_reg(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *data, uint8_t len)
{
    uint8_t buffer[32];
    buffer[0] = reg;

    for (uint8_t i = 0; i < len; i++) {
        buffer[i + 1] = data[i];
    }

    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(hi2c, DFROBOT_I2C_ADDR << 1, buffer, len + 1, 1000);

    if (status != HAL_OK) {
        printf("Write failed: status=%d, error=0x%08lX\n", status, hi2c->ErrorCode);
        return 1;
    }

    return 0;
}

static uint8_t dfrobot_read_reg(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *data, uint8_t len)
{
    HAL_StatusTypeDef status;

    // Write register address
    status = HAL_I2C_Master_Transmit(hi2c, DFROBOT_I2C_ADDR << 1, &reg, 1, 1000);
    if (status != HAL_OK) {
        printf("Read reg write failed: status=%d\n", status);
        return 1;
    }

    // Read data
    status = HAL_I2C_Master_Receive(hi2c, DFROBOT_I2C_ADDR << 1, data, len, 1000);
    if (status != HAL_OK) {
        printf("Read reg receive failed: status=%d\n", status);
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
