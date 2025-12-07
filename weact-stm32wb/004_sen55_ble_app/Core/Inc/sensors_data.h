/**
 ******************************************************************************
 * @file    sensors_data.h
 * @brief   Global sensor data storage with thread-safe access
 ******************************************************************************
 * @attention
 *
 * This module provides centralized storage for SEN55 sensor data with
 * mechanisms to prevent race conditions between:
 * - I2C sensor reading (from BLE timer callback)
 * - USB CDC transmission (from main loop)
 * - BLE notifications (from BLE stack)
 *
 ******************************************************************************
 */

#ifndef SENSORS_DATA_H
#define SENSORS_DATA_H

#ifdef __cplusplus
extern "C" {
#endif

#include "sensirion_i2c_hal.h"
#include <stdint.h>

/**
 * @brief Global sensor data container with validity flag
 */
typedef struct {
    sen55_data_t data;           // Actual sensor readings
    uint8_t valid;               // 1 if data is valid, 0 otherwise
    uint32_t timestamp;          // HAL_GetTick() when data was last updated
    volatile uint8_t lock;       // Simple spinlock for atomic access (0=unlocked, 1=locked)
} SensorDataContainer_t;

/**
 * @brief Initialize the global sensor data storage
 */
void SensorData_Init(void);

/**
 * @brief Update sensor data (called from I2C read operation)
 * @param new_data Pointer to new sensor data
 * @return 1 on success, 0 if failed to acquire lock
 * @note This function is thread-safe and uses a spinlock
 */
uint8_t SensorData_Update(const sen55_data_t *new_data);

/**
 * @brief Get a copy of current sensor data
 * @param data_out Pointer to buffer where data will be copied
 * @return 1 if data is valid and copied, 0 if no valid data or failed to acquire lock
 * @note This function is thread-safe and uses a spinlock
 */
uint8_t SensorData_Get(sen55_data_t *data_out);

/**
 * @brief Check if sensor data is valid
 * @return 1 if valid data available, 0 otherwise
 */
uint8_t SensorData_IsValid(void);

/**
 * @brief Get timestamp of last sensor data update
 * @return Timestamp in milliseconds (HAL_GetTick())
 */
uint32_t SensorData_GetTimestamp(void);

/**
 * @brief Mark sensor data as invalid (e.g., after I2C error)
 */
void SensorData_Invalidate(void);

#ifdef __cplusplus
}
#endif

#endif /* SENSORS_DATA_H */
