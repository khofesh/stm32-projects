/**
 ******************************************************************************
 * @file    sensors_data.c
 * @brief   Global sensor data storage implementation
 ******************************************************************************
 */

#include "sensors_data.h"
#include "stm32wbxx_hal.h"
#include <string.h>

/* Private variables ---------------------------------------------------------*/
static SensorDataContainer_t g_sensor_data;

/* Private function prototypes -----------------------------------------------*/
static uint8_t acquire_lock(volatile uint8_t *lock, uint32_t timeout_ms);
static void release_lock(volatile uint8_t *lock);

/**
 * @brief Initialize the global sensor data storage
 */
void SensorData_Init(void)
{
    memset(&g_sensor_data, 0, sizeof(SensorDataContainer_t));
    g_sensor_data.valid = 0;
    g_sensor_data.lock = 0;
}

/**
 * @brief Update sensor data (called from I2C read operation)
 * @param new_data Pointer to new sensor data
 * @return 1 on success, 0 if failed to acquire lock
 */
uint8_t SensorData_Update(const sen55_data_t *new_data)
{
    if (new_data == NULL) {
        return 0;
    }

    // Try to acquire lock with 10ms timeout
    if (!acquire_lock(&g_sensor_data.lock, 10)) {
        return 0;  // Failed to acquire lock
    }

    // Critical section - update data
    memcpy(&g_sensor_data.data, new_data, sizeof(sen55_data_t));
    g_sensor_data.valid = 1;
    g_sensor_data.timestamp = HAL_GetTick();

    release_lock(&g_sensor_data.lock);
    return 1;
}

/**
 * @brief Get a copy of current sensor data
 * @param data_out Pointer to buffer where data will be copied
 * @return 1 if data is valid and copied, 0 if no valid data or failed to acquire lock
 */
uint8_t SensorData_Get(sen55_data_t *data_out)
{
    if (data_out == NULL) {
        return 0;
    }

    // Try to acquire lock with 10ms timeout
    if (!acquire_lock(&g_sensor_data.lock, 10)) {
        return 0;  // Failed to acquire lock
    }

    // Critical section - check validity and copy data
    uint8_t is_valid = g_sensor_data.valid;
    if (is_valid) {
        memcpy(data_out, &g_sensor_data.data, sizeof(sen55_data_t));
    }

    release_lock(&g_sensor_data.lock);
    return is_valid;
}

/**
 * @brief Check if sensor data is valid
 * @return 1 if valid data available, 0 otherwise
 */
uint8_t SensorData_IsValid(void)
{
    // Reading a single byte is atomic on ARM Cortex-M, no lock needed
    return g_sensor_data.valid;
}

/**
 * @brief Get timestamp of last sensor data update
 * @return Timestamp in milliseconds (HAL_GetTick())
 */
uint32_t SensorData_GetTimestamp(void)
{
    // Reading uint32_t might not be atomic, use lock
    if (!acquire_lock(&g_sensor_data.lock, 10)) {
        return 0;
    }

    uint32_t ts = g_sensor_data.timestamp;
    release_lock(&g_sensor_data.lock);
    return ts;
}

/**
 * @brief Mark sensor data as invalid (e.g., after I2C error)
 */
void SensorData_Invalidate(void)
{
    if (!acquire_lock(&g_sensor_data.lock, 10)) {
        return;
    }

    g_sensor_data.valid = 0;
    release_lock(&g_sensor_data.lock);
}

/* Private functions ---------------------------------------------------------*/

/**
 * @brief Simple spinlock implementation with timeout
 * @param lock Pointer to lock variable
 * @param timeout_ms Timeout in milliseconds
 * @return 1 if lock acquired, 0 if timeout
 */
static uint8_t acquire_lock(volatile uint8_t *lock, uint32_t timeout_ms)
{
    uint32_t start_tick = HAL_GetTick();
    
    while (1) {
        // Try to acquire lock using atomic test-and-set
        uint32_t primask = __get_PRIMASK();
        __disable_irq();
        
        if (*lock == 0) {
            *lock = 1;
            __set_PRIMASK(primask);
            return 1;  // Lock acquired
        }
        
        __set_PRIMASK(primask);
        
        // Check timeout
        if ((HAL_GetTick() - start_tick) >= timeout_ms) {
            return 0;  // Timeout
        }
        
        // Small delay to prevent bus contention
        for (volatile int i = 0; i < 100; i++);
    }
}

/**
 * @brief Release spinlock
 * @param lock Pointer to lock variable
 */
static void release_lock(volatile uint8_t *lock)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    *lock = 0;
    __set_PRIMASK(primask);
}
