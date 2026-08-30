/**
 ******************************************************************************
 * @file    bme280_app.h
 * @brief   BME280 acquisition on I2C1, non blocking forced mode sampling
 *
 * This layer owns the BME280 registers, the Bosch compensation and the forced
 * mode handshake, nothing else. When to sample, what a change means and who
 * gets told about it are decisions of app_env, not of this driver.
 ******************************************************************************
 */

#ifndef __BME280_APP_H
#define __BME280_APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * @brief compensated measurement, fixed point
 */
typedef struct
{
    int32_t  temperature;        /**< 0.01 degC, 2534 = 25.34 degC */
    uint32_t pressure;           /**< Pa */
    uint32_t humidity;           /**< 1024 * %RH, 51200 = 50 %RH */
} bme280_app_data_t;

/**
 * @brief  bring the sensor up and apply the default settings
 * @return 0 on success, 1 on failure
 */
uint8_t BME280_APP_Init(void);

/**
 * @brief  kick off one forced mode conversion
 * @return 0 on success, 1 on failure
 * @note   the part returns to sleep on its own once the conversion completes
 */
uint8_t BME280_APP_StartMeasurement(void);

/**
 * @brief  worst case conversion time for the configured oversampling
 * @return delay in milliseconds
 */
uint32_t BME280_APP_GetMeasurementDelayMs(void);

/**
 * @brief  read out and latch the result of the last started conversion
 * @param[out] out destination, may be NULL if only latching is wanted
 * @return 0 on success, 1 on failure
 */
uint8_t BME280_APP_ReadMeasurement(bme280_app_data_t *out);

/**
 * @brief  copy out the last completed sample
 * @param[out] out destination
 * @return 0 on success, 1 when no sample has been taken yet
 */
uint8_t BME280_APP_GetData(bme280_app_data_t *out);

/**
 * @brief  number of consecutive failed conversions since the last good one
 */
uint32_t BME280_APP_GetErrorCount(void);

#ifdef __cplusplus
}
#endif

#endif /* __BME280_APP_H */
