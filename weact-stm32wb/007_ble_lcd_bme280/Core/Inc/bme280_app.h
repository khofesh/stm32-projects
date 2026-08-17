/**
 ******************************************************************************
 * @file    bme280_app.h
 * @brief   BME280 acquisition on I2C1, non blocking forced mode sampling
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
 * @brief  drive the sampling state machine, call from the main loop
 * @return 1 when a fresh sample became available, 0 otherwise
 */
uint8_t BME280_APP_Process(void);

/**
 * @brief  copy out the last completed sample
 * @param[out] out destination
 * @return 0 on success, 1 when no sample has been taken yet
 */
uint8_t BME280_APP_GetData(bme280_app_data_t *out);

#ifdef __cplusplus
}
#endif

#endif /* __BME280_APP_H */
