/**
 ******************************************************************************
 * @file    display_app.h
 * @brief   SSD1315 OLED rendering of the BME280 measurement
 ******************************************************************************
 */

#ifndef __DISPLAY_APP_H
#define __DISPLAY_APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "bme280_app.h"

/**
 * @brief  bring the OLED up and paint the splash screen
 * @return 0 on success, 1 on failure
 */
uint8_t DISPLAY_APP_Init(void);

/**
 * @brief  render one measurement
 * @param[in] data measurement to show
 * @return 0 on success, 1 on failure
 */
uint8_t DISPLAY_APP_ShowMeasurement(const bme280_app_data_t *data);

#ifdef __cplusplus
}
#endif

#endif /* __DISPLAY_APP_H */
