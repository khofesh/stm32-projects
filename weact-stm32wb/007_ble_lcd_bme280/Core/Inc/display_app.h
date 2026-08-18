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
 * @brief  compose one measurement into the frame buffer and queue it
 * @param[in] data measurement to show
 * @return 0 on success, 1 on failure
 * @note   no bus traffic here, DISPLAY_APP_Process() does the pushing
 */
uint8_t DISPLAY_APP_ShowMeasurement(const bme280_app_data_t *data);

/**
 * @brief  push at most one page of a queued frame, call from the main loop
 */
void DISPLAY_APP_Process(void);

#ifdef __cplusplus
}
#endif

#endif /* __DISPLAY_APP_H */
