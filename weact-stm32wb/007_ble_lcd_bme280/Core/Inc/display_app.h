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
 * @brief  push at most one page of a queued frame
 * @note   driven by the display task, one page per call
 */
void DISPLAY_APP_Process(void);

/**
 * @brief  is a composed frame still waiting to be pushed to the panel
 * @return 1 while pages remain, 0 when the panel is idle
 */
uint8_t DISPLAY_APP_HasPendingFrame(void);

/**
 * @brief  power abstraction, currently the SSD1315 display ON/OFF commands
 * @note   when a load switch is fitted, drive it here behind the same calls.
 *         Define OLED_PWR_GPIO_Port / OLED_PWR_Pin in main.h to enable it.
 * @return 0 on success, 1 on failure
 */
uint8_t DISPLAY_APP_PowerOn(void);

/**
 * @brief  see DISPLAY_APP_PowerOn()
 * @return 0 on success, 1 on failure
 */
uint8_t DISPLAY_APP_PowerOff(void);

/**
 * @brief  is the panel currently lit
 * @return 1 when powered, 0 otherwise
 */
uint8_t DISPLAY_APP_IsPowered(void);

#ifdef __cplusplus
}
#endif

#endif /* __DISPLAY_APP_H */
