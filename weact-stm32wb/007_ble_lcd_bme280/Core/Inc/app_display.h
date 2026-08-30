/**
 ******************************************************************************
 * @file    app_display.h
 * @brief   Display policy: when the OLED is awake and what it shows
 *
 * Decides the timeout, formats the readings and drives the OLED transport in
 * display_app.c. It never talks to the panel registers itself.
 ******************************************************************************
 */

#ifndef __APP_DISPLAY_H
#define __APP_DISPLAY_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "bme280_app.h"

/**
 * @brief  register the tasks and timer, leave the panel off
 */
void APP_DISPLAY_Init(void);

/**
 * @brief  a fresh reading arrived, repaint only if the panel is awake and the
 *         displayed values actually changed
 */
void APP_DISPLAY_OnReading(const bme280_app_data_t *data);

/**
 * @brief  wake the panel and (re)start the inactivity timeout
 */
void APP_DISPLAY_Wake(void);

/**
 * @brief  is the panel currently awake
 */
uint8_t APP_DISPLAY_IsAwake(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_DISPLAY_H */
