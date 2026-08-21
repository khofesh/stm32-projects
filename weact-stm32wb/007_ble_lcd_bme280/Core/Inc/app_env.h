/**
 ******************************************************************************
 * @file    app_env.h
 * @brief   Environmental sampling policy and STABLE/ACTIVE/INTERACTIVE states
 *
 * Owns the sensor schedule, the previous/current readings and the change
 * detection. Sampling is driven by the STM32WB timer server (RTC backed, so it
 * survives STOP2) and the work is dispatched through the sequencer; there is
 * no polling from the main loop.
 ******************************************************************************
 */

#ifndef __APP_ENV_H
#define __APP_ENV_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "bme280_app.h"

typedef enum
{
    APP_STATE_STABLE = 0,        /**< nothing is changing, sample slowly */
    APP_STATE_ACTIVE,            /**< environment is moving, sample fast */
    APP_STATE_INTERACTIVE        /**< user is looking at the display */
} AppState;

/**
 * @brief  register the sequencer tasks, create the timers and take the first
 *         measurement
 * @note   call after MX_APPE_Init(), the timer server must already be up
 */
void APP_ENV_Init(void);

/**
 * @brief  current operating state
 */
AppState APP_ENV_GetState(void);

/**
 * @brief  last completed reading
 * @param[out] out destination
 * @return 0 on success, 1 when no sample has been taken yet
 */
uint8_t APP_ENV_GetReading(bme280_app_data_t *out);

/**
 * @brief  a user interaction happened, enter INTERACTIVE
 * @note   safe to call from an interrupt, the work is deferred to a task
 */
void APP_ENV_OnUserInteraction(void);

/**
 * @brief  the display timed out, fall back to ACTIVE or STABLE
 */
void APP_ENV_OnDisplayTimeout(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_ENV_H */
