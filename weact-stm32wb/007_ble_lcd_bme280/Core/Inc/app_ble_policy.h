/**
 ******************************************************************************
 * @file    app_ble_policy.h
 * @brief   BLE publication policy: notification thresholds, advertising
 *          payload and connection parameter management
 *
 * The last value reported over BLE is tracked separately from the previous
 * sensor reading: they answer different questions, and conflating them would
 * either drop notifications or send redundant ones.
 ******************************************************************************
 */

#ifndef __APP_BLE_POLICY_H
#define __APP_BLE_POLICY_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "app_env.h"
#include "bme280_app.h"

/**
 * @brief  register the publication task
 */
void APP_BLE_POLICY_Init(void);

/**
 * @brief  a fresh reading arrived, publish it if it is worth publishing
 */
void APP_BLE_POLICY_OnReading(const bme280_app_data_t *data);

/**
 * @brief  a central connected
 */
void APP_BLE_POLICY_OnConnected(uint16_t connection_handle);

/**
 * @brief  the central went away
 */
void APP_BLE_POLICY_OnDisconnected(void);

/**
 * @brief  the client subscribed to or unsubscribed from the environmental
 *         characteristic
 */
void APP_BLE_POLICY_OnNotifyEnabled(uint8_t enabled);

/**
 * @brief  the application state changed, review the connection interval
 * @note   a request is only issued on an actual transition, never repeatedly
 */
void APP_BLE_POLICY_OnStateChanged(AppState state);

#ifdef __cplusplus
}
#endif

#endif /* __APP_BLE_POLICY_H */
