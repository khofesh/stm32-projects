/**
 ******************************************************************************
 * @file    app_config.h
 * @brief   Application policy tuning values
 *
 * Every tunable of the adaptive sampling / display / BLE policy lives here.
 * Nothing in this header depends on HAL or on the BLE stack, so it can be
 * included from drivers as well as from the application layer.
 ******************************************************************************
 */

#ifndef __APP_CONFIG_H
#define __APP_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

/* ---------------------------------------------------------------------------
 * Adaptive sampling
 * ------------------------------------------------------------------------- */

#define SENSOR_STABLE_INTERVAL_MS          20000U
#define SENSOR_ACTIVE_INTERVAL_MS           1000U

/* the BME280 driver reports temperature in 0.01 degC and humidity in 1/1024 %RH,
   so the thresholds are expressed in the same fixed point units to keep the
   change detection free of floating point */
#define TEMP_CHANGE_THRESHOLD_C_X100          10      /**< 0.10 degC */
#define HUMIDITY_CHANGE_THRESHOLD_X1024     1024U     /**< 1.00 %RH  */

#define ACTIVE_STABILITY_TIMEOUT_MS        30000U

/* a reading is considered stale once the sampling interval in use exceeds this,
   which is how the INTERACTIVE path decides whether to force a fresh sample.
   No wall clock is needed, so the test stays correct across STOP2 */
#define SENSOR_DATA_STALE_MS                5000U

/* consecutive unchanged ACTIVE samples needed before falling back to STABLE */
#define ACTIVE_STABLE_SAMPLE_COUNT \
  (ACTIVE_STABILITY_TIMEOUT_MS / SENSOR_ACTIVE_INTERVAL_MS)

/* ---------------------------------------------------------------------------
 * Display
 * ------------------------------------------------------------------------- */

#define OLED_TIMEOUT_MS                    10000U

/* ---------------------------------------------------------------------------
 * Button
 * ------------------------------------------------------------------------- */

#define BUTTON_DEBOUNCE_MS                   200U

/* ---------------------------------------------------------------------------
 * BLE notification policy
 * ------------------------------------------------------------------------- */

#define BLE_TEMP_NOTIFY_DELTA_C_X100           5      /**< 0.05 degC */
#define BLE_HUMIDITY_NOTIFY_DELTA_X1024      512U     /**< 0.50 %RH  */
#define BLE_PRESSURE_NOTIFY_DELTA_PA          20U     /**< 0.20 hPa  */

/* temperature (int32, 0.01 degC) + pressure (uint32, Pa) + humidity (uint32, 1/1024 %RH) */
#define BLE_ENV_PAYLOAD_LEN                   12U

/* ---------------------------------------------------------------------------
 * BLE advertising, units of 0.625 ms
 * ------------------------------------------------------------------------- */

#define BLE_ADV_INTERVAL_MIN              (0x0640)    /**< 1000 ms */
#define BLE_ADV_INTERVAL_MAX              (0x0680)    /**< 1040 ms */

/* Bluetooth SIG company identifier used in the advertised manufacturer data */
#define BLE_ADV_COMPANY_ID                (0x0030)    /**< STMicroelectronics */

/* ---------------------------------------------------------------------------
 * BLE connection parameters, intervals in units of 1.25 ms,
 * supervision timeout in units of 10 ms
 * ------------------------------------------------------------------------- */

#define BLE_CONN_INTERVAL_RELAXED_MIN        (400)    /**< 500 ms  */
#define BLE_CONN_INTERVAL_RELAXED_MAX        (800)    /**< 1000 ms */
#define BLE_CONN_INTERVAL_FAST_MIN            (80)    /**< 100 ms  */
#define BLE_CONN_INTERVAL_FAST_MAX           (200)    /**< 250 ms  */
#define BLE_CONN_LATENCY                       (0)
#define BLE_CONN_SUPERVISION_TIMEOUT         (500)    /**< 5 s */

/* ---------------------------------------------------------------------------
 * Instrumentation
 * ------------------------------------------------------------------------- */

/* set to 1 and define POWER_TRACE_GPIO_Port / POWER_TRACE_Pin in main.h to
   toggle a pin around the sensor measurement for scope based power profiling */
#define APP_POWER_TRACE_ENABLED                0

/* Blink the board LED (PE4) once per BLE notification the stack accepted.
   A debug aid only: the LED draws roughly 1-2 mA while lit, which averages
   about 30 uA at the 1 Hz ACTIVE notify rate - several times the STOP2 budget
   this design targets. Keep it at 0 for any current measurement. */
#define LED_NOTIFY_BLINK                       0
#define LED_NOTIFY_BLINK_MS                   20U

/* 1 when the LED lights on a high level, 0 when the cathode side is driven.
   Check the board schematic; a wrong value leaves the LED permanently lit */
#define LED_ACTIVE_HIGH                        1

#ifdef __cplusplus
}
#endif

#endif /* __APP_CONFIG_H */
