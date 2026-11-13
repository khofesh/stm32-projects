#ifndef __CALIBRATION_H
#define __CALIBRATION_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>

#include "calib_math.h"
#include "vec3.h"

/* Calibration state machine, SPEC §22.2. Inputs are raw body-frame samples;
 * the axis map is applied before they are fed here. */

#define CALIB_GYRO_SAMPLES 200u
#define CALIB_ACCEL_SAMPLES 100u
#define CALIB_BARO_SAMPLES 50u
#define CALIB_MAG_MIN_SAMPLES 2000u /* >= 20 s at the 100 Hz IMU rate */

/* Standard error of the 200-sample mean: 0.05 deg/s (per-sample std up to
 * ~0.7 deg/s, above the I3G4250D noise of ~0.2 deg/s at 25 Hz bandwidth). */
#define CALIB_GYRO_MAX_SEM_RAD_S 0.0008726646f
#define CALIB_ACCEL_MAX_STD_MS2 0.1f /* per-axis, rejects a moving pose */
#define CALIB_MAG_MIN_SPAN_UT 30.0f
#define CALIB_MAG_MAX_SPAN_RATIO 1.5f
#define CALIB_BARO_MAX_STD_PA 3.0f

/* calib_state byte: bits 3:0 = capture active, bits 6:4 = accel
 * orientations already accepted (0..6), bit 7 = last attempt rejected
 * (cleared by the next command). */
#define CALIB_STATE_GYRO_ACTIVE (1u << 0)
#define CALIB_STATE_ACCEL_ACTIVE (1u << 1)
#define CALIB_STATE_MAG_ACTIVE (1u << 2)
#define CALIB_STATE_BARO_ACTIVE (1u << 3)
#define CALIB_STATE_ACCEL_COUNT_SHIFT 4u
#define CALIB_STATE_ACCEL_COUNT_MASK 0x07u
#define CALIB_STATE_REJECTED (1u << 7)

/* sensor_ok uses the STATUS_*_OK bits from telemetry.h. Loads the stored
 * parameters and starts the boot gyro-bias attempt when the gyro is present. */
void calibration_init(uint16_t sensor_ok);

/* Update the present-sensor mask after a runtime re-init (NFR-15). A gyro
 * that appears after boot starts its bias attempt. */
void calibration_set_sensor_ok(uint16_t sensor_ok);

/* Feed one raw body-frame sample per sensor task. */
void calibration_feed_gyro(const vec3_t *raw_body);
void calibration_feed_accel(const vec3_t *raw_body);
void calibration_feed_mag(const vec3_t *raw_body);
void calibration_feed_baro(float pressure_pa);

/* Apply the stored parameters (no-op for a sensor whose calibration is not
 * valid). */
void calibration_apply_gyro(vec3_t *body);
void calibration_apply_accel(vec3_t *body);
void calibration_apply_mag(vec3_t *body);

/* Commands from §22.4. */
void calibration_start_gyro(void);
void calibration_start_accel(void);
void calibration_toggle_mag(void);
void calibration_start_baro(void);
void calibration_request_save(void);
/* 1 once per pending 'S'; the main loop then calls calibration_save(). */
uint8_t calibration_take_save_request(void);
/* elapsed_ms (may be NULL) returns the flash stall duration. */
int32_t calibration_save(uint32_t *elapsed_ms);
void calibration_reset_defaults(void);

uint16_t calibration_status(void); /* STATUS_*_CAL / STATUS_BARO_REF */
uint8_t calibration_state(void);   /* calib_state byte */
uint8_t calibration_active(void);  /* 1 while a capture or accel sequence runs */
float calibration_p0_pa(void);

#ifdef __cplusplus
}
#endif

#endif /* __CALIBRATION_H */
