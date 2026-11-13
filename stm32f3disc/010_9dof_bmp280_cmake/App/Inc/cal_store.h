#ifndef __CAL_STORE_H
#define __CAL_STORE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "calib_math.h"

/* Persisted calibration, SPEC §22.3: last 2 KB flash page.
 *
 *   offset  size  field
 *   0       4     magic = 0x31434C41 ("ALC1" little-endian)
 *   4       2     version = 1
 *   6       2     parameter payload length = 68
 *   8       68    17 floats: gyro bias, accel bias, accel scale, mag bias,
 *                 mag scale, baro p0, flags (all little-endian)
 *   76      2     CRC16-CCITT-FALSE over bytes 0..75
 */
#define CAL_STORE_MAGIC 0x31434C41u
#define CAL_STORE_VERSION 1u
#define CAL_STORE_PARAM_LEN 68u
#define CAL_STORE_LEN 78u

/* Encode into buf; returns the number of bytes written, or 0 if cap is too
 * small or an argument is invalid. */
uint16_t cal_store_encode(const calib_params_t *p, uint8_t *buf, uint16_t cap);

/* Verify magic, version, length and CRC, then fill p. */
int32_t cal_store_decode(const uint8_t *buf, uint16_t len, calib_params_t *p);

/* Load from flash; on any error p is set to defaults (SPEC §22.3). */
int32_t cal_store_load(calib_params_t *p);

/* Erase and program the calibration page. */
int32_t cal_store_save(const calib_params_t *p);

#ifdef __cplusplus
}
#endif

#endif /* __CAL_STORE_H */
