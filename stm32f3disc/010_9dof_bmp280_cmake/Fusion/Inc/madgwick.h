#ifndef __MADGWICK_H
#define __MADGWICK_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "vec3.h"

/* Madgwick AHRS, SPEC §21.4. Body frame FRD, earth frame NED; the quaternion
 * rotates body -> earth. Gyro in rad/s, accel in m/s^2, mag in uT. */
void madgwick_init(float beta, float sample_hz);

/* m == NULL selects IMU-only (no magnetometer correction). Accel and gyro
 * must be valid. */
void madgwick_update(const vec3_t *g, const vec3_t *a, const vec3_t *m);

/* q = w, x, y, z. */
void madgwick_get_quat(float q[4]);

#ifdef __cplusplus
}
#endif

#endif /* __MADGWICK_H */
