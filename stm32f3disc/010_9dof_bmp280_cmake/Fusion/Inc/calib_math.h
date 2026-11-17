#ifndef __CALIB_MATH_H
#define __CALIB_MATH_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>

#include "vec3.h"

/* Calibration models, SPEC §22.1:
 *   gyro:  w = raw - b_g
 *   accel: a = S_a * (raw - b_a)
 *   mag:   m = S_m * (raw - b_m)
 *   baro:  h = altitude(p, p0)
 *
 * Parameters are kept in the body frame: the sensor -> body axis map is
 * applied before these corrections. */

#define CAL_FLAG_GYRO (1u << 0)
#define CAL_FLAG_ACCEL (1u << 1)
#define CAL_FLAG_MAG (1u << 2)
#define CAL_FLAG_BARO (1u << 3)
#define CAL_FLAG_ALL (CAL_FLAG_GYRO | CAL_FLAG_ACCEL | CAL_FLAG_MAG | CAL_FLAG_BARO)

#define CALIB_STD_GRAVITY 9.80665f

typedef struct
{
  vec3_t gyro_bias;   /* rad/s */
  vec3_t accel_bias;  /* m/s^2 */
  vec3_t accel_scale; /* dimensionless */
  vec3_t mag_bias;    /* uT */
  vec3_t mag_scale;   /* dimensionless */
  float baro_p0_pa;   /* Pa */
  uint16_t flags;     /* CAL_FLAG_* */
} calib_params_t;

/* Streaming statistics: Welford mean/variance plus min/max. */
typedef struct
{
  uint32_t n;
  vec3_t mean;
  vec3_t m2;
  vec3_t min;
  vec3_t max;
} calib_accum_t;

/* Min/max only (mag capture). */
typedef struct
{
  uint32_t n;
  vec3_t min;
  vec3_t max;
} calib_range_t;

typedef struct
{
  uint32_t n;
  float mean;
  float m2;
  float min;
  float max;
} calib_scalar_accum_t;

void calib_params_defaults(calib_params_t *p);

void calib_apply_gyro(const calib_params_t *p, vec3_t *v);
void calib_apply_accel(const calib_params_t *p, vec3_t *v);
void calib_apply_mag(const calib_params_t *p, vec3_t *v);

void calib_accum_reset(calib_accum_t *a);
void calib_accum_add(calib_accum_t *a, const vec3_t *v);
void calib_range_reset(calib_range_t *r);
void calib_range_add(calib_range_t *r, const vec3_t *v);
void calib_scalar_accum_reset(calib_scalar_accum_t *a);
void calib_scalar_accum_add(calib_scalar_accum_t *a, float v);

int32_t calib_accum_mean(const calib_accum_t *a, vec3_t *mean);
int32_t calib_accum_variance(const calib_accum_t *a, vec3_t *variance);
int32_t calib_scalar_accum_stats(const calib_scalar_accum_t *a, float *mean, float *variance);

/* -ERANGE if any axis standard deviation exceeds max_std. */
int32_t calib_accum_still(const calib_accum_t *a, float max_std);

/* Average and reject if any axis standard error of the mean (std / sqrt(n))
 * exceeds max_sem_rad_s. */
int32_t calib_gyro_bias(const calib_accum_t *a, float max_sem_rad_s, vec3_t *bias);

/* Dominant body axis and sign of the mean, rejecting means more than 10 deg
 * away from an axis. axis is 0..2 (x, y, z), sign is +1 or -1. */
int32_t calib_accel_orientation(const calib_accum_t *a, uint8_t *axis, int8_t *sign);

/* Solve b_a and S_a from the means in the +axis and -axis orientations. */
int32_t calib_accel_solve(const vec3_t pos[3], const vec3_t neg[3], vec3_t *bias, vec3_t *scale);

/* Hard-iron centre and diagonal soft-iron scale from the captured min/max.
 * Accepts when every span exceeds min_span_ut and max_span/min_span is below
 * max_span_ratio. */
int32_t calib_mag_solve(const calib_range_t *r, float min_span_ut, float max_span_ratio, vec3_t *bias, vec3_t *scale);

/* Mean reference pressure, rejected when the standard deviation exceeds
 * max_std_pa. */
int32_t calib_baro_p0(const calib_scalar_accum_t *a, float max_std_pa, float *p0_pa);

#ifdef __cplusplus
}
#endif

#endif /* __CALIB_MATH_H */
