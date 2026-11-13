#include "calibration.h"

#include <stddef.h>
#include <string.h>

#include "cal_store.h"
#include "telemetry.h"

static calib_params_t s_params;
static uint16_t s_sensor_ok;

static uint8_t s_gyro_active;
static uint8_t s_accel_active;
static uint8_t s_mag_active;
static uint8_t s_baro_active;
static uint8_t s_rejected;
static uint8_t s_save_pending;

static calib_accum_t s_gyro_acc;
static calib_accum_t s_accel_acc;
static calib_range_t s_mag_rng;
static calib_scalar_accum_t s_baro_acc;

static vec3_t s_accel_pos[3];
static vec3_t s_accel_neg[3];
static uint8_t s_accel_seen; /* bit (axis * 2 + (sign < 0)) */
static uint8_t s_accel_count;

static void captures_reset(void)
{
  s_gyro_active = 0u;
  s_accel_active = 0u;
  s_mag_active = 0u;
  s_baro_active = 0u;
  calib_accum_reset(&s_gyro_acc);
  calib_accum_reset(&s_accel_acc);
  calib_range_reset(&s_mag_rng);
  calib_scalar_accum_reset(&s_baro_acc);
  s_accel_seen = 0u;
  s_accel_count = 0u;
}

void calibration_init(uint16_t sensor_ok)
{
  s_sensor_ok = sensor_ok;
  s_rejected = 0u;
  s_save_pending = 0u;
  captures_reset();
  (void)cal_store_load(&s_params);

  if ((s_sensor_ok & STATUS_GYRO_OK) != 0u)
  {
    calibration_start_gyro();
  }
}

void calibration_start_gyro(void)
{
  s_rejected = 0u;
  if ((s_sensor_ok & STATUS_GYRO_OK) == 0u)
  {
    s_rejected = 1u;
    return;
  }
  calib_accum_reset(&s_gyro_acc);
  s_gyro_active = 1u;
}

void calibration_start_accel(void)
{
  if (s_accel_active != 0u)
  {
    return;
  }
  s_rejected = 0u;
  if ((s_sensor_ok & STATUS_ACCEL_OK) == 0u)
  {
    s_rejected = 1u;
    return;
  }
  calib_accum_reset(&s_accel_acc);
  s_accel_active = 1u;
}

void calibration_toggle_mag(void)
{
  vec3_t bias;
  vec3_t scale;

  s_rejected = 0u;
  if ((s_sensor_ok & STATUS_MAG_OK) == 0u)
  {
    s_rejected = 1u;
    return;
  }

  if (s_mag_active == 0u)
  {
    calib_range_reset(&s_mag_rng);
    s_mag_active = 1u;
    return;
  }

  s_mag_active = 0u;
  if ((s_mag_rng.n < CALIB_MAG_MIN_SAMPLES) ||
      (calib_mag_solve(&s_mag_rng, CALIB_MAG_MIN_SPAN_UT, CALIB_MAG_MAX_SPAN_RATIO, &bias,
                       &scale) != 0))
  {
    s_rejected = 1u;
    return;
  }

  s_params.mag_bias = bias;
  s_params.mag_scale = scale;
  s_params.flags |= CAL_FLAG_MAG;
}

void calibration_start_baro(void)
{
  s_rejected = 0u;
  if ((s_sensor_ok & STATUS_BARO_OK) == 0u)
  {
    s_rejected = 1u;
    return;
  }
  calib_scalar_accum_reset(&s_baro_acc);
  s_baro_active = 1u;
}

void calibration_feed_gyro(const vec3_t *raw_body)
{
  vec3_t bias;

  if ((s_gyro_active == 0u) || (raw_body == NULL))
  {
    return;
  }

  calib_accum_add(&s_gyro_acc, raw_body);
  if (s_gyro_acc.n < CALIB_GYRO_SAMPLES)
  {
    return;
  }

  s_gyro_active = 0u;
  if (calib_gyro_bias(&s_gyro_acc, CALIB_GYRO_MAX_SEM_RAD_S, &bias) != 0)
  {
    s_rejected = 1u;
    return;
  }
  s_params.gyro_bias = bias;
  s_params.flags |= CAL_FLAG_GYRO;
}

/* Accept one finished 100-sample pose; 0 on success. */
static int32_t accel_take_pose(void)
{
  uint8_t axis = 0u;
  int8_t sign = 1;
  uint8_t bit;
  vec3_t mean;

  if ((calib_accum_still(&s_accel_acc, CALIB_ACCEL_MAX_STD_MS2) != 0) ||
      (calib_accel_orientation(&s_accel_acc, &axis, &sign) != 0) ||
      (calib_accum_mean(&s_accel_acc, &mean) != 0))
  {
    return -1;
  }

  bit = (uint8_t)((axis * 2u) + ((sign > 0) ? 0u : 1u));
  if ((s_accel_seen & (uint8_t)(1u << bit)) != 0u)
  {
    return -1;
  }

  s_accel_seen |= (uint8_t)(1u << bit);
  s_accel_count++;
  if (sign > 0)
  {
    s_accel_pos[axis] = mean;
  }
  else
  {
    s_accel_neg[axis] = mean;
  }
  return 0;
}

void calibration_feed_accel(const vec3_t *raw_body)
{
  vec3_t bias;
  vec3_t scale;
  int32_t st;

  if ((s_accel_active == 0u) || (raw_body == NULL))
  {
    return;
  }

  calib_accum_add(&s_accel_acc, raw_body);
  if (s_accel_acc.n < CALIB_ACCEL_SAMPLES)
  {
    return;
  }

  s_accel_active = 0u;
  st = accel_take_pose();
  calib_accum_reset(&s_accel_acc);
  if (st != 0)
  {
    s_rejected = 1u;
    return;
  }
  if (s_accel_count < 6u)
  {
    return;
  }

  if (calib_accel_solve(s_accel_pos, s_accel_neg, &bias, &scale) == 0)
  {
    s_params.accel_bias = bias;
    s_params.accel_scale = scale;
    s_params.flags |= CAL_FLAG_ACCEL;
  }
  else
  {
    s_rejected = 1u;
  }
  /* Starting a new sequence is the only way to retry the six poses. */
  s_accel_seen = 0u;
  s_accel_count = 0u;
}

void calibration_feed_mag(const vec3_t *raw_body)
{
  if ((s_mag_active == 0u) || (raw_body == NULL))
  {
    return;
  }
  calib_range_add(&s_mag_rng, raw_body);
}

void calibration_feed_baro(float pressure_pa)
{
  float p0;

  if (s_baro_active == 0u)
  {
    return;
  }

  calib_scalar_accum_add(&s_baro_acc, pressure_pa);
  if (s_baro_acc.n < CALIB_BARO_SAMPLES)
  {
    return;
  }

  s_baro_active = 0u;
  if (calib_baro_p0(&s_baro_acc, CALIB_BARO_MAX_STD_PA, &p0) != 0)
  {
    s_rejected = 1u;
    return;
  }
  s_params.baro_p0_pa = p0;
  s_params.flags |= CAL_FLAG_BARO;
}

void calibration_apply_gyro(vec3_t *body)
{
  calib_apply_gyro(&s_params, body);
}

void calibration_apply_accel(vec3_t *body)
{
  calib_apply_accel(&s_params, body);
}

void calibration_apply_mag(vec3_t *body)
{
  calib_apply_mag(&s_params, body);
}

void calibration_request_save(void)
{
  s_rejected = 0u;
  s_save_pending = 1u;
}

uint8_t calibration_take_save_request(void)
{
  uint8_t r = s_save_pending;

  s_save_pending = 0u;
  return r;
}

int32_t calibration_save(uint32_t *elapsed_ms)
{
  int32_t st = cal_store_save(&s_params, elapsed_ms);

  if (st != 0)
  {
    s_rejected = 1u;
  }
  return st;
}

void calibration_reset_defaults(void)
{
  calib_params_defaults(&s_params);
  captures_reset();
  s_rejected = 0u;
}

uint16_t calibration_status(void)
{
  uint16_t st = 0u;

  if ((s_params.flags & CAL_FLAG_GYRO) != 0u)
  {
    st |= STATUS_GYRO_CAL;
  }
  if ((s_params.flags & CAL_FLAG_ACCEL) != 0u)
  {
    st |= STATUS_ACCEL_CAL;
  }
  if ((s_params.flags & CAL_FLAG_MAG) != 0u)
  {
    st |= STATUS_MAG_CAL;
  }
  if ((s_params.flags & CAL_FLAG_BARO) != 0u)
  {
    st |= STATUS_BARO_REF;
  }
  return st;
}

uint8_t calibration_state(void)
{
  uint8_t st = (uint8_t)((s_accel_count & CALIB_STATE_ACCEL_COUNT_MASK)
                         << CALIB_STATE_ACCEL_COUNT_SHIFT);

  if (s_gyro_active != 0u)
  {
    st |= CALIB_STATE_GYRO_ACTIVE;
  }
  if (s_accel_active != 0u)
  {
    st |= CALIB_STATE_ACCEL_ACTIVE;
  }
  if (s_mag_active != 0u)
  {
    st |= CALIB_STATE_MAG_ACTIVE;
  }
  if (s_baro_active != 0u)
  {
    st |= CALIB_STATE_BARO_ACTIVE;
  }
  if (s_rejected != 0u)
  {
    st |= CALIB_STATE_REJECTED;
  }
  return st;
}

uint8_t calibration_active(void)
{
  if ((s_gyro_active != 0u) || (s_accel_active != 0u) || (s_mag_active != 0u) ||
      (s_baro_active != 0u) || (s_accel_count != 0u))
  {
    return 1u;
  }
  return 0u;
}

float calibration_p0_pa(void)
{
  return s_params.baro_p0_pa;
}
