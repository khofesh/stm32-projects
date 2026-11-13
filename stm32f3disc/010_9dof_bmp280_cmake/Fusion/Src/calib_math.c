#include "calib_math.h"

#include <errno.h>
#include <stddef.h>
#include <string.h>

/* cos^2(10 deg): the accel mean must be within 10 deg of its dominant axis. */
#define ACCEL_AXIS_COS2 0.96984631f

static float fabs_f(float v)
{
  return (v < 0.0f) ? -v : v;
}

static float fmax3(float a, float b, float c)
{
  float m = (a > b) ? a : b;

  return (m > c) ? m : c;
}

static float fmin3(float a, float b, float c)
{
  float m = (a < b) ? a : b;

  return (m < c) ? m : c;
}

static void accum_add_axis(float *mean, float *m2, uint32_t n, float v)
{
  float delta = v - *mean;
  float inv = 1.0f / (float)n;

  *mean += delta * inv;
  *m2 += delta * (v - *mean);
}

void calib_params_defaults(calib_params_t *p)
{
  if (p == NULL)
  {
    return;
  }
  memset(p, 0, sizeof(*p));
  p->accel_scale.x = 1.0f;
  p->accel_scale.y = 1.0f;
  p->accel_scale.z = 1.0f;
  p->mag_scale.x = 1.0f;
  p->mag_scale.y = 1.0f;
  p->mag_scale.z = 1.0f;
}

void calib_apply_gyro(const calib_params_t *p, vec3_t *v)
{
  if ((p == NULL) || (v == NULL) || ((p->flags & CAL_FLAG_GYRO) == 0u))
  {
    return;
  }
  v->x -= p->gyro_bias.x;
  v->y -= p->gyro_bias.y;
  v->z -= p->gyro_bias.z;
}

void calib_apply_accel(const calib_params_t *p, vec3_t *v)
{
  if ((p == NULL) || (v == NULL) || ((p->flags & CAL_FLAG_ACCEL) == 0u))
  {
    return;
  }
  v->x = p->accel_scale.x * (v->x - p->accel_bias.x);
  v->y = p->accel_scale.y * (v->y - p->accel_bias.y);
  v->z = p->accel_scale.z * (v->z - p->accel_bias.z);
}

void calib_apply_mag(const calib_params_t *p, vec3_t *v)
{
  if ((p == NULL) || (v == NULL) || ((p->flags & CAL_FLAG_MAG) == 0u))
  {
    return;
  }
  v->x = p->mag_scale.x * (v->x - p->mag_bias.x);
  v->y = p->mag_scale.y * (v->y - p->mag_bias.y);
  v->z = p->mag_scale.z * (v->z - p->mag_bias.z);
}

void calib_accum_reset(calib_accum_t *a)
{
  if (a == NULL)
  {
    return;
  }
  memset(a, 0, sizeof(*a));
}

void calib_accum_add(calib_accum_t *a, const vec3_t *v)
{
  if ((a == NULL) || (v == NULL))
  {
    return;
  }

  if (a->n == 0u)
  {
    a->n = 1u;
    a->mean = *v;
    a->m2.x = 0.0f;
    a->m2.y = 0.0f;
    a->m2.z = 0.0f;
    a->min = *v;
    a->max = *v;
    return;
  }

  a->n++;
  accum_add_axis(&a->mean.x, &a->m2.x, a->n, v->x);
  accum_add_axis(&a->mean.y, &a->m2.y, a->n, v->y);
  accum_add_axis(&a->mean.z, &a->m2.z, a->n, v->z);

  if (v->x < a->min.x)
  {
    a->min.x = v->x;
  }
  if (v->y < a->min.y)
  {
    a->min.y = v->y;
  }
  if (v->z < a->min.z)
  {
    a->min.z = v->z;
  }
  if (v->x > a->max.x)
  {
    a->max.x = v->x;
  }
  if (v->y > a->max.y)
  {
    a->max.y = v->y;
  }
  if (v->z > a->max.z)
  {
    a->max.z = v->z;
  }
}

void calib_range_reset(calib_range_t *r)
{
  if (r == NULL)
  {
    return;
  }
  memset(r, 0, sizeof(*r));
}

void calib_range_add(calib_range_t *r, const vec3_t *v)
{
  if ((r == NULL) || (v == NULL))
  {
    return;
  }
  if (r->n == 0u)
  {
    r->min = *v;
    r->max = *v;
  }
  else
  {
    r->min.x = (v->x < r->min.x) ? v->x : r->min.x;
    r->min.y = (v->y < r->min.y) ? v->y : r->min.y;
    r->min.z = (v->z < r->min.z) ? v->z : r->min.z;
    r->max.x = (v->x > r->max.x) ? v->x : r->max.x;
    r->max.y = (v->y > r->max.y) ? v->y : r->max.y;
    r->max.z = (v->z > r->max.z) ? v->z : r->max.z;
  }
  if (r->n < UINT32_MAX)
  {
    r->n++;
  }
}

void calib_scalar_accum_reset(calib_scalar_accum_t *a)
{
  if (a == NULL)
  {
    return;
  }
  memset(a, 0, sizeof(*a));
}

void calib_scalar_accum_add(calib_scalar_accum_t *a, float v)
{
  if (a == NULL)
  {
    return;
  }

  if (a->n == 0u)
  {
    a->n = 1u;
    a->mean = v;
    a->m2 = 0.0f;
    a->min = v;
    a->max = v;
    return;
  }

  a->n++;
  accum_add_axis(&a->mean, &a->m2, a->n, v);
  if (v < a->min)
  {
    a->min = v;
  }
  if (v > a->max)
  {
    a->max = v;
  }
}

int32_t calib_accum_mean(const calib_accum_t *a, vec3_t *mean)
{
  if ((a == NULL) || (mean == NULL))
  {
    return -EINVAL;
  }
  if (a->n == 0u)
  {
    return -ENODATA;
  }
  *mean = a->mean;
  return 0;
}

int32_t calib_accum_variance(const calib_accum_t *a, vec3_t *variance)
{
  if ((a == NULL) || (variance == NULL))
  {
    return -EINVAL;
  }
  if (a->n == 0u)
  {
    return -ENODATA;
  }
  variance->x = a->m2.x / (float)a->n;
  variance->y = a->m2.y / (float)a->n;
  variance->z = a->m2.z / (float)a->n;
  return 0;
}

int32_t calib_scalar_accum_stats(const calib_scalar_accum_t *a, float *mean, float *variance)
{
  if ((a == NULL) || (mean == NULL) || (variance == NULL))
  {
    return -EINVAL;
  }
  if (a->n == 0u)
  {
    return -ENODATA;
  }
  *mean = a->mean;
  *variance = a->m2 / (float)a->n;
  return 0;
}

int32_t calib_accum_still(const calib_accum_t *a, float max_std)
{
  vec3_t var;
  float limit;
  int32_t st;

  if (!(max_std > 0.0f))
  {
    return -EINVAL;
  }
  st = calib_accum_variance(a, &var);
  if (st != 0)
  {
    return st;
  }
  limit = max_std * max_std;
  if ((var.x > limit) || (var.y > limit) || (var.z > limit))
  {
    return -ERANGE;
  }
  return 0;
}

int32_t calib_gyro_bias(const calib_accum_t *a, float max_sem_rad_s, vec3_t *bias)
{
  vec3_t var;
  float limit;
  int32_t st;

  if (bias == NULL)
  {
    return -EINVAL;
  }
  if (!(max_sem_rad_s > 0.0f))
  {
    return -EINVAL;
  }
  st = calib_accum_variance(a, &var);
  if (st != 0)
  {
    return st;
  }

  /* var / n is the squared standard error of the mean. */
  limit = max_sem_rad_s * max_sem_rad_s * (float)a->n;
  if ((var.x > limit) || (var.y > limit) || (var.z > limit))
  {
    return -ERANGE;
  }

  *bias = a->mean;
  return 0;
}

int32_t calib_accel_orientation(const calib_accum_t *a, uint8_t *axis, int8_t *sign)
{
  float ax;
  float ay;
  float az;
  float n2;
  float dom2;
  uint8_t idx;

  if ((a == NULL) || (axis == NULL) || (sign == NULL))
  {
    return -EINVAL;
  }
  if (a->n == 0u)
  {
    return -ENODATA;
  }

  ax = fabs_f(a->mean.x);
  ay = fabs_f(a->mean.y);
  az = fabs_f(a->mean.z);
  n2 = (a->mean.x * a->mean.x) + (a->mean.y * a->mean.y) + (a->mean.z * a->mean.z);

  if (ax >= ay)
  {
    idx = (ax >= az) ? 0u : 2u;
  }
  else
  {
    idx = (ay >= az) ? 1u : 2u;
  }

  dom2 = (idx == 0u) ? (a->mean.x * a->mean.x)
                     : ((idx == 1u) ? (a->mean.y * a->mean.y) : (a->mean.z * a->mean.z));

  /* Rejects free-fall / all-zero captures as well as tilted ones. */
  if (!(n2 > 1.0f) || (dom2 < (n2 * ACCEL_AXIS_COS2)))
  {
    return -ERANGE;
  }

  *axis = idx;
  if (idx == 0u)
  {
    *sign = (a->mean.x < 0.0f) ? -1 : 1;
  }
  else if (idx == 1u)
  {
    *sign = (a->mean.y < 0.0f) ? -1 : 1;
  }
  else
  {
    *sign = (a->mean.z < 0.0f) ? -1 : 1;
  }
  return 0;
}

int32_t calib_accel_solve(const vec3_t pos[3], const vec3_t neg[3], vec3_t *bias, vec3_t *scale)
{
  vec3_t b;
  vec3_t s;
  float p[3];
  float n[3];
  float denom;

  if ((pos == NULL) || (neg == NULL) || (bias == NULL) || (scale == NULL))
  {
    return -EINVAL;
  }

  p[0] = pos[0].x;
  p[1] = pos[1].y;
  p[2] = pos[2].z;
  n[0] = neg[0].x;
  n[1] = neg[1].y;
  n[2] = neg[2].z;

  denom = p[0] - n[0];
  if (!(denom > 0.001f))
  {
    return -ERANGE;
  }
  b.x = (p[0] + n[0]) * 0.5f;
  s.x = (2.0f * CALIB_STD_GRAVITY) / denom;

  denom = p[1] - n[1];
  if (!(denom > 0.001f))
  {
    return -ERANGE;
  }
  b.y = (p[1] + n[1]) * 0.5f;
  s.y = (2.0f * CALIB_STD_GRAVITY) / denom;

  denom = p[2] - n[2];
  if (!(denom > 0.001f))
  {
    return -ERANGE;
  }
  b.z = (p[2] + n[2]) * 0.5f;
  s.z = (2.0f * CALIB_STD_GRAVITY) / denom;

  *bias = b;
  *scale = s;
  return 0;
}

int32_t calib_mag_solve(const calib_range_t *r, float min_span_ut, float max_span_ratio,
                        vec3_t *bias, vec3_t *scale)
{
  float sx;
  float sy;
  float sz;
  float hx;
  float hy;
  float hz;
  float target;
  float ratio;

  if ((r == NULL) || (bias == NULL) || (scale == NULL))
  {
    return -EINVAL;
  }
  if (r->n < 2u)
  {
    return -ENODATA;
  }
  if (!(min_span_ut > 0.0f) || !(max_span_ratio > 1.0f))
  {
    return -EINVAL;
  }

  sx = r->max.x - r->min.x;
  sy = r->max.y - r->min.y;
  sz = r->max.z - r->min.z;
  if ((sx <= min_span_ut) || (sy <= min_span_ut) || (sz <= min_span_ut))
  {
    return -ERANGE;
  }

  hx = sx * 0.5f;
  hy = sy * 0.5f;
  hz = sz * 0.5f;
  target = (hx + hy + hz) / 3.0f;
  ratio = fmax3(hx, hy, hz) / fmin3(hx, hy, hz);
  if (!(ratio < max_span_ratio))
  {
    return -ERANGE;
  }

  bias->x = (r->min.x + r->max.x) * 0.5f;
  bias->y = (r->min.y + r->max.y) * 0.5f;
  bias->z = (r->min.z + r->max.z) * 0.5f;

  scale->x = target / hx;
  scale->y = target / hy;
  scale->z = target / hz;
  return 0;
}

int32_t calib_baro_p0(const calib_scalar_accum_t *a, float max_std_pa, float *p0_pa)
{
  float mean;
  float var;
  int32_t st;

  if (p0_pa == NULL)
  {
    return -EINVAL;
  }
  if (!(max_std_pa > 0.0f))
  {
    return -EINVAL;
  }
  st = calib_scalar_accum_stats(a, &mean, &var);
  if (st != 0)
  {
    return st;
  }
  if (var > (max_std_pa * max_std_pa))
  {
    return -ERANGE;
  }

  *p0_pa = mean;
  return 0;
}
