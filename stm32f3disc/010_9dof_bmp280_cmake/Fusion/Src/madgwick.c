#include "madgwick.h"

#include <math.h>
#include <stddef.h>

#define MADGWICK_DEFAULT_DT 0.01f
#define MADGWICK_MIN_NORM 1e-6f

static float s_beta;
static float s_dt;
static float s_q[4] = {1.0f, 0.0f, 0.0f, 0.0f};

static float inv_norm(float x, float y, float z)
{
  float n2 = (x * x) + (y * y) + (z * z);

  if (n2 < MADGWICK_MIN_NORM)
  {
    return 0.0f;
  }
  return 1.0f / sqrtf(n2);
}

void madgwick_init(float beta, float sample_hz)
{
  s_beta = (beta > 0.0f) ? beta : 0.0f;
  s_dt = (sample_hz > 0.0f) ? (1.0f / sample_hz) : MADGWICK_DEFAULT_DT;

  s_q[0] = 1.0f;
  s_q[1] = 0.0f;
  s_q[2] = 0.0f;
  s_q[3] = 0.0f;
}

void madgwick_set_beta(float beta)
{
  s_beta = (beta > 0.0f) ? beta : 0.0f;
}

void madgwick_update(const vec3_t *g, const vec3_t *a, const vec3_t *m)
{
  madgwick_update_dt(g, a, m, s_dt);
}

void madgwick_update_dt(const vec3_t *g, const vec3_t *a, const vec3_t *m,
                        float dt)
{
  float q0;
  float q1;
  float q2;
  float q3;
  float s0 = 0.0f;
  float s1 = 0.0f;
  float s2 = 0.0f;
  float s3 = 0.0f;
  float sn;
  float gx;
  float gy;
  float gz;
  float beta = s_beta;

  if (g == NULL)
  {
    return;
  }
  if (dt <= 0.0f)
  {
    dt = s_dt;
  }

  q0 = s_q[0];
  q1 = s_q[1];
  q2 = s_q[2];
  q3 = s_q[3];

  /* Accelerometer: predicted gravity-up direction in the body frame is
   * R^T * (0, 0, -1); the accelerometer measures that same direction. */
  if (a != NULL)
  {
    float n = inv_norm(a->x, a->y, a->z);

    if (n > 0.0f)
    {
      float ax = a->x * n;
      float ay = a->y * n;
      float az = a->z * n;
      float f1 = 2.0f * ((q1 * q3) - (q0 * q2)) + ax;
      float f2 = 2.0f * ((q0 * q1) + (q2 * q3)) + ay;
      float f3 = (q0 * q0) - (q1 * q1) - (q2 * q2) + (q3 * q3) + az;

      s0 += (-2.0f * q2 * f1) + (2.0f * q1 * f2) + (2.0f * q0 * f3);
      s1 += (2.0f * q3 * f1) + (2.0f * q0 * f2) - (2.0f * q1 * f3);
      s2 += (-2.0f * q0 * f1) + (2.0f * q3 * f2) - (2.0f * q2 * f3);
      s3 += (2.0f * q1 * f1) + (2.0f * q2 * f2) + (2.0f * q3 * f3);
    }
  }

  /* Magnetometer: rotate the measurement to NED, keep its horizontal
   * magnitude on the north axis, rotate the expected field back to the body
   * frame and add the residual to the same gradient. */
  if (m != NULL)
  {
    float n = inv_norm(m->x, m->y, m->z);

    if (n > 0.0f)
    {
      float mx = m->x * n;
      float my = m->y * n;
      float mz = m->z * n;
      float hx = ((1.0f - (2.0f * ((q2 * q2) + (q3 * q3)))) * mx) +
                 (2.0f * ((q1 * q2) - (q0 * q3)) * my) +
                 (2.0f * ((q1 * q3) + (q0 * q2)) * mz);
      float hy = (2.0f * ((q1 * q2) + (q0 * q3)) * mx) +
                 ((1.0f - (2.0f * ((q1 * q1) + (q3 * q3)))) * my) +
                 (2.0f * ((q2 * q3) - (q0 * q1)) * mz);
      float hz = (2.0f * ((q1 * q3) - (q0 * q2)) * mx) +
                 (2.0f * ((q0 * q1) + (q2 * q3)) * my) +
                 ((1.0f - (2.0f * ((q1 * q1) + (q2 * q2)))) * mz);
      float bx = sqrtf((hx * hx) + (hy * hy));
      float bz = hz;
      float w1 = ((1.0f - (2.0f * ((q2 * q2) + (q3 * q3)))) * bx) +
                 (2.0f * ((q1 * q3) - (q0 * q2)) * bz);
      float w2 = (2.0f * ((q1 * q2) - (q0 * q3)) * bx) +
                 (2.0f * ((q0 * q1) + (q2 * q3)) * bz);
      float w3 = (2.0f * ((q1 * q3) + (q0 * q2)) * bx) +
                 ((1.0f - (2.0f * ((q1 * q1) + (q2 * q2)))) * bz);
      float f1 = w1 - mx;
      float f2 = w2 - my;
      float f3 = w3 - mz;

      s0 += (-2.0f * bz * q2 * f1) + (2.0f * ((bz * q1) - (bx * q3)) * f2) +
            (2.0f * bx * q2 * f3);
      s1 += (2.0f * bz * q3 * f1) + (2.0f * ((bx * q2) + (bz * q0)) * f2) +
            (2.0f * ((bx * q3) - (2.0f * bz * q1)) * f3);
      s2 += ((-4.0f * bx * q2) - (2.0f * bz * q0)) * f1 +
            (2.0f * ((bx * q1) + (bz * q3)) * f2) +
            (2.0f * ((bx * q0) - (2.0f * bz * q2)) * f3);
      s3 += ((2.0f * bz * q1) - (4.0f * bx * q3)) * f1 +
            (2.0f * ((bz * q2) - (bx * q0)) * f2) + (2.0f * bx * q1 * f3);
    }
  }

  sn = sqrtf((s0 * s0) + (s1 * s1) + (s2 * s2) + (s3 * s3));
  if (sn > 0.0f)
  {
    float inv = 1.0f / sn;

    s0 *= inv;
    s1 *= inv;
    s2 *= inv;
    s3 *= inv;
  }
  else
  {
    s0 = 0.0f;
    s1 = 0.0f;
    s2 = 0.0f;
    s3 = 0.0f;
  }

  gx = g->x;
  gy = g->y;
  gz = g->z;

  /* qdot = 0.5 * q (x) omega - beta * gradient */
  q0 = s_q[0] + (-(s_q[1] * gx) - (s_q[2] * gy) - (s_q[3] * gz) -
                 (2.0f * beta * s0)) * (0.5f * dt);
  q1 = s_q[1] + ((s_q[0] * gx) + (s_q[2] * gz) - (s_q[3] * gy) -
                 (2.0f * beta * s1)) * (0.5f * dt);
  q2 = s_q[2] + ((s_q[0] * gy) - (s_q[1] * gz) + (s_q[3] * gx) -
                 (2.0f * beta * s2)) * (0.5f * dt);
  q3 = s_q[3] + ((s_q[0] * gz) + (s_q[1] * gy) - (s_q[2] * gx) -
                 (2.0f * beta * s3)) * (0.5f * dt);

  sn = sqrtf((q0 * q0) + (q1 * q1) + (q2 * q2) + (q3 * q3));
  if (sn > 0.0f)
  {
    float inv = 1.0f / sn;

    s_q[0] = q0 * inv;
    s_q[1] = q1 * inv;
    s_q[2] = q2 * inv;
    s_q[3] = q3 * inv;
  }
}

void madgwick_get_quat(float q[4])
{
  if (q == NULL)
  {
    return;
  }
  q[0] = s_q[0];
  q[1] = s_q[1];
  q[2] = s_q[2];
  q[3] = s_q[3];
}
