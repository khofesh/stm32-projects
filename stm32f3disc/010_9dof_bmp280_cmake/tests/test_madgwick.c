#include "test.h"

#include <math.h>

#include "madgwick.h"

#define IMU_HZ 100.0f
#define IMU_DT (1.0f / IMU_HZ)

#define DEG_TO_RAD 0.0174532925f
#define RAD_TO_DEG 57.2957795f

static float quat_norm(const float q[4])
{
  return sqrtf((q[0] * q[0]) + (q[1] * q[1]) + (q[2] * q[2]) + (q[3] * q[3]));
}

static float quat_yaw_deg(const float q[4])
{
  float r10 = 2.0f * ((q[1] * q[2]) + (q[0] * q[3]));
  float r00 = 1.0f - (2.0f * ((q[2] * q[2]) + (q[3] * q[3])));

  return atan2f(r10, r00) * RAD_TO_DEG;
}

static float quat_pitch_deg(const float q[4])
{
  float r20 = 2.0f * ((q[1] * q[3]) - (q[0] * q[2]));

  return asinf(-r20) * RAD_TO_DEG;
}

static float quat_roll_deg(const float q[4])
{
  float r21 = 2.0f * ((q[2] * q[3]) + (q[0] * q[1]));
  float r22 = 1.0f - (2.0f * ((q[1] * q[1]) + (q[2] * q[2])));

  return atan2f(r21, r22) * RAD_TO_DEG;
}

/* Earth-frame direction of a body-frame vector: R(q) * v. */
static void quat_rotate(const float q[4], const vec3_t *in, vec3_t *out)
{
  float w = q[0];
  float x = q[1];
  float y = q[2];
  float z = q[3];

  out->x = ((1.0f - (2.0f * ((y * y) + (z * z)))) * in->x) +
           (2.0f * ((x * y) - (w * z)) * in->y) +
           (2.0f * ((x * z) + (w * y)) * in->z);
  out->y = (2.0f * ((x * y) + (w * z)) * in->x) +
           ((1.0f - (2.0f * ((x * x) + (z * z)))) * in->y) +
           (2.0f * ((y * z) - (w * x)) * in->z);
  out->z = (2.0f * ((x * z) - (w * y)) * in->x) +
           (2.0f * ((y * z) + (w * x)) * in->y) +
           ((1.0f - (2.0f * ((x * x) + (y * y)))) * in->z);
}

/* Body-frame direction of an earth-frame vector: R(q)^T * v. */
static void quat_rotate_inv(const float q[4], const vec3_t *in, vec3_t *out)
{
  float conj[4] = {q[0], -q[1], -q[2], -q[3]};

  quat_rotate(conj, in, out);
}

static void quat_from_euler(float roll, float pitch, float yaw, float q[4])
{
  float cr = cosf(roll * 0.5f);
  float sr = sinf(roll * 0.5f);
  float cp = cosf(pitch * 0.5f);
  float sp = sinf(pitch * 0.5f);
  float cy = cosf(yaw * 0.5f);
  float sy = sinf(yaw * 0.5f);

  q[0] = (cr * cp * cy) + (sr * sp * sy);
  q[1] = (sr * cp * cy) - (cr * sp * sy);
  q[2] = (cr * sp * cy) + (sr * cp * sy);
  q[3] = (cr * cp * sy) - (sr * sp * cy);
}

static void test_norm(void)
{
  const vec3_t g = {0.1f, -0.2f, 0.3f};
  const vec3_t a = {0.02f, 0.01f, -9.8f};
  float q[4];

  madgwick_init(0.1f, IMU_HZ);
  for (uint32_t i = 0u; i < 1000000u; i++)
  {
    madgwick_update(&g, &a, NULL);
  }
  madgwick_get_quat(q);
  CHECK_NEAR_F(quat_norm(q), 1.0f, 1e-5f);
}

static void test_gyro_integration(void)
{
  const vec3_t a = {0.0f, 0.0f, -9.80665f};
  float q[4];

  /* +1 rad/s about body Z for 1 s == +1 rad yaw (NED, right-hand rule). */
  {
    const vec3_t g = {0.0f, 0.0f, 1.0f};

    madgwick_init(0.0f, IMU_HZ);
    for (uint32_t i = 0u; i < (uint32_t)IMU_HZ; i++)
    {
      madgwick_update(&g, &a, NULL);
    }
    madgwick_get_quat(q);
    CHECK_NEAR_F(quat_norm(q), 1.0f, 1e-5f);
    CHECK_NEAR_F(q[0], cosf(0.5f), 1e-4f);
    CHECK_NEAR_F(q[1], 0.0f, 1e-4f);
    CHECK_NEAR_F(q[2], 0.0f, 1e-4f);
    CHECK_NEAR_F(q[3], sinf(0.5f), 1e-4f);
  }

  /* 0.5 rad/s about body X for 2 s. */
  {
    const vec3_t g = {0.5f, 0.0f, 0.0f};

    madgwick_init(0.0f, IMU_HZ);
    for (uint32_t i = 0u; i < (uint32_t)(2.0f * IMU_HZ); i++)
    {
      madgwick_update(&g, &a, NULL);
    }
    madgwick_get_quat(q);
    CHECK_NEAR_F(q[0], cosf(0.5f), 1e-4f);
    CHECK_NEAR_F(q[1], sinf(0.5f), 1e-4f);
    CHECK_NEAR_F(q[2], 0.0f, 1e-4f);
    CHECK_NEAR_F(q[3], 0.0f, 1e-4f);
  }
}

static void test_accel_level(void)
{
  const vec3_t g = {0.0f, 0.0f, 0.0f};
  const vec3_t a = {0.0f, 0.0f, -9.80665f};
  float q[4];

  madgwick_init(0.1f, IMU_HZ);
  for (uint32_t i = 0u; i < 1000u; i++)
  {
    madgwick_update(&g, &a, NULL);
  }
  madgwick_get_quat(q);
  CHECK_NEAR_F(q[0], 1.0f, 1e-6f);
  CHECK_NEAR_F(q[1], 0.0f, 1e-6f);
  CHECK_NEAR_F(q[2], 0.0f, 1e-6f);
  CHECK_NEAR_F(q[3], 0.0f, 1e-6f);
}

static void test_accel_tilt(void)
{
  const vec3_t g = {0.0f, 0.0f, 0.0f};
  const float tilt = 20.0f * DEG_TO_RAD;
  const vec3_t a = {sinf(tilt), 0.0f, -cosf(tilt)};
  vec3_t earth;
  float q[4];

  madgwick_init(0.1f, IMU_HZ);
  for (uint32_t i = 0u; i < 2000u; i++)
  {
    madgwick_update(&g, &a, NULL);
  }
  madgwick_get_quat(q);
  quat_rotate(q, &a, &earth);
  CHECK_NEAR_F(earth.x, 0.0f, 0.01f);
  CHECK_NEAR_F(earth.y, 0.0f, 0.01f);
  CHECK_NEAR_F(earth.z, -1.0f, 0.01f);
  CHECK_NEAR_F(quat_roll_deg(q), 0.0f, 0.1f);
  CHECK_NEAR_F(quat_pitch_deg(q), 20.0f, 0.1f);
  CHECK_NEAR_F(quat_yaw_deg(q), 0.0f, 0.1f);
}

static void test_mag_heading(void)
{
  const vec3_t g = {0.0f, 0.0f, 0.0f};
  const vec3_t a = {0.0f, 0.0f, -9.80665f};
  const float bh = 20.0f;
  const float bd = 40.0f;
  /* Body yawed +90 deg: body X points east, so earth north is body -Y. */
  const vec3_t m = {0.0f, -bh, bd};
  float q[4];

  madgwick_init(0.1f, IMU_HZ);
  for (uint32_t i = 0u; i < 2000u; i++)
  {
    madgwick_update(&g, &a, &m);
  }
  madgwick_get_quat(q);
  CHECK_NEAR_F(quat_norm(q), 1.0f, 1e-5f);
  CHECK_NEAR_F(quat_yaw_deg(q), 90.0f, 0.2f);
  CHECK_NEAR_F(quat_roll_deg(q), 0.0f, 0.2f);
  CHECK_NEAR_F(quat_pitch_deg(q), 0.0f, 0.2f);
}

static void test_marg_tilted(void)
{
  const vec3_t g = {0.0f, 0.0f, 0.0f};
  const vec3_t a_earth = {0.0f, 0.0f, -1.0f};
  const vec3_t m_earth = {20.0f, 0.0f, 40.0f};
  vec3_t a;
  vec3_t m;
  float want[4];
  float q[4];
  float dot;

  quat_from_euler(15.0f * DEG_TO_RAD, -10.0f * DEG_TO_RAD, 30.0f * DEG_TO_RAD, want);
  quat_rotate_inv(want, &a_earth, &a);
  quat_rotate_inv(want, &m_earth, &m);

  madgwick_init(0.1f, IMU_HZ);
  for (uint32_t i = 0u; i < 4000u; i++)
  {
    madgwick_update(&g, &a, &m);
  }
  madgwick_get_quat(q);
  dot = (q[0] * want[0]) + (q[1] * want[1]) + (q[2] * want[2]) + (q[3] * want[3]);
  CHECK_NEAR_F(fabsf(dot), 1.0f, 0.01f);
  CHECK_NEAR_F(quat_norm(q), 1.0f, 1e-5f);
}

void test_madgwick(void)
{
  test_norm();
  test_gyro_integration();
  test_accel_level();
  test_accel_tilt();
  test_mag_heading();
  test_marg_tilted();
}
