#include "test.h"

#include <errno.h>
#include <string.h>

#include "bsp_flash_stub.h"
#include "cal_store.h"
#include "calib_math.h"
#include "calibration.h"
#include "telemetry.h"

#define CALIB_TEST_GYRO_SAMPLES 200u
#define CALIB_TEST_BARO_SAMPLES 50u

static float test_noise(uint32_t i, float amplitude)
{
  int32_t v = (int32_t)((i * 37u + 11u) % 21u) - 10;

  return ((float)v / 10.0f) * amplitude;
}

static void add_const(calib_accum_t *a, const vec3_t *v, uint32_t n)
{
  for (uint32_t i = 0u; i < n; i++)
  {
    calib_accum_add(a, v);
  }
}

static void test_defaults(void)
{
  calib_params_t p;

  calib_params_defaults(&p);
  CHECK_EQ_U16(p.flags, 0u);
  CHECK_NEAR_F(p.gyro_bias.x, 0.0f, 0.0f);
  CHECK_NEAR_F(p.accel_bias.z, 0.0f, 0.0f);
  CHECK_NEAR_F(p.accel_scale.x, 1.0f, 0.0f);
  CHECK_NEAR_F(p.accel_scale.y, 1.0f, 0.0f);
  CHECK_NEAR_F(p.accel_scale.z, 1.0f, 0.0f);
  CHECK_NEAR_F(p.mag_scale.x, 1.0f, 0.0f);
  CHECK_NEAR_F(p.mag_scale.y, 1.0f, 0.0f);
  CHECK_NEAR_F(p.mag_scale.z, 1.0f, 0.0f);
  CHECK_NEAR_F(p.baro_p0_pa, 0.0f, 0.0f);

  /* Flags clear: the correction is a no-op. */
  vec3_t v = {1.5f, -2.0f, 3.0f};
  calib_apply_gyro(&p, &v);
  calib_apply_accel(&p, &v);
  calib_apply_mag(&p, &v);
  CHECK_NEAR_F(v.x, 1.5f, 0.0f);
  CHECK_NEAR_F(v.y, -2.0f, 0.0f);
  CHECK_NEAR_F(v.z, 3.0f, 0.0f);
}

static void test_gyro_still(void)
{
  const vec3_t bias = {0.0012f, -0.0007f, 0.0034f};
  calib_accum_t acc;
  vec3_t out;
  vec3_t var;

  calib_accum_reset(&acc);
  for (uint32_t i = 0u; i < CALIB_TEST_GYRO_SAMPLES; i++)
  {
    vec3_t s = {bias.x + test_noise(i, 0.0004f), bias.y + test_noise(i + 7u, 0.0004f),
                bias.z + test_noise(i + 13u, 0.0004f)};
    calib_accum_add(&acc, &s);
  }
  CHECK(acc.n == CALIB_TEST_GYRO_SAMPLES);
  CHECK(calib_gyro_bias(&acc, 0.0008726646f, &out) == 0);
  CHECK_NEAR_F(out.x, bias.x, 0.0002f);
  CHECK_NEAR_F(out.y, bias.y, 0.0002f);
  CHECK_NEAR_F(out.z, bias.z, 0.0002f);

  CHECK(calib_accum_variance(&acc, &var) == 0);
  CHECK(var.x < 0.0000008f);

  /* I3G4250D noise at 25 Hz bandwidth (~0.2 deg/s per sample) passes:
   * the limit is on the standard error of the mean. */
  calib_accum_reset(&acc);
  for (uint32_t i = 0u; i < CALIB_TEST_GYRO_SAMPLES; i++)
  {
    vec3_t s = {bias.x + test_noise(i, 0.006f), bias.y + test_noise(i + 7u, 0.006f),
                bias.z + test_noise(i + 13u, 0.006f)};
    calib_accum_add(&acc, &s);
  }
  CHECK(calib_gyro_bias(&acc, 0.0008726646f, &out) == 0);
  CHECK_NEAR_F(out.x, bias.x, 0.0005f);

  /* Moving board (~1.7 deg/s per sample): standard error exceeds 0.05 deg/s. */
  calib_accum_reset(&acc);
  for (uint32_t i = 0u; i < CALIB_TEST_GYRO_SAMPLES; i++)
  {
    vec3_t s = {bias.x + test_noise(i, 0.05f), bias.y + test_noise(i + 7u, 0.05f),
                bias.z + test_noise(i + 13u, 0.05f)};
    calib_accum_add(&acc, &s);
  }
  CHECK(calib_gyro_bias(&acc, 0.0008726646f, &out) != 0);

  CHECK(calib_gyro_bias(NULL, 0.001f, &out) == -EINVAL);
  calib_accum_reset(&acc);
  CHECK(calib_gyro_bias(&acc, 0.001f, &out) == -ENODATA);
}

static void test_accel_orientation(void)
{
  calib_accum_t acc;
  uint8_t axis = 0u;
  int8_t sign = 0;
  vec3_t v;

  /* Flat on body Z, small cross-axis component well inside 10 deg. */
  calib_accum_reset(&acc);
  v.x = 0.3f;
  v.y = -0.4f;
  v.z = 9.7f;
  add_const(&acc, &v, 100u);
  CHECK(calib_accel_orientation(&acc, &axis, &sign) == 0);
  CHECK_EQ_U16(axis, 2u);
  CHECK(sign == 1);

  /* 9 deg from +X passes, 12 deg does not. */
  calib_accum_reset(&acc);
  v.x = 9.80665f * 0.9876883f; /* sin(9 deg) */
  v.y = 9.80665f * 0.1564345f; /* cos(9 deg) */
  v.z = 0.0f;
  add_const(&acc, &v, 100u);
  CHECK(calib_accel_orientation(&acc, &axis, &sign) == 0);
  CHECK_EQ_U16(axis, 0u);
  CHECK(sign == 1);

  calib_accum_reset(&acc);
  v.x = 9.80665f * 0.9781476f; /* sin(12 deg) */
  v.y = 9.80665f * 0.2079117f; /* cos(12 deg) */
  v.z = 0.0f;
  add_const(&acc, &v, 100u);
  CHECK(calib_accel_orientation(&acc, &axis, &sign) != 0);

  /* Negative dominant axis. */
  calib_accum_reset(&acc);
  v.x = -9.6f;
  v.y = 0.2f;
  v.z = 0.3f;
  add_const(&acc, &v, 100u);
  CHECK(calib_accel_orientation(&acc, &axis, &sign) == 0);
  CHECK_EQ_U16(axis, 0u);
  CHECK(sign == -1);
  CHECK(calib_accum_still(&acc, 0.1f) == 0);

  /* Moving pose: mean is on-axis but the spread exceeds 0.1 m/s^2. */
  calib_accum_reset(&acc);
  for (uint32_t i = 0u; i < 100u; i++)
  {
    v.x = 0.0f;
    v.y = 0.0f;
    v.z = 9.8f + test_noise(i, 0.5f);
    calib_accum_add(&acc, &v);
  }
  CHECK(calib_accel_orientation(&acc, &axis, &sign) == 0);
  CHECK(calib_accum_still(&acc, 0.1f) == -ERANGE);
  CHECK(calib_accum_still(&acc, 0.0f) == -EINVAL);
}

static void test_accel_six_position(void)
{
  static const float scale[3] = {1.02f, 0.98f, 1.01f};
  static const float bias[3] = {0.15f, -0.22f, 0.33f};
  vec3_t pos[3];
  vec3_t neg[3];
  vec3_t out_bias;
  vec3_t out_scale;
  calib_params_t p;

  for (uint8_t i = 0u; i < 3u; i++)
  {
    float g = CALIB_STD_GRAVITY / scale[i];
    pos[i].x = bias[0];
    pos[i].y = bias[1];
    pos[i].z = bias[2];
    neg[i].x = bias[0];
    neg[i].y = bias[1];
    neg[i].z = bias[2];
    if (i == 0u)
    {
      pos[i].x += g;
      neg[i].x -= g;
    }
    else if (i == 1u)
    {
      pos[i].y += g;
      neg[i].y -= g;
    }
    else
    {
      pos[i].z += g;
      neg[i].z -= g;
    }
  }

  CHECK(calib_accel_solve(pos, neg, &out_bias, &out_scale) == 0);
  for (uint8_t i = 0u; i < 3u; i++)
  {
    /* Components are addressed through the vectors to keep the test readable. */
    float b = (i == 0u) ? out_bias.x : ((i == 1u) ? out_bias.y : out_bias.z);
    float s = (i == 0u) ? out_scale.x : ((i == 1u) ? out_scale.y : out_scale.z);

    CHECK_NEAR_F(b, bias[i], 1e-4f);
    CHECK_NEAR_F(s, scale[i], 1e-4f);
  }

  calib_params_defaults(&p);
  p.flags = CAL_FLAG_ACCEL;
  p.accel_bias = out_bias;
  p.accel_scale = out_scale;
  for (uint8_t i = 0u; i < 3u; i++)
  {
    vec3_t v = pos[i];

    calib_apply_accel(&p, &v);
    CHECK_NEAR_F((i == 0u) ? v.x : ((i == 1u) ? v.y : v.z), CALIB_STD_GRAVITY, 1e-3f);
  }

  /* An axis without a positive/negative separation cannot be solved. */
  neg[0] = pos[0];
  CHECK(calib_accel_solve(pos, neg, &out_bias, &out_scale) != 0);
}

static void test_mag_hard_soft_iron(void)
{
  const vec3_t bias = {20.0f, -35.0f, 12.0f};
  const vec3_t half = {45.0f, 40.0f, 38.0f};
  calib_range_t acc;
  vec3_t min;
  vec3_t max;
  vec3_t center = bias;
  vec3_t out_bias;
  vec3_t out_scale;
  calib_params_t p;

  min.x = center.x - half.x;
  min.y = center.y - half.y;
  min.z = center.z - half.z;
  max.x = center.x + half.x;
  max.y = center.y + half.y;
  max.z = center.z + half.z;

  calib_range_reset(&acc);
  calib_range_add(&acc, &min);
  calib_range_add(&acc, &center);
  calib_range_add(&acc, &max);
  CHECK(acc.n == 3u);
  CHECK(calib_mag_solve(&acc, 30.0f, 1.5f, &out_bias, &out_scale) == 0);
  CHECK_NEAR_F(out_bias.x, bias.x, 1e-4f);
  CHECK_NEAR_F(out_bias.y, bias.y, 1e-4f);
  CHECK_NEAR_F(out_bias.z, bias.z, 1e-4f);

  calib_params_defaults(&p);
  p.flags = CAL_FLAG_MAG;
  p.mag_bias = out_bias;
  p.mag_scale = out_scale;

  vec3_t lo = min;
  vec3_t hi = max;
  calib_apply_mag(&p, &lo);
  calib_apply_mag(&p, &hi);
  CHECK_NEAR_F(hi.x, 41.0f, 1e-3f);
  CHECK_NEAR_F(hi.y, 41.0f, 1e-3f);
  CHECK_NEAR_F(hi.z, 41.0f, 1e-3f);
  CHECK_NEAR_F(lo.x, -41.0f, 1e-3f);
  CHECK_NEAR_F(lo.y, -41.0f, 1e-3f);
  CHECK_NEAR_F(lo.z, -41.0f, 1e-3f);

  /* Span below 30 uT on any axis is rejected. */
  calib_range_reset(&acc);
  vec3_t small_min = {-14.0f, -15.0f, -15.0f};
  vec3_t small_max = {14.0f, 15.0f, 15.0f};
  calib_range_add(&acc, &small_min);
  calib_range_add(&acc, &small_max);
  CHECK(calib_mag_solve(&acc, 30.0f, 1.5f, &out_bias, &out_scale) != 0);

  /* Span ratio 2.5 is rejected even though every span is large enough. */
  calib_range_reset(&acc);
  vec3_t ratio_min = {-50.0f, -20.0f, -20.0f};
  vec3_t ratio_max = {50.0f, 20.0f, 20.0f};
  calib_range_add(&acc, &ratio_min);
  calib_range_add(&acc, &ratio_max);
  CHECK(calib_mag_solve(&acc, 30.0f, 1.5f, &out_bias, &out_scale) != 0);
}

static void test_baro_p0(void)
{
  calib_scalar_accum_t acc;
  float p0 = 0.0f;

  calib_scalar_accum_reset(&acc);
  for (uint32_t i = 0u; i < CALIB_TEST_BARO_SAMPLES; i++)
  {
    calib_scalar_accum_add(&acc, 100587.0f + test_noise(i, 2.0f));
  }
  CHECK(calib_baro_p0(&acc, 3.0f, &p0) == 0);
  CHECK_NEAR_F(p0, 100587.0f, 0.5f);

  calib_scalar_accum_reset(&acc);
  for (uint32_t i = 0u; i < CALIB_TEST_BARO_SAMPLES; i++)
  {
    calib_scalar_accum_add(&acc, 100587.0f + test_noise(i, 20.0f));
  }
  CHECK(calib_baro_p0(&acc, 3.0f, &p0) != 0);

  calib_scalar_accum_reset(&acc);
  CHECK(calib_baro_p0(&acc, 3.0f, &p0) == -ENODATA);
}

static void fill_params(calib_params_t *p)
{
  calib_params_defaults(p);
  p->gyro_bias.x = 0.001f;
  p->gyro_bias.y = -0.002f;
  p->gyro_bias.z = 0.0035f;
  p->accel_bias.x = 0.1f;
  p->accel_bias.y = -0.2f;
  p->accel_bias.z = 0.3f;
  p->accel_scale.x = 1.01f;
  p->accel_scale.y = 0.99f;
  p->accel_scale.z = 1.02f;
  p->mag_bias.x = 20.0f;
  p->mag_bias.y = -35.0f;
  p->mag_bias.z = 12.0f;
  p->mag_scale.x = 0.9f;
  p->mag_scale.y = 1.1f;
  p->mag_scale.z = 1.0f;
  p->baro_p0_pa = 100587.0f;
  p->flags = CAL_FLAG_ALL;
}

static void test_cal_store(void)
{
  calib_params_t in;
  calib_params_t out;
  uint8_t buf[CAL_STORE_LEN];
  uint16_t n;

  fill_params(&in);
  n = cal_store_encode(&in, buf, (uint16_t)sizeof(buf));
  CHECK_EQ_U16(n, CAL_STORE_LEN);

  /* Header is fixed by SPEC §22.3. */
  CHECK(buf[0] == 0x41u && buf[1] == 0x4Cu && buf[2] == 0x43u && buf[3] == 0x31u);
  CHECK(buf[4] == 2u && buf[5] == 0u);
  CHECK(buf[6] == 68u && buf[7] == 0u);

  memset(&out, 0, sizeof(out));
  CHECK(cal_store_decode(buf, n, &out) == 0);
  CHECK_EQ_U16(out.flags, CAL_FLAG_ALL);
  CHECK_NEAR_F(out.gyro_bias.x, in.gyro_bias.x, 0.0f);
  CHECK_NEAR_F(out.gyro_bias.y, in.gyro_bias.y, 0.0f);
  CHECK_NEAR_F(out.gyro_bias.z, in.gyro_bias.z, 0.0f);
  CHECK_NEAR_F(out.accel_bias.x, in.accel_bias.x, 0.0f);
  CHECK_NEAR_F(out.accel_bias.y, in.accel_bias.y, 0.0f);
  CHECK_NEAR_F(out.accel_bias.z, in.accel_bias.z, 0.0f);
  CHECK_NEAR_F(out.accel_scale.x, in.accel_scale.x, 0.0f);
  CHECK_NEAR_F(out.accel_scale.y, in.accel_scale.y, 0.0f);
  CHECK_NEAR_F(out.accel_scale.z, in.accel_scale.z, 0.0f);
  CHECK_NEAR_F(out.mag_bias.x, in.mag_bias.x, 0.0f);
  CHECK_NEAR_F(out.mag_bias.y, in.mag_bias.y, 0.0f);
  CHECK_NEAR_F(out.mag_bias.z, in.mag_bias.z, 0.0f);
  CHECK_NEAR_F(out.mag_scale.x, in.mag_scale.x, 0.0f);
  CHECK_NEAR_F(out.mag_scale.y, in.mag_scale.y, 0.0f);
  CHECK_NEAR_F(out.mag_scale.z, in.mag_scale.z, 0.0f);
  CHECK_NEAR_F(out.baro_p0_pa, in.baro_p0_pa, 0.0f);

  /* Bad magic. */
  fill_params(&in);
  n = cal_store_encode(&in, buf, (uint16_t)sizeof(buf));
  buf[0] ^= 0x01u;
  CHECK(cal_store_decode(buf, n, &out) == -EBADMSG);

  /* Bad version. */
  n = cal_store_encode(&in, buf, (uint16_t)sizeof(buf));
  buf[4] = 1u; /* v1 stored flags as a float */
  CHECK(cal_store_decode(buf, n, &out) == -EBADMSG);

  /* Bad payload length. */
  n = cal_store_encode(&in, buf, (uint16_t)sizeof(buf));
  buf[6] = 64u;
  CHECK(cal_store_decode(buf, n, &out) == -EBADMSG);

  /* Corrupted payload: CRC rejects it. */
  n = cal_store_encode(&in, buf, (uint16_t)sizeof(buf));
  buf[20] ^= 0x01u;
  CHECK(cal_store_decode(buf, n, &out) == -EBADMSG);

  /* Truncated record. */
  n = cal_store_encode(&in, buf, (uint16_t)sizeof(buf));
  CHECK(cal_store_decode(buf, (uint16_t)(n - 1u), &out) == -EINVAL);

  /* Flags outside CAL_FLAG_ALL are malformed. */
  in.flags = 0x00FFu;
  n = cal_store_encode(&in, buf, (uint16_t)sizeof(buf));
  CHECK(cal_store_decode(buf, n, &out) == -EBADMSG);

  /* Encode refuses a short buffer. */
  CHECK_EQ_U16(cal_store_encode(&in, buf, CAL_STORE_LEN - 1u), 0u);
}

static void test_cal_store_save(void)
{
  calib_params_t in;
  calib_params_t out;

  fill_params(&in);
  in.flags = CAL_FLAG_GYRO | CAL_FLAG_ACCEL | CAL_FLAG_BARO;

  uint32_t ms = 0u;

  stub_flash_reset();
  stub_flash_elapsed_ms = 37u;
  CHECK(cal_store_save(&in, &ms) == 0);
  CHECK(ms == 37u);
  CHECK(stub_flash_calls == 1u);
  CHECK(stub_flash_addr == BSP_FLASH_PAGE_ADDR);
  CHECK_EQ_U16(stub_flash_len, CAL_STORE_LEN);
  CHECK(cal_store_decode(stub_flash_data, stub_flash_len, &out) == 0);
  CHECK_EQ_U16(out.flags, in.flags);
  CHECK_NEAR_F(out.baro_p0_pa, in.baro_p0_pa, 0.0f);
  CHECK_EQ_U16(stub_flash_data[72], in.flags);
  CHECK_EQ_U16(stub_flash_data[74], 0u);

  CHECK(cal_store_load(&out) == 0);
  CHECK_EQ_U16(out.flags, in.flags);

  stub_flash_reset();
  stub_flash_result = -EIO;
  CHECK(cal_store_save(&in, NULL) == -EIO);
  CHECK(stub_flash_calls == 1u);

  /* Erased page loads defaults. */
  stub_flash_reset();
  CHECK(cal_store_load(&out) == -EBADMSG);
  CHECK_EQ_U16(out.flags, 0u);
  CHECK_NEAR_F(out.accel_scale.x, 1.0f, 0.0f);
}

#define ALL_OK (STATUS_GYRO_OK | STATUS_ACCEL_OK | STATUS_MAG_OK | STATUS_BARO_OK)
#define ALL_CAL (STATUS_GYRO_CAL | STATUS_ACCEL_CAL | STATUS_MAG_CAL | STATUS_BARO_REF)

static uint8_t rejected(void)
{
  return ((calibration_state() & CALIB_STATE_REJECTED) != 0u) ? 1u : 0u;
}

static uint8_t accel_count(void)
{
  return (uint8_t)((calibration_state() >> CALIB_STATE_ACCEL_COUNT_SHIFT) &
                   CALIB_STATE_ACCEL_COUNT_MASK);
}

static void feed_accel_pose(uint8_t axis, float sign, float wobble)
{
  for (uint32_t i = 0u; i < CALIB_ACCEL_SAMPLES; i++)
  {
    vec3_t v = {0.1f, -0.1f, 0.2f};
    float g = (sign * 9.7f) + test_noise(i, wobble);

    if (axis == 0u)
    {
      v.x += g;
    }
    else if (axis == 1u)
    {
      v.y += g;
    }
    else
    {
      v.z += g;
    }
    calibration_feed_accel(&v);
  }
}

static void feed_mag(uint32_t n)
{
  static const vec3_t dir[6] = {{1.0f, 0.0f, 0.0f},  {-1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f},
                                {0.0f, -1.0f, 0.0f}, {0.0f, 0.0f, 1.0f},  {0.0f, 0.0f, -1.0f}};

  for (uint32_t i = 0u; i < n; i++)
  {
    vec3_t v = {10.0f + (45.0f * dir[i % 6u].x), -5.0f + (45.0f * dir[i % 6u].y),
                3.0f + (45.0f * dir[i % 6u].z)};

    calibration_feed_mag(&v);
  }
}

static void test_state_machine(void)
{
  const vec3_t gb = {0.01f, -0.02f, 0.005f};
  vec3_t v;
  uint32_t ms = 0u;

  /* Boot with an erased page: defaults, boot gyro capture running. */
  stub_flash_reset();
  calibration_init(ALL_OK);
  CHECK(calibration_status() == 0u);
  CHECK((calibration_state() & CALIB_STATE_GYRO_ACTIVE) != 0u);
  CHECK(calibration_active() == 1u);
  for (uint32_t i = 0u; i < CALIB_GYRO_SAMPLES; i++)
  {
    calibration_feed_gyro(&gb);
  }
  CHECK((calibration_status() & STATUS_GYRO_CAL) != 0u);
  CHECK(calibration_state() == 0u);
  v = gb;
  calibration_apply_gyro(&v);
  CHECK_NEAR_F(v.z, 0.0f, 1e-6f);

  /* Moving gyro capture: rejected, previous bias kept. */
  calibration_start_gyro();
  for (uint32_t i = 0u; i < CALIB_GYRO_SAMPLES; i++)
  {
    vec3_t s = {((i & 1u) != 0u) ? 0.2f : -0.2f, 0.0f, 0.0f};
    calibration_feed_gyro(&s);
  }
  CHECK(rejected() == 1u);
  CHECK((calibration_status() & STATUS_GYRO_CAL) != 0u);
  v = gb;
  calibration_apply_gyro(&v);
  CHECK_NEAR_F(v.x, 0.0f, 1e-6f);

  /* Accel: first pose accepted, duplicate and moving poses rejected. */
  calibration_start_accel();
  CHECK(rejected() == 0u);
  feed_accel_pose(2u, 1.0f, 0.0f);
  CHECK(accel_count() == 1u);
  CHECK(rejected() == 0u);
  CHECK(calibration_active() == 1u);

  calibration_start_accel();
  feed_accel_pose(2u, 1.0f, 0.0f);
  CHECK(rejected() == 1u);
  CHECK(accel_count() == 1u);

  calibration_start_accel();
  feed_accel_pose(2u, -1.0f, 0.5f);
  CHECK(rejected() == 1u);
  CHECK(accel_count() == 1u);

  for (uint8_t k = 0u; k < 5u; k++)
  {
    static const uint8_t ax[5] = {2u, 0u, 0u, 1u, 1u};
    static const float sg[5] = {-1.0f, 1.0f, -1.0f, 1.0f, -1.0f};

    calibration_start_accel();
    feed_accel_pose(ax[k], sg[k], 0.0f);
    CHECK(rejected() == 0u);
  }
  CHECK((calibration_status() & STATUS_ACCEL_CAL) != 0u);
  CHECK(accel_count() == 0u);
  CHECK(calibration_active() == 0u);
  v.x = 0.1f;
  v.y = -0.1f;
  v.z = 0.2f + 9.7f;
  calibration_apply_accel(&v);
  CHECK_NEAR_F(v.z, CALIB_STD_GRAVITY, 1e-3f);
  CHECK_NEAR_F(v.x, 0.0f, 1e-4f);

  /* Mag: stopping early rejects; a full capture solves. */
  calibration_toggle_mag();
  CHECK((calibration_state() & CALIB_STATE_MAG_ACTIVE) != 0u);
  feed_mag(CALIB_MAG_MIN_SAMPLES - 1u);
  calibration_toggle_mag();
  CHECK(rejected() == 1u);
  CHECK((calibration_status() & STATUS_MAG_CAL) == 0u);

  calibration_toggle_mag();
  CHECK(rejected() == 0u);
  feed_mag(CALIB_MAG_MIN_SAMPLES);
  calibration_toggle_mag();
  CHECK(rejected() == 0u);
  CHECK((calibration_status() & STATUS_MAG_CAL) != 0u);
  v.x = 55.0f;
  v.y = -5.0f;
  v.z = 3.0f;
  calibration_apply_mag(&v);
  CHECK_NEAR_F(v.x, 45.0f, 1e-3f);

  /* Baro. */
  calibration_start_baro();
  for (uint32_t i = 0u; i < CALIB_BARO_SAMPLES; i++)
  {
    calibration_feed_baro(100587.0f + test_noise(i, 1.0f));
  }
  CHECK(rejected() == 0u);
  CHECK(calibration_status() == ALL_CAL);
  CHECK_NEAR_F(calibration_p0_pa(), 100587.0f, 0.5f);

  /* Save is deferred to the main loop and reports the stall. */
  calibration_request_save();
  CHECK(calibration_take_save_request() == 1u);
  CHECK(calibration_take_save_request() == 0u);
  stub_flash_elapsed_ms = 25u;
  CHECK(calibration_save(&ms) == 0);
  CHECK(ms == 25u);
  CHECK(rejected() == 0u);

  /* Failed save is reported, stored page untouched. */
  stub_flash_result = -EIO;
  calibration_request_save();
  CHECK(calibration_save(NULL) == -EIO);
  CHECK(rejected() == 1u);
  stub_flash_result = 0;

  /* T-15: D clears RAM only; a reboot reloads the stored values. */
  calibration_reset_defaults();
  CHECK(calibration_status() == 0u);
  CHECK(rejected() == 0u);
  calibration_init(0u);
  CHECK(calibration_status() == ALL_CAL);
  CHECK(calibration_state() == 0u);

  /* Command for an absent sensor is rejected. */
  calibration_start_baro();
  CHECK(rejected() == 1u);
  CHECK(calibration_state() == CALIB_STATE_REJECTED);
}

void test_calibration(void)
{
  test_defaults();
  test_gyro_still();
  test_accel_orientation();
  test_accel_six_position();
  test_mag_hard_soft_iron();
  test_baro_p0();
  test_cal_store();
  test_cal_store_save();
  test_state_machine();
}
