#include "test.h"

#include <errno.h>
#include <string.h>

#include "bsp_uart_stub.h"
#include "crc16.h"
#include "telemetry.h"

#define DATA_SEQ 0x1234u
#define DATA_MS 0xDEADBEEFu

static void fill_data(frame_data_t *f)
{
  memset(f, 0, sizeof(*f));
  f->seq = DATA_SEQ;
  f->t_ms = DATA_MS;
  f->q[0] = 1.0f;
  f->q[1] = 0.25f;
  f->q[2] = -0.5f;
  f->q[3] = 0.75f;
  f->accel.x = 1.5f;
  f->accel.y = -2.5f;
  f->accel.z = 9.75f;
  f->gyro.x = 0.125f;
  f->gyro.y = -0.25f;
  f->gyro.z = 1.75f;
  f->mag.x = 21.5f;
  f->mag.y = -42.25f;
  f->mag.z = 8.0f;
  f->pressure_pa = 100653.0f;
  f->altitude_m = 12.5f;
  f->temperature_cdeg = -2501;
  f->status = STATUS_GYRO_OK | STATUS_ACCEL_OK | STATUS_MAG_OK | STATUS_BARO_OK;
  f->cpu_load = 17u;
  f->calib_state = 3u;
  f->overruns = 7u;
  f->frames_skipped = 9u;
}

void test_frame_data_layout(void)
{
  uint8_t buf[FRAME_LEN];
  frame_data_t in;
  frame_data_t out;
  uint16_t crc;

  fill_data(&in);
  frame_data_pack(buf, &in);

  CHECK(buf[0] == 0xAAu);
  CHECK(buf[1] == 0x55u);
  CHECK(buf[2] == 0x01u);
  CHECK(buf[3] == 78u);

  /* Sequence, timestamp. */
  CHECK(buf[4] == 0x34u);
  CHECK(buf[5] == 0x12u);
  CHECK(buf[6] == 0xEFu);
  CHECK(buf[9] == 0xDEu);

  /* Quaternion starts at 10, accel at 26, gyro at 38, mag at 50. */
  CHECK(buf[10] == 0x00u && buf[11] == 0x00u && buf[12] == 0x80u && buf[13] == 0x3Fu);
  CHECK(buf[26] == 0x00u && buf[27] == 0x00u && buf[28] == 0xC0u && buf[29] == 0x3Fu);
  CHECK(buf[50] == 0x00u && buf[51] == 0x00u && buf[52] == 0xACu && buf[53] == 0x41u);

  /* Status, CPU load, calib_state, counters, reserved. */
  CHECK(buf[72] == 0x0Fu && buf[73] == 0x00u);
  CHECK(buf[74] == 17u);
  CHECK(buf[75] == 3u);
  CHECK(buf[76] == 7u && buf[77] == 0u);
  CHECK(buf[78] == 9u && buf[79] == 0u);
  CHECK(buf[80] == 0u && buf[81] == 0u);

  /* CRC covers bytes 2..81. */
  crc = crc16_ccitt(&buf[2], 80u);
  CHECK(buf[82] == (uint8_t)(crc & 0xFFu));
  CHECK(buf[83] == (uint8_t)(crc >> 8));

  CHECK(frame_data_unpack(buf, &out) == 0);
  CHECK(out.seq == DATA_SEQ);
  CHECK(out.t_ms == DATA_MS);
  CHECK_NEAR_F(out.q[0], 1.0f, 0.0f);
  CHECK_NEAR_F(out.q[1], 0.25f, 0.0f);
  CHECK_NEAR_F(out.q[2], -0.5f, 0.0f);
  CHECK_NEAR_F(out.q[3], 0.75f, 0.0f);
  CHECK_NEAR_F(out.accel.x, 1.5f, 0.0f);
  CHECK_NEAR_F(out.accel.y, -2.5f, 0.0f);
  CHECK_NEAR_F(out.accel.z, 9.75f, 0.0f);
  CHECK_NEAR_F(out.gyro.x, 0.125f, 0.0f);
  CHECK_NEAR_F(out.gyro.y, -0.25f, 0.0f);
  CHECK_NEAR_F(out.gyro.z, 1.75f, 0.0f);
  CHECK_NEAR_F(out.mag.x, 21.5f, 0.0f);
  CHECK_NEAR_F(out.mag.y, -42.25f, 0.0f);
  CHECK_NEAR_F(out.mag.z, 8.0f, 0.0f);
  CHECK_NEAR_F(out.pressure_pa, 100653.0f, 0.01f);
  CHECK_NEAR_F(out.altitude_m, 12.5f, 0.0f);
  CHECK(out.temperature_cdeg == -2501);
  CHECK(out.status == in.status);
  CHECK(out.cpu_load == 17u);
  CHECK(out.calib_state == 3u);
  CHECK(out.overruns == 7u);
  CHECK(out.frames_skipped == 9u);
}

void test_frame_rejects_bad_frames(void)
{
  uint8_t buf[FRAME_LEN];
  frame_data_t in;
  frame_data_t out;

  fill_data(&in);
  frame_data_pack(buf, &in);

  buf[0] = 0x00u;
  CHECK(frame_data_unpack(buf, &out) == -EBADMSG);

  frame_data_pack(buf, &in);
  buf[2] = 0x02u;
  CHECK(frame_data_unpack(buf, &out) == -EBADMSG);

  frame_data_pack(buf, &in);
  buf[3] = 77u;
  CHECK(frame_data_unpack(buf, &out) == -EBADMSG);

  frame_data_pack(buf, &in);
  buf[40] ^= 0x01u;
  CHECK(frame_data_unpack(buf, &out) == -EBADMSG);
}

void test_frame_debug_layout(void)
{
  static const char desc[] = "v0.1-3-gabcdef-dirty";
  uint8_t buf[FRAME_LEN];
  frame_debug_t in;
  frame_debug_t out;
  uint16_t crc;

  memset(&in, 0, sizeof(in));
  in.seq = 5u;
  in.t_ms = 12345u;
  memcpy(in.build_git_desc, desc, sizeof(desc) - 1u);
  in.i2c1_errors = 1u;
  in.i2c2_errors = 2u;
  in.spi1_errors = 3u;
  in.max_loop_us = 4u;
  in.cpu_load_permille = 5u;
  in.free_stack = 6u;

  frame_debug_pack(buf, &in);

  CHECK(buf[2] == 0x02u);
  CHECK(buf[3] == 78u);
  CHECK(buf[42] == 1u && buf[43] == 0u);
  CHECK(buf[44] == 2u);
  CHECK(buf[46] == 3u);
  CHECK(buf[48] == 4u);
  CHECK(buf[50] == 5u);
  CHECK(buf[52] == 6u);

  /* Descriptor is NUL-padded, reserved bytes are zero. */
  CHECK(buf[10] == (uint8_t)'v');
  CHECK(buf[10 + (sizeof(desc) - 1u)] == 0u);
  for (uint16_t i = 54u; i < 82u; i++)
  {
    CHECK(buf[i] == 0u);
  }

  crc = crc16_ccitt(&buf[2], 80u);
  CHECK(buf[82] == (uint8_t)(crc & 0xFFu));
  CHECK(buf[83] == (uint8_t)(crc >> 8));

  CHECK(frame_debug_unpack(buf, &out) == 0);
  CHECK(out.seq == 5u);
  CHECK(out.t_ms == 12345u);
  CHECK(out.max_loop_us == 4u);
  CHECK(out.free_stack == 6u);
  for (uint8_t i = 0u; i < 32u; i++)
  {
    CHECK(out.build_git_desc[i] == in.build_git_desc[i]);
  }
}

void test_telemetry_send(void)
{
  frame_data_t f;
  uint16_t before;

  fill_data(&f);

  stub_tx_busy = 0;
  stub_tx_len = 0;
  CHECK(telemetry_send_data(&f) == 0);
  CHECK(stub_tx_len == FRAME_LEN);
  CHECK(stub_tx[0] == 0xAAu && stub_tx[1] == 0x55u && stub_tx[2] == 0x01u);

  before = telemetry_skipped();
  stub_tx_busy = 1;
  CHECK(telemetry_send_data(&f) == -EBUSY);
  CHECK(telemetry_skipped() == (uint16_t)(before + 1u));
  stub_tx_busy = 0;
}

void test_frame(void)
{
  test_frame_data_layout();
  test_frame_rejects_bad_frames();
  test_frame_debug_layout();
  test_telemetry_send();
}
