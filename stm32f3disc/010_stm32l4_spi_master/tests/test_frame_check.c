#include <string.h>

#include "frame_check.h"
#include "test.h"

/* Data frame as the slave packs it: header, seq, filler, CRC over 2..81. */
static void make_data(uint8_t f[FC_FRAME_LEN], uint16_t seq)
{
  uint16_t crc;

  for (uint8_t i = 0u; i < FC_FRAME_LEN; i++)
  {
    f[i] = (uint8_t)(i * 7u);
  }
  f[0] = 0xAAu;
  f[1] = 0x55u;
  f[2] = 0x01u;
  f[3] = 78u;
  f[4] = (uint8_t)seq;
  f[5] = (uint8_t)(seq >> 8);
  crc = crc16_ccitt(&f[2], 80u);
  f[82] = (uint8_t)crc;
  f[83] = (uint8_t)(crc >> 8);
}

static void test_crc(void)
{
  static const uint8_t v[] = "123456789";

  CHECK(crc16_ccitt(v, 9u) == 0x29B1u);
  CHECK(crc16_ccitt(v, 0u) == 0xFFFFu);
}

static void test_check(void)
{
  uint8_t f[FC_FRAME_LEN];

  make_data(f, 0x1234u);
  CHECK(fc_check(f, FC_TYPE_DATA) == FC_OK);
  CHECK(fc_seq(f) == 0x1234u);
  CHECK(fc_check(f, FC_TYPE_STATS) == FC_BAD_HDR);

  memset(f, 0, sizeof(f));
  CHECK(fc_check(f, FC_TYPE_DATA) == FC_ZERO);

  make_data(f, 1u);
  f[0] = 0xABu;
  CHECK(fc_check(f, FC_TYPE_DATA) == FC_BAD_HDR);
  make_data(f, 1u);
  f[3] = 77u;
  CHECK(fc_check(f, FC_TYPE_DATA) == FC_BAD_HDR);
  make_data(f, 1u);
  f[40] ^= 0x01u;
  CHECK(fc_check(f, FC_TYPE_DATA) == FC_BAD_CRC);
  make_data(f, 1u);
  f[83] ^= 0x80u;
  CHECK(fc_check(f, FC_TYPE_DATA) == FC_BAD_CRC);

  /* A frame shifted by stale FIFO bytes (the slave bug T-16 hunts) fails. */
  make_data(f, 9u);
  memmove(&f[4], &f[0], FC_FRAME_LEN - 4u);
  CHECK(fc_check(f, FC_TYPE_DATA) != FC_OK);
}

static void test_seq(void)
{
  fc_seq_t s = {0};
  uint32_t gaps = 0u;
  uint32_t dups = 0u;

  fc_seq_track(&s, 100u, &gaps, &dups);
  CHECK(gaps == 0u && dups == 0u);
  fc_seq_track(&s, 101u, &gaps, &dups);
  CHECK(gaps == 0u && dups == 0u);
  fc_seq_track(&s, 101u, &gaps, &dups);
  CHECK(gaps == 0u && dups == 1u);
  fc_seq_track(&s, 105u, &gaps, &dups);
  CHECK(gaps == 3u && dups == 1u);

  s.valid = 0u;
  gaps = 0u;
  fc_seq_track(&s, 65534u, &gaps, &dups);
  fc_seq_track(&s, 65535u, &gaps, &dups);
  fc_seq_track(&s, 0u, &gaps, &dups);
  fc_seq_track(&s, 2u, &gaps, &dups);
  CHECK(gaps == 1u);
  CHECK(s.last == 2u);
}

static void test_stats(void)
{
  uint8_t f[FC_FRAME_LEN];
  uint32_t v[FC_STATS_MAX + 2u];

  for (uint8_t i = 0u; i < (FC_STATS_MAX + 2u); i++)
  {
    v[i] = 0x01020304u + i;
  }
  fc_stats_pack(f, 0xBEEFu, 0x11223344u, v, 3u);
  CHECK(fc_check(f, FC_TYPE_STATS) == FC_OK);
  CHECK(f[4] == 0xEFu && f[5] == 0xBEu);
  CHECK(f[6] == 0x44u && f[9] == 0x11u);
  CHECK(f[10] == 0x04u && f[13] == 0x01u);
  CHECK(f[18] == 0x06u);
  CHECK(f[22] == 0u && f[81] == 0u);

  /* n is clamped: the last field ends right before the CRC. */
  fc_stats_pack(f, 0u, 0u, v, (uint8_t)(FC_STATS_MAX + 2u));
  CHECK(fc_check(f, FC_TYPE_STATS) == FC_OK);
  CHECK(f[78] == (uint8_t)(0x04u + FC_STATS_MAX - 1u));
}

void test_frame_check(void)
{
  test_crc();
  test_check();
  test_seq();
  test_stats();
}
