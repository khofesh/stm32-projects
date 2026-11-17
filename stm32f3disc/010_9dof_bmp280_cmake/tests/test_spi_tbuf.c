#include "test.h"

#include <string.h>

#include "spi_tbuf.h"

#define FRAME_BYTES 8u

static void frame_fill(uint8_t *f, uint8_t v)
{
  memset(f, v, FRAME_BYTES);
}

static void test_spi_tbuf_init(void)
{
  spi_tbuf_t t;

  spi_tbuf_init(&t);
  CHECK(t.write_idx < SPI_TBUF_COUNT);
  CHECK(t.pub_idx < SPI_TBUF_COUNT);
  CHECK(t.armed_idx < SPI_TBUF_COUNT);
  CHECK(t.write_idx != t.pub_idx);
  CHECK(t.write_idx != t.armed_idx);
  /* The all-zero frame is armed before the first publish. */
  CHECK(t.pub_idx == t.armed_idx);
  CHECK(spi_tbuf_next_free(&t) != t.armed_idx);
}

static void test_spi_tbuf_sequence(void)
{
  spi_tbuf_t t;
  uint8_t seen = 0u;

  spi_tbuf_init(&t);
  for (uint8_t i = 0u; i < 9u; i++)
  {
    uint8_t w = spi_tbuf_next_free(&t);

    CHECK(w < SPI_TBUF_COUNT);
    CHECK(w != t.armed_idx);
    CHECK(w != t.pub_idx);
    seen |= (uint8_t)(1u << w);

    spi_tbuf_publish(&t);
    CHECK(t.pub_idx == w);
    CHECK(t.write_idx != w);
    CHECK(t.write_idx != t.armed_idx);

    CHECK(spi_tbuf_arm(&t) == w);
    CHECK(t.armed_idx == w);
    CHECK(t.write_idx != w);
  }
  CHECK(seen == 7u); /* all three buffers take turns */
}

static void test_spi_tbuf_armed_never_written(void)
{
  spi_tbuf_t t;
  uint8_t buf[SPI_TBUF_COUNT][FRAME_BYTES];
  uint8_t snapshot[FRAME_BYTES];

  memset(buf, 0, sizeof(buf));
  spi_tbuf_init(&t);
  (void)spi_tbuf_arm(&t);
  memcpy(snapshot, buf[t.armed_idx], FRAME_BYTES);

  for (uint16_t i = 0u; i < 300u; i++)
  {
    uint8_t w = spi_tbuf_next_free(&t);

    CHECK(w != t.armed_idx);
    CHECK(memcmp(buf[t.armed_idx], snapshot, FRAME_BYTES) == 0);

    frame_fill(buf[w], (uint8_t)i);
    CHECK(memcmp(buf[t.armed_idx], snapshot, FRAME_BYTES) == 0);

    spi_tbuf_publish(&t);
    CHECK(t.pub_idx == w);
    CHECK(memcmp(buf[t.armed_idx], snapshot, FRAME_BYTES) == 0);

    CHECK(spi_tbuf_arm(&t) == w);
    memcpy(snapshot, buf[w], FRAME_BYTES);
  }
}

static void test_spi_tbuf_publish_during_transaction(void)
{
  spi_tbuf_t t;
  uint8_t buf[SPI_TBUF_COUNT][FRAME_BYTES];
  uint8_t frame_a[FRAME_BYTES];
  uint8_t frame_b[FRAME_BYTES];
  uint8_t armed;
  uint8_t w;

  memset(buf, 0, sizeof(buf));
  spi_tbuf_init(&t);

  /* Frame A published and armed: the master starts a transaction. */
  w = spi_tbuf_next_free(&t);
  frame_fill(buf[w], 0xA5u);
  memcpy(frame_a, buf[w], FRAME_BYTES);
  spi_tbuf_publish(&t);
  armed = spi_tbuf_arm(&t);
  CHECK(armed == w);

  /* Frame B is published while frame A is clocked out. */
  w = spi_tbuf_next_free(&t);
  CHECK(w != armed);
  frame_fill(buf[w], 0x5Au);
  memcpy(frame_b, buf[w], FRAME_BYTES);
  spi_tbuf_publish(&t);
  CHECK(t.pub_idx == w);
  CHECK(t.armed_idx == armed);
  CHECK(memcmp(buf[armed], frame_a, FRAME_BYTES) == 0);

  /* The NSS rising edge arms the newest frame for the next transaction. */
  CHECK(spi_tbuf_arm(&t) == w);
  CHECK(memcmp(buf[t.armed_idx], frame_b, FRAME_BYTES) == 0);
  CHECK(memcmp(buf[t.armed_idx], frame_a, FRAME_BYTES) != 0);
}

static void test_spi_tbuf_publish_without_arm(void)
{
  spi_tbuf_t t;
  uint8_t buf[SPI_TBUF_COUNT][FRAME_BYTES];
  uint8_t snapshot[FRAME_BYTES];
  uint8_t armed;

  memset(buf, 0, sizeof(buf));
  spi_tbuf_init(&t);
  armed = spi_tbuf_arm(&t);
  memcpy(snapshot, buf[armed], FRAME_BYTES);

  /* The master holds NSS low across several publishes. */
  for (uint8_t i = 0u; i < 6u; i++)
  {
    uint8_t w = spi_tbuf_next_free(&t);

    CHECK(w != armed);
    CHECK(w != t.pub_idx);
    frame_fill(buf[w], (uint8_t)(0x10u + i));
    spi_tbuf_publish(&t);
    CHECK(t.armed_idx == armed);
    CHECK(memcmp(buf[armed], snapshot, FRAME_BYTES) == 0);
  }

  /* NSS finally rises: the newest published frame is armed. */
  CHECK(spi_tbuf_arm(&t) == t.pub_idx);
}

static void test_spi_tbuf_state_invariants(void)
{
  /* Every reachable state: write is neither published nor armed. */
  for (uint8_t w = 0u; w < SPI_TBUF_COUNT; w++)
  {
    for (uint8_t p = 0u; p < SPI_TBUF_COUNT; p++)
    {
      for (uint8_t a = 0u; a < SPI_TBUF_COUNT; a++)
      {
        spi_tbuf_t t;
        uint8_t armed;

        if ((w == p) || (w == a))
        {
          continue;
        }
        t.write_idx = w;
        t.pub_idx = p;
        t.armed_idx = a;

        spi_tbuf_publish(&t);
        CHECK(t.pub_idx == w);
        CHECK(t.write_idx != t.pub_idx);
        CHECK(t.write_idx != t.armed_idx);

        armed = spi_tbuf_arm(&t);
        CHECK(armed == w);
        CHECK(t.armed_idx == w);
        CHECK(t.write_idx != t.armed_idx);
      }
    }
  }
}

void test_spi_tbuf(void)
{
  test_spi_tbuf_init();
  test_spi_tbuf_sequence();
  test_spi_tbuf_state_invariants();
  test_spi_tbuf_armed_never_written();
  test_spi_tbuf_publish_during_transaction();
  test_spi_tbuf_publish_without_arm();
}
