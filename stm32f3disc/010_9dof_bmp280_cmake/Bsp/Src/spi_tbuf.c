#include "spi_tbuf.h"

static uint8_t other(uint8_t a, uint8_t b)
{
  for (uint8_t i = 0u; i < SPI_TBUF_COUNT; i++)
  {
    if ((i != a) && (i != b))
    {
      return i;
    }
  }
  return 0u;
}

void spi_tbuf_init(spi_tbuf_t *t)
{
  t->pub_idx = 0u;
  t->armed_idx = 0u;
  t->write_idx = other(t->pub_idx, t->armed_idx);
}

uint8_t spi_tbuf_next_free(const spi_tbuf_t *t)
{
  return t->write_idx;
}

void spi_tbuf_publish(spi_tbuf_t *t)
{
  uint8_t w = t->write_idx;
  uint8_t a;

  /* Publish first, then read armed: an NSS edge in between can only arm w,
   * and the third buffer is then free either way. */
  t->pub_idx = w;
  a = t->armed_idx;
  t->write_idx = other(w, a);
}

uint8_t spi_tbuf_arm(spi_tbuf_t *t)
{
  uint8_t idx = t->pub_idx;

  t->armed_idx = idx;
  return idx;
}
