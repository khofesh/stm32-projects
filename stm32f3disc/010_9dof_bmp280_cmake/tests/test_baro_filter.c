#include "test.h"

#include "baro_filter.h"

void test_baro_filter(void)
{
  float y = 0.0f;

  /* First sample primes the filter. */
  baro_filter_init(0.2f, 0.01f, 0.04f);
  CHECK_NEAR_F(baro_filter_update(100.0f), 100.0f, 0.0f);
  for (uint32_t i = 0u; i < 100u; i++)
  {
    y = baro_filter_update(100.0f);
  }
  CHECK_NEAR_F(y, 100.0f, 1e-4f);

  /* Step 0 -> 10 m: position settles, velocity decays. */
  baro_filter_init(0.2f, 0.01f, 0.04f);
  (void)baro_filter_update(0.0f);
  for (uint32_t i = 0u; i < 5000u; i++)
  {
    y = baro_filter_update(10.0f);
  }
  CHECK_NEAR_F(y, 10.0f, 0.01f);

  /* Constant climb of 0.25 m/s after the transient. */
  baro_filter_init(0.2f, 0.01f, 0.04f);
  (void)baro_filter_update(0.0f);
  for (uint32_t i = 0u; i < 3000u; i++)
  {
    y = baro_filter_update(0.01f * (float)i);
  }
  CHECK_NEAR_F(y, 0.01f * 2999.0f, 0.05f);
}
