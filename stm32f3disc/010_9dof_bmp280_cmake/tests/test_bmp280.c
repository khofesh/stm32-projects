#include "test.h"

#include "bmp280_regs_stub.h"

/* Datasheet §3.12 worked example (BMP280 datasheet, calibration and ADC
 * values of the reference measurement). */
#define CAL_T1 27504u
#define CAL_T2 26435u
#define CAL_T3 0xFC18u /* -1000 */
#define CAL_P1 36477u
#define CAL_P2 0xD643u /* -10685 */
#define CAL_P3 3024u
#define CAL_P4 2855u
#define CAL_P5 140u
#define CAL_P6 0xFFF9u /* -7 */
#define CAL_P7 15500u
#define CAL_P8 0xC6F8u /* -14600 */
#define CAL_P9 6000u

#define ADC_T 519888u
#define ADC_P 415148u

#define EXPECT_T_C 25.08f
#define EXPECT_P_PA 100653.0f

static void load_raw(uint32_t adc_p, uint32_t adc_t)
{
  stub_regs[0xF7] = (uint8_t)(adc_p >> 12);
  stub_regs[0xF8] = (uint8_t)(adc_p >> 4);
  stub_regs[0xF9] = (uint8_t)(adc_p << 4);

  stub_regs[0xFA] = (uint8_t)(adc_t >> 12);
  stub_regs[0xFB] = (uint8_t)(adc_t >> 4);
  stub_regs[0xFC] = (uint8_t)(adc_t << 4);
}

void test_bmp280(void)
{
  bmp280_handle_t h;
  uint32_t t_raw = 0u;
  uint32_t p_raw = 0u;
  float t = 0.0f;
  float p = 0.0f;

  bmp280_stub_calibration(CAL_T1, CAL_T2, CAL_T3, CAL_P1, CAL_P2, CAL_P3, CAL_P4, CAL_P5,
                          CAL_P6, CAL_P7, CAL_P8, CAL_P9);
  bmp280_stub_link(&h);

  /* bmp280_init() checks the chip ID and parses the NVM calibration. */
  CHECK(bmp280_init(&h) == 0u);
  CHECK(h.t1 == CAL_T1);
  CHECK(h.t2 == (int16_t)CAL_T2);
  CHECK(h.t3 == (int16_t)CAL_T3);
  CHECK(h.p1 == CAL_P1);
  CHECK(h.p9 == 6000);

  load_raw(ADC_P, ADC_T);
  CHECK(bmp280_read_temperature_pressure(&h, &t_raw, &t, &p_raw, &p) == 0u);

  CHECK(t_raw == ADC_T);
  CHECK(p_raw == ADC_P);
  CHECK_NEAR_F(t, EXPECT_T_C, 0.01f);
  CHECK_NEAR_F(p, EXPECT_P_PA, 1.0f);
}
