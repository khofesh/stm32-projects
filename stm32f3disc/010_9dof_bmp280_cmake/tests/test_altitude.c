#include "test.h"

#include "altitude.h"

void test_altitude(void)
{
  CHECK_NEAR_F(altitude_from_pressure(101325.0f, 101325.0f), 0.0f, 0.01f);

  /* ISA pressures at 500 m and 1000 m. */
  CHECK_NEAR_F(altitude_from_pressure(95460.8f, 101325.0f), 500.0f, 1.0f);
  CHECK_NEAR_F(altitude_from_pressure(89874.6f, 101325.0f), 1000.0f, 1.0f);

  CHECK_NEAR_F(altitude_from_pressure(90000.0f, 90000.0f), 0.0f, 0.01f);
  CHECK_NEAR_F(altitude_from_pressure(0.0f, 101325.0f), 0.0f, 0.0f);
  CHECK_NEAR_F(altitude_from_pressure(101325.0f, 0.0f), 0.0f, 0.0f);
  CHECK_NEAR_F(altitude_from_pressure(-1.0f, 101325.0f), 0.0f, 0.0f);
}
