#include "altitude.h"

#include <math.h>

#define ALTITUDE_EXPONENT (1.0f / 5.255f)

float altitude_from_pressure(float pa, float p0_pa)
{
  if ((pa <= 0.0f) || (p0_pa <= 0.0f))
  {
    return 0.0f;
  }
  return 44330.0f * (1.0f - powf(pa / p0_pa, ALTITUDE_EXPONENT));
}
