#include "axis_map.h"

void axis_map_apply(const int8_t m[3][3], const vec3_t *in, vec3_t *out)
{
  const vec3_t v = *in;

  out->x = ((float)m[0][0] * v.x) + ((float)m[0][1] * v.y) + ((float)m[0][2] * v.z);
  out->y = ((float)m[1][0] * v.x) + ((float)m[1][1] * v.y) + ((float)m[1][2] * v.z);
  out->z = ((float)m[2][0] * v.x) + ((float)m[2][1] * v.y) + ((float)m[2][2] * v.z);
}
