#include "axis_map.h"

#include <stddef.h>

const int8_t axis_map_identity[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};

void axis_map_apply(const int8_t m[3][3], const vec3_t *in, vec3_t *out)
{
  const vec3_t v = *in;

  out->x = ((float)m[0][0] * v.x) + ((float)m[0][1] * v.y) + ((float)m[0][2] * v.z);
  out->y = ((float)m[1][0] * v.x) + ((float)m[1][1] * v.y) + ((float)m[1][2] * v.z);
  out->z = ((float)m[2][0] * v.x) + ((float)m[2][1] * v.y) + ((float)m[2][2] * v.z);
}

uint8_t axis_map_is_signed_permutation(const int8_t m[3][3])
{
  uint8_t rows_ok = 0u;
  uint8_t cols_ok = 0u;

  if (m == NULL)
  {
    return 0u;
  }

  for (uint8_t r = 0u; r < 3u; r++)
  {
    uint8_t nonzero = 0u;

    for (uint8_t c = 0u; c < 3u; c++)
    {
      if ((m[r][c] != 0) && (m[r][c] != 1) && (m[r][c] != -1))
      {
        return 0u;
      }
      if (m[r][c] != 0)
      {
        nonzero++;
      }
    }
    if (nonzero == 1u)
    {
      rows_ok++;
    }
  }

  for (uint8_t c = 0u; c < 3u; c++)
  {
    uint8_t nonzero = 0u;

    for (uint8_t r = 0u; r < 3u; r++)
    {
      if (m[r][c] != 0)
      {
        nonzero++;
      }
    }
    if (nonzero == 1u)
    {
      cols_ok++;
    }
  }

  return ((rows_ok == 3u) && (cols_ok == 3u)) ? 1u : 0u;
}
