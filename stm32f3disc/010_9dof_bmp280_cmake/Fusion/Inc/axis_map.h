#ifndef __AXIS_MAP_H
#define __AXIS_MAP_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>

#include "vec3.h"

  /* out = m * in, m being a signed permutation (one +/-1 per row and column). */
  void axis_map_apply(const int8_t m[3][3], const vec3_t *in, vec3_t *out);

  /* True if m is a signed permutation: exactly one +/-1 per row and column.
 * The determinant may be -1: some ST sensor packages use a left-handed
 * output triad (measured on the LSM303AGR of this board, T-03). */
  uint8_t axis_map_is_signed_permutation(const int8_t m[3][3]);

  extern const int8_t axis_map_identity[3][3];

#ifdef __cplusplus
}
#endif

#endif /* __AXIS_MAP_H */
