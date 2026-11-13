#ifndef __AXIS_MAP_H
#define __AXIS_MAP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "vec3.h"

/* out = m * in, m being a signed permutation (one +/-1 per row and column). */
void axis_map_apply(const int8_t m[3][3], const vec3_t *in, vec3_t *out);

#ifdef __cplusplus
}
#endif

#endif /* __AXIS_MAP_H */
