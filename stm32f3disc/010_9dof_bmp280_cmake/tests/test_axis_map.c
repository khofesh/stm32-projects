#include "axis_map.h"
#include "test.h"

void test_axis_map(void)
{
  static const int8_t ident[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
  /* x' = y, y' = -x, z' = -z */
  static const int8_t rot[3][3] = {{0, 1, 0}, {-1, 0, 0}, {0, 0, -1}};
  const vec3_t in = {1.5f, -2.25f, 9.81f};
  vec3_t out;

  axis_map_apply(ident, &in, &out);
  CHECK_NEAR_F(out.x, 1.5f, 1e-6f);
  CHECK_NEAR_F(out.y, -2.25f, 1e-6f);
  CHECK_NEAR_F(out.z, 9.81f, 1e-6f);

  axis_map_apply(rot, &in, &out);
  CHECK_NEAR_F(out.x, -2.25f, 1e-6f);
  CHECK_NEAR_F(out.y, -1.5f, 1e-6f);
  CHECK_NEAR_F(out.z, -9.81f, 1e-6f);

  /* In-place: out aliases in. */
  vec3_t v = in;
  axis_map_apply(rot, &v, &v);
  CHECK_NEAR_F(v.x, -2.25f, 1e-6f);
  CHECK_NEAR_F(v.y, -1.5f, 1e-6f);
  CHECK_NEAR_F(v.z, -9.81f, 1e-6f);
}
