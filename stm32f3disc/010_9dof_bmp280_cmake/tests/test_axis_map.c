#include "axis_map.h"
#include "board_axes.h"
#include "test.h"

static int32_t det3(const int8_t m[3][3])
{
  return ((int32_t)m[0][0] *
          (((int32_t)m[1][1] * m[2][2]) - ((int32_t)m[1][2] * m[2][1]))) -
         ((int32_t)m[0][1] *
          (((int32_t)m[1][0] * m[2][2]) - ((int32_t)m[1][2] * m[2][0]))) +
         ((int32_t)m[0][2] *
          (((int32_t)m[1][0] * m[2][1]) - ((int32_t)m[1][1] * m[2][0])));
}

/* All 48 signed permutations: six axis permutations times eight sign
 * combinations. 24 are proper rotations, 24 are reflections; both are valid
 * sensor -> body maps because some ST sensor triads are left-handed. */
static void test_all_orientations(void)
{
  static const uint8_t perms[6][3] = {
      {0, 1, 2}, {0, 2, 1}, {1, 0, 2}, {1, 2, 0}, {2, 0, 1}, {2, 1, 0}};
  const vec3_t in = {1.5f, -2.25f, 9.81f};
  int proper = 0;
  int improper = 0;

  for (uint8_t p = 0u; p < 6u; p++)
  {
    for (uint8_t signs = 0u; signs < 8u; signs++)
    {
      int8_t m[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
      vec3_t out;
      float expected[3];

      for (uint8_t r = 0u; r < 3u; r++)
      {
        m[r][perms[p][r]] = ((signs >> r) & 1u) != 0u ? -1 : 1;
      }
      CHECK(axis_map_is_signed_permutation(m));
      if (det3(m) == 1)
      {
        proper++;
      }
      else
      {
        improper++;
      }

      for (uint8_t r = 0u; r < 3u; r++)
      {
        expected[r] = ((float)m[r][0] * in.x) + ((float)m[r][1] * in.y) +
                      ((float)m[r][2] * in.z);
      }
      axis_map_apply(m, &in, &out);
      CHECK_NEAR_F(out.x, expected[0], 1e-6f);
      CHECK_NEAR_F(out.y, expected[1], 1e-6f);
      CHECK_NEAR_F(out.z, expected[2], 1e-6f);

      /* A signed permutation preserves length and maps axes onto axes. */
      CHECK_NEAR_F((out.x * out.x) + (out.y * out.y) + (out.z * out.z),
                   (in.x * in.x) + (in.y * in.y) + (in.z * in.z), 1e-4f);
    }
  }
  CHECK(proper == 24);
  CHECK(improper == 24);
}

static void test_rejects_invalid(void)
{
  static const int8_t zero_row[3][3] = {{1, 0, 0}, {0, 0, 0}, {0, 0, 1}};
  static const int8_t duplicate_col[3][3] = {{1, 0, 0}, {0, 0, 1}, {0, 0, 1}};
  static const int8_t bad_value[3][3] = {{2, 0, 0}, {0, 1, 0}, {0, 0, 1}};

  CHECK(!axis_map_is_signed_permutation(zero_row));
  CHECK(!axis_map_is_signed_permutation(duplicate_col));
  CHECK(!axis_map_is_signed_permutation(bad_value));
  CHECK(!axis_map_is_signed_permutation(NULL));
}

static void test_known_rotation(void)
{
  /* +90 degrees about body Z: x' = -y, y' = x, z' = z. */
  static const int8_t rot[3][3] = {{0, -1, 0}, {1, 0, 0}, {0, 0, 1}};
  const vec3_t in = {1.5f, -2.25f, 9.81f};
  vec3_t out;

  CHECK(axis_map_is_signed_permutation(rot));
  axis_map_apply(rot, &in, &out);
  CHECK_NEAR_F(out.x, 2.25f, 1e-6f);
  CHECK_NEAR_F(out.y, 1.5f, 1e-6f);
  CHECK_NEAR_F(out.z, 9.81f, 1e-6f);

  /* In-place: out aliases in. */
  vec3_t v = in;
  axis_map_apply(rot, &v, &v);
  CHECK_NEAR_F(v.x, 2.25f, 1e-6f);
  CHECK_NEAR_F(v.y, 1.5f, 1e-6f);
  CHECK_NEAR_F(v.z, 9.81f, 1e-6f);
}

static void test_board_maps(void)
{
  CHECK(axis_map_is_signed_permutation(board_axis_accel));
  CHECK(axis_map_is_signed_permutation(board_axis_gyro));
  CHECK(axis_map_is_signed_permutation(board_axis_mag));
}

void test_axis_map(void)
{
  static const int8_t ident[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
  const vec3_t in = {1.5f, -2.25f, 9.81f};
  vec3_t out;

  axis_map_apply(ident, &in, &out);
  CHECK_NEAR_F(out.x, 1.5f, 1e-6f);
  CHECK_NEAR_F(out.y, -2.25f, 1e-6f);
  CHECK_NEAR_F(out.z, 9.81f, 1e-6f);

  test_known_rotation();
  test_all_orientations();
  test_rejects_invalid();
  test_board_maps();
}
