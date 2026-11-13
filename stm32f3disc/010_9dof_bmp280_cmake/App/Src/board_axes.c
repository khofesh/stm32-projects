#include "board_axes.h"

#include "axis_map.h"

/* STM32F3DISCOVERY (MB1035-F303C-E02) sensor -> body frame (FRD).
 * Measured by T-03 on 2026-11-13; see docs/hardware.md for the poses.
 *
 * The accelerometer triad is left-handed (det = -1), so these maps are
 * signed permutations but not rotations. The gyro is assumed to share the
 * same package orientation; its signs are checked by the yaw test in T-03
 * and during M4 tuning. */
const int8_t board_axis_accel[3][3] = {{0, 1, 0}, {-1, 0, 0}, {0, 0, -1}};
const int8_t board_axis_gyro[3][3] = {{0, 1, 0}, {-1, 0, 0}, {0, 0, -1}};
const int8_t board_axis_mag[3][3] = {{0, 1, 0}, {-1, 0, 0}, {0, 0, -1}};

void board_axes_apply_accel(vec3_t *v)
{
  axis_map_apply(board_axis_accel, v, v);
}

void board_axes_apply_gyro(vec3_t *v)
{
  axis_map_apply(board_axis_gyro, v, v);
}

void board_axes_apply_mag(vec3_t *v)
{
  axis_map_apply(board_axis_mag, v, v);
}
