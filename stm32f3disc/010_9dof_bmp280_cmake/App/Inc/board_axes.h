#ifndef __BOARD_AXES_H
#define __BOARD_AXES_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "vec3.h"

/* Sensor -> body (FRD: X forward, Y right, Z down) signed permutations for
 * the STM32F3DISCOVERY, measured by T-03 (docs/hardware.md). */
extern const int8_t board_axis_accel[3][3];
extern const int8_t board_axis_gyro[3][3];
extern const int8_t board_axis_mag[3][3];

void board_axes_apply_accel(vec3_t *v);
void board_axes_apply_gyro(vec3_t *v);
void board_axes_apply_mag(vec3_t *v);

#ifdef __cplusplus
}
#endif

#endif /* __BOARD_AXES_H */
