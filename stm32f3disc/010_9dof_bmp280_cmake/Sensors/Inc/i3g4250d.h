#ifndef __I3G4250D_H
#define __I3G4250D_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>

#include "vec3.h"

/* I3G4250D gyroscope on SPI1 (CS PE3). */

#define I3G4250D_WHO_AM_I_VALUE 0xD3u

#define I3G4250D_REG_WHO_AM_I 0x0Fu
#define I3G4250D_REG_CTRL1 0x20u
#define I3G4250D_REG_CTRL4 0x23u
#define I3G4250D_REG_OUT_X_L 0x28u

/* +-500 dps, 17.5 mdps/LSB -> rad/s. */
#define I3G4250D_DPS_TO_RAD_S 0.017453293f
#define I3G4250D_DPS_PER_LSB 17.5e-3f

int32_t i3g4250d_init(void);

/* Angular rate in rad/s, sensor frame. */
int32_t i3g4250d_read(vec3_t *rad_s);

/* WHO_AM_I as read by the last init, 0 before that (T-02). */
uint8_t i3g4250d_id(void);

#ifdef __cplusplus
}
#endif

#endif /* __I3G4250D_H */
