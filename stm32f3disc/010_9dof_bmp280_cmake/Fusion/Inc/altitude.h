#ifndef __ALTITUDE_H
#define __ALTITUDE_H

#ifdef __cplusplus
extern "C"
{
#endif

/* ISA sea-level pressure, used until the baro reference 'B' is captured. */
#define ALTITUDE_SEA_LEVEL_PA 101325.0f

/* 44330 * (1 - (p / p0)^(1/5.255)), SPEC §21.4. Returns 0 for invalid
 * pressures. */
float altitude_from_pressure(float pa, float p0_pa);

#ifdef __cplusplus
}
#endif

#endif /* __ALTITUDE_H */
