#ifndef __BARO_FILTER_H
#define __BARO_FILTER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Alpha-beta (constant-velocity) filter for the relative altitude. */
void  baro_filter_init(float alpha, float beta, float dt);
float baro_filter_update(float alt_m);

#ifdef __cplusplus
}
#endif

#endif /* __BARO_FILTER_H */
