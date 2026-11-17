#ifndef __APP_H
#define __APP_H

#ifdef __cplusplus
extern "C"
{
#endif

/* Initialise the application; returns normally. */
void app_init(void);

/* Run the cooperative scheduler; never returns. */
void app_run(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_H */
