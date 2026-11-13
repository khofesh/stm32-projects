#ifndef __CMD_H
#define __CMD_H

#ifdef __cplusplus
extern "C" {
#endif

/* Consume every byte waiting in the USART1 RX ring (SPEC §22.4). Unknown
 * bytes are ignored. Called from the main loop. */
void cmd_poll(void);

#ifdef __cplusplus
}
#endif

#endif /* __CMD_H */
