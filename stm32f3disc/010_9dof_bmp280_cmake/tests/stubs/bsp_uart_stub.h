#ifndef __BSP_UART_STUB_H
#define __BSP_UART_STUB_H

#include <stdint.h>

/* Capture buffer served by the stub bsp_uart_send_dma(). */
extern uint8_t stub_tx[128];
extern uint16_t stub_tx_len;
extern int stub_tx_busy;
extern unsigned stub_tx_calls;

#endif /* __BSP_UART_STUB_H */
