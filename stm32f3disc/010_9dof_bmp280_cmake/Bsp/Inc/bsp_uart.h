#ifndef __BSP_UART_H
#define __BSP_UART_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>

/* USART1 (ST-LINK VCP): DMA TX, interrupt RX into a 64-byte ring. */

int32_t bsp_uart_init(void);
int32_t bsp_uart_send_dma(const uint8_t *buf, uint16_t len);
uint8_t bsp_uart_tx_ready(void);
uint16_t bsp_uart_rx_available(void);
int32_t bsp_uart_rx_pop(uint8_t *byte);
uint32_t bsp_uart_rx_drops(void);
uint32_t bsp_uart_rx_arm_fails(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_UART_H */
