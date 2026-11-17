#include <errno.h>
#include <string.h>

#include "bsp_uart.h"

/* Capture buffer for the telemetry tests. */
uint8_t stub_tx[128];
uint16_t stub_tx_len;
int stub_tx_busy;
int stub_tx_fail;
unsigned stub_tx_calls;

uint8_t bsp_uart_tx_ready(void)
{
  return (stub_tx_busy == 0) ? 1u : 0u;
}

int32_t bsp_uart_send_dma(const uint8_t *buf, uint16_t len)
{
  stub_tx_calls++;
  if (stub_tx_busy != 0)
  {
    return -EBUSY;
  }
  if (stub_tx_fail != 0)
  {
    return -EIO;
  }
  if (len > sizeof(stub_tx))
  {
    len = (uint16_t)sizeof(stub_tx);
  }
  memcpy(stub_tx, buf, len);
  stub_tx_len = len;
  return 0;
}
