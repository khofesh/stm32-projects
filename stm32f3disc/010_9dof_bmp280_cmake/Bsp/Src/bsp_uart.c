#include "bsp_uart.h"

#include <errno.h>
#include <stddef.h>

#include "main.h"

extern UART_HandleTypeDef huart1;

#define RX_RING_SIZE 64u
#define RX_RING_MASK (RX_RING_SIZE - 1u)

static uint8_t s_rx_byte;
static uint8_t s_rx_ring[RX_RING_SIZE];
static volatile uint16_t s_rx_head; /* written by the ISR */
static volatile uint16_t s_rx_tail; /* written by the consumer */
static volatile uint32_t s_rx_drops;
static volatile uint32_t s_rx_arm_fails;

static HAL_StatusTypeDef rx_arm(void)
{
  HAL_StatusTypeDef st = HAL_UART_Receive_IT(&huart1, &s_rx_byte, 1u);

  if (st != HAL_OK)
  {
    s_rx_arm_fails++;
  }
  return st;
}

int32_t bsp_uart_init(void)
{
  s_rx_head = 0u;
  s_rx_tail = 0u;
  s_rx_drops = 0u;
  s_rx_arm_fails = 0u;

  if (rx_arm() != HAL_OK)
  {
    return -EIO;
  }
  return 0;
}

int32_t bsp_uart_send_dma(const uint8_t *buf, uint16_t len)
{
  if ((buf == NULL) || (len == 0u))
  {
    return -EINVAL;
  }
  if (huart1.gState != HAL_UART_STATE_READY)
  {
    return -EBUSY;
  }
  /* HAL takes a non-const pointer but only reads from it. */
  if (HAL_UART_Transmit_DMA(&huart1, (uint8_t *)buf, len) != HAL_OK)
  {
    return -EIO;
  }
  return 0;
}

uint8_t bsp_uart_tx_ready(void)
{
  return (huart1.gState == HAL_UART_STATE_READY) ? 1u : 0u;
}

uint16_t bsp_uart_rx_available(void)
{
  return (uint16_t)((s_rx_head - s_rx_tail) & RX_RING_MASK);
}

int32_t bsp_uart_rx_pop(uint8_t *byte)
{
  if (byte == NULL)
  {
    return -EINVAL;
  }
  if (s_rx_head == s_rx_tail)
  {
    return -EAGAIN;
  }
  *byte = s_rx_ring[s_rx_tail];
  s_rx_tail = (uint16_t)((s_rx_tail + 1u) & RX_RING_MASK);
  return 0;
}

uint32_t bsp_uart_rx_drops(void)
{
  return s_rx_drops;
}

uint32_t bsp_uart_rx_arm_fails(void)
{
  return s_rx_arm_fails;
}

// cppcheck-suppress constParameterPointer
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance != USART1)
  {
    return;
  }

  uint16_t next = (uint16_t)((s_rx_head + 1u) & RX_RING_MASK);
  if (next == s_rx_tail)
  {
    s_rx_drops++;
  }
  else
  {
    s_rx_ring[s_rx_head] = s_rx_byte;
    s_rx_head = next;
  }
  (void)rx_arm(); /* failure counted in s_rx_arm_fails */
}

// cppcheck-suppress constParameterPointer
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  /* Overrun/framing error leaves the receiver disabled; re-arm it unless a
   * TX-side error left RX still running. */
  if ((huart->Instance == USART1) && (huart->RxState == HAL_UART_STATE_READY))
  {
    (void)rx_arm(); /* failure counted in s_rx_arm_fails */
  }
}

#ifdef BSP_UART_STDIO
#define PUTCHAR_TIMEOUT_MS 10u

/* Newlib retarget: printf() in Debug builds ends up here. Waits for an
 * in-flight DMA transfer instead of dropping the character. */
int __io_putchar(int ch)
{
  uint8_t c = (uint8_t)ch;
  uint32_t t0 = HAL_GetTick();

  while ((huart1.gState != HAL_UART_STATE_READY) && ((HAL_GetTick() - t0) < PUTCHAR_TIMEOUT_MS))
  {
  }
  if (HAL_UART_Transmit(&huart1, &c, 1u, PUTCHAR_TIMEOUT_MS) != HAL_OK)
  {
    return -1;
  }
  return ch;
}
#endif /* BSP_UART_STDIO */
