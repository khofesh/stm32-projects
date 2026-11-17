#include "master.h"

#include <string.h>

#include "main.h"
#include "frame_check.h"

extern SPI_HandleTypeDef hspi1;
extern UART_HandleTypeDef huart2;

#define PART_LEN 40u
#define PAIRS 100u
#define GAP_US 30u /* slave needs >= 20 us NSS high (NFR-17) */
#define STATS_MS 1000u
#define TXQ 4u
#define LED_EVERY 50u

typedef enum
{
  X_IDLE = 0,
  X_BUSY,
  X_DONE,
  X_ERR
} xfer_t;

static uint8_t s_tx_zero[FC_FRAME_LEN];
static uint8_t s_rx[FC_FRAME_LEN];
static uint8_t s_part[PART_LEN];

static uint8_t s_q[TXQ][FC_FRAME_LEN];
static uint8_t s_qh;
static uint8_t s_qt;
static uint8_t s_sending;
static volatile uint8_t s_uart_busy;
static uint8_t s_cmd_rx;
static volatile uint8_t s_cmd;

static volatile uint32_t s_drdy_cnt;
static volatile uint32_t s_drdy_cyc;
static uint32_t s_drdy_seen;

static volatile xfer_t s_x;
static volatile uint32_t s_nss_high_cyc;
static uint32_t s_nss_low_cyc;
static uint16_t s_len;
static uint8_t s_pending; /* a DRDY edge (or the full half of a pair) to serve */
static uint8_t s_step;    /* pair: 0 = 40-byte read next, 1 = full read next */
static uint32_t s_pairs_left;

static uint32_t s_st[ST_COUNT];
static fc_seq_t s_seq;
static uint16_t s_stats_seq;
static uint32_t s_stats_ms;
static uint32_t s_cyc_us;
static uint32_t s_led;

static uint32_t cyc_to_us(uint32_t c)
{
  return c / s_cyc_us;
}

static void nss_high(void)
{
  HAL_GPIO_WritePin(SPI_NSS_GPIO_Port, SPI_NSS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(PROBE_GPIO_Port, PROBE_Pin, GPIO_PIN_RESET);
  s_nss_high_cyc = DWT->CYCCNT;
}

static void enqueue(const uint8_t *f)
{
  uint8_t next = (uint8_t)((s_qh + 1u) % TXQ);

  if (next == s_qt)
  {
    s_st[ST_UART_DROP]++;
    return;
  }
  memcpy(s_q[s_qh], f, FC_FRAME_LEN);
  s_qh = next;
}

static void start(void)
{
  uint16_t len = ((s_step == 0u) && (s_pairs_left != 0u)) ? (uint16_t)PART_LEN : (uint16_t)FC_FRAME_LEN;
  uint8_t *rx = (len == PART_LEN) ? s_part : s_rx;
  uint32_t lat;

  s_len = len;
  s_x = X_BUSY;
  HAL_GPIO_WritePin(PROBE_GPIO_Port, PROBE_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(SPI_NSS_GPIO_Port, SPI_NSS_Pin, GPIO_PIN_RESET);
  s_nss_low_cyc = DWT->CYCCNT;
  if (s_step == 0u)
  {
    lat = cyc_to_us(s_nss_low_cyc - s_drdy_cyc);
    if (lat > s_st[ST_MAX_LAT_US])
    {
      s_st[ST_MAX_LAT_US] = lat;
    }
  }
  if (HAL_SPI_TransmitReceive_DMA(&hspi1, s_tx_zero, rx, len) != HAL_OK)
  {
    nss_high();
    s_x = X_IDLE;
    s_st[ST_HAL_ERR]++;
    s_pending = 0u;
    s_step = 0u;
  }
}

static void finish_full(void)
{
  fc_result_t r = fc_check(s_rx, FC_TYPE_DATA);
  uint32_t us = cyc_to_us(s_nss_high_cyc - s_nss_low_cyc);

  if (us > s_st[ST_MAX_XFER_US])
  {
    s_st[ST_MAX_XFER_US] = us;
  }
  s_st[ST_READS]++;
  switch (r)
  {
    case FC_OK:
      s_st[ST_OK]++;
      s_st[ST_LAST_SEQ] = fc_seq(s_rx);
      fc_seq_track(&s_seq, fc_seq(s_rx), &s_st[ST_GAPS], &s_st[ST_DUPS]);
      enqueue(s_rx);
      if (++s_led >= LED_EVERY)
      {
        s_led = 0u;
        BSP_LED_Toggle(LED_GREEN);
      }
      break;
    case FC_ZERO:
      s_st[ST_ZERO]++;
      break;
    case FC_BAD_HDR:
      s_st[ST_BAD_HDR]++;
      break;
    default:
      s_st[ST_BAD_CRC]++;
      break;
  }

  if (s_step == 1u)
  {
    s_step = 0u;
    s_st[ST_PAIR_DONE]++;
    if ((r != FC_OK) || (memcmp(s_part, s_rx, PART_LEN) != 0))
    {
      s_st[ST_PAIR_FAIL]++;
    }
    if (s_pairs_left != 0u)
    {
      s_pairs_left--;
    }
  }
}

static void finish(void)
{
  xfer_t x = s_x;

  s_x = X_IDLE;
  if (x == X_ERR)
  {
    s_st[ST_HAL_ERR]++;
    s_pending = 0u;
    s_step = 0u;
    return;
  }
  if (s_len == PART_LEN)
  {
    s_step = 1u; /* s_pending stays set: full read follows */
    return;
  }
  s_pending = 0u;
  finish_full();
}

static void drdy_service(void)
{
  uint32_t n = s_drdy_cnt;
  uint32_t d = n - s_drdy_seen;

  if (d == 0u)
  {
    return;
  }
  s_drdy_seen = n;
  s_st[ST_DRDY] += d;
  if ((s_pending != 0u) || (s_x != X_IDLE))
  {
    s_st[ST_MISSED_DRDY] += d;
    return;
  }
  s_st[ST_MISSED_DRDY] += d - 1u;
  s_pending = 1u;
  s_step = 0u;
}

static void uart_service(void)
{
  if ((s_sending != 0u) && (s_uart_busy == 0u))
  {
    s_qt = (uint8_t)((s_qt + 1u) % TXQ);
    s_sending = 0u;
  }
  if ((s_sending == 0u) && (s_qt != s_qh))
  {
    s_uart_busy = 1u;
    s_sending = 1u;
    if (HAL_UART_Transmit_IT(&huart2, s_q[s_qt], (uint16_t)FC_FRAME_LEN) != HAL_OK)
    {
      s_uart_busy = 0u;
      s_sending = 0u;
      s_qt = (uint8_t)((s_qt + 1u) % TXQ);
      s_st[ST_UART_DROP]++;
    }
  }
}

static void cmd_service(void)
{
  uint8_t c = s_cmd;

  /* RX is re-armed here, never in the callback; a failed arm retries next pass. */
  if (huart2.RxState == HAL_UART_STATE_READY)
  {
    (void)HAL_UART_Receive_IT(&huart2, &s_cmd_rx, 1u);
  }
  if (c == 0u)
  {
    return;
  }
  s_cmd = 0u;
  switch (c)
  {
    case 'p':
      s_pairs_left = PAIRS;
      break;
    case 'f':
      s_pairs_left = 0u;
      break;
    case 'r':
      memset(s_st, 0, sizeof(s_st));
      s_seq.valid = 0u;
      break;
    default:
      break;
  }
}

static void stats_service(void)
{
  uint8_t f[FC_FRAME_LEN];

  if ((HAL_GetTick() - s_stats_ms) < STATS_MS)
  {
    return;
  }
  s_stats_ms += STATS_MS;
  s_st[ST_PAIRS_LEFT] = s_pairs_left;
  fc_stats_pack(f, s_stats_seq++, HAL_GetTick(), s_st, (uint8_t)ST_COUNT);
  enqueue(f);
}

void master_init(void)
{
  _Static_assert(ST_COUNT <= FC_STATS_MAX, "stats frame overflow");

  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0u;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  s_cyc_us = SystemCoreClock / 1000000u;

  nss_high();
  s_drdy_seen = s_drdy_cnt;
  s_stats_ms = HAL_GetTick();
}

void master_poll(void)
{
  xfer_t x = s_x;

  if ((x == X_DONE) || (x == X_ERR))
  {
    finish();
  }
  drdy_service();
  if ((s_pending != 0u) && (s_x == X_IDLE) && ((DWT->CYCCNT - s_nss_high_cyc) >= (GAP_US * s_cyc_us)))
  {
    start();
  }
  uart_service();
  cmd_service();
  stats_service();
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == SPI_DRDY_Pin)
  {
    s_drdy_cyc = DWT->CYCCNT;
    s_drdy_cnt++;
  }
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi == &hspi1)
  {
    nss_high();
    s_x = X_DONE;
  }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi == &hspi1)
  {
    nss_high();
    s_x = X_ERR;
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart2)
  {
    s_uart_busy = 0u;
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart2)
  {
    s_cmd = s_cmd_rx;
  }
}
