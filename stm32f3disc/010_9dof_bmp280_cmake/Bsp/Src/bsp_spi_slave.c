#include "bsp_spi_slave.h"

#include <stddef.h>
#include <string.h>

#include "main.h"
#include "spi_tbuf.h"

extern SPI_HandleTypeDef hspi2;
extern DMA_HandleTypeDef hdma_spi2_tx;

static uint8_t s_frame[SPI_TBUF_COUNT][BSP_SPI_SLAVE_FRAME_LEN];
static spi_tbuf_t s_tbuf;
static volatile uint8_t s_rearm_req;
static volatile uint8_t s_ready;
static volatile uint32_t s_served;
static volatile uint32_t s_partial;

static uint8_t nss_idle(void)
{
  return (HAL_GPIO_ReadPin(SPI_NSS_GPIO_Port, SPI_NSS_Pin) == GPIO_PIN_SET) ? 1u : 0u;
}

/* Bytes clocked since the last arm show up as RXNE/OVR (MOSI is never read),
 * whatever triggered the ISR. Full = DMA done and TX FIFO drained. */
static void count_transaction(void)
{
  uint32_t sr = hspi2.Instance->SR;

  if ((sr & (SPI_SR_RXNE | SPI_SR_OVR)) == 0u)
  {
    return;
  }
  if ((hdma_spi2_tx.Instance->CNDTR == 0u) && ((sr & SPI_SR_FTLVL) == 0u))
  {
    s_served++;
  }
  else
  {
    s_partial++;
  }
}

/* SPEC §23.6.3: the F3 TX FIFO keeps stale bytes after a partial read and
 * cannot be flushed, so every transaction starts from a peripheral reset.
 * ISR context only. */
static void slave_arm(void)
{
  uint8_t idx = spi_tbuf_arm(&s_tbuf);

  (void)HAL_DMA_Abort(&hdma_spi2_tx);
  __HAL_RCC_SPI2_FORCE_RESET();
  __HAL_RCC_SPI2_RELEASE_RESET();
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    return;
  }
  if (HAL_DMA_Start(&hdma_spi2_tx, (uint32_t)(uintptr_t)&s_frame[idx][0], (uint32_t)(uintptr_t)&hspi2.Instance->DR,
                    BSP_SPI_SLAVE_FRAME_LEN) != HAL_OK)
  {
    return;
  }
  SET_BIT(hspi2.Instance->CR2, SPI_CR2_TXDMAEN);
  __HAL_SPI_ENABLE(&hspi2);
}

void bsp_spi_slave_init(void)
{
  uint32_t primask;

  memset(s_frame, 0, sizeof(s_frame));
  s_rearm_req = 0u;
  s_served = 0u;
  s_partial = 0u;

  primask = __get_PRIMASK();
  __disable_irq();
  __HAL_GPIO_EXTI_CLEAR_IT(SPI_NSS_Pin);
  spi_tbuf_init(&s_tbuf);
  slave_arm();
  s_ready = 1u;
  __set_PRIMASK(primask);
}

void bsp_spi_slave_publish(const uint8_t *frame)
{
  uint8_t idx;

  if (frame == NULL)
  {
    return;
  }
  idx = spi_tbuf_next_free(&s_tbuf);
  memcpy(s_frame[idx], frame, BSP_SPI_SLAVE_FRAME_LEN);
  spi_tbuf_publish(&s_tbuf);

  /* The ISR hands the frame to the DMA: make the copy visible first. */
  __DMB();
  s_rearm_req = 1u;
  WRITE_REG(EXTI->SWIER, SPI_NSS_Pin);
  /* Take the EXTI before DRDY moves, so the frame is armed first. */
  __DSB();
  __ISB();
  HAL_GPIO_TogglePin(SPI_DRDY_GPIO_Port, SPI_DRDY_Pin);
}

void bsp_spi_slave_on_exti(void)
{
  uint8_t req = s_rearm_req;

  /* PB12 EXTI is live from MX_GPIO_Init, before hspi2/hdma_spi2_tx exist. */
  if (s_ready == 0u)
  {
    return;
  }
  s_rearm_req = 0u;

  /* A publish that lands mid-transaction leaves the re-arm to its NSS edge. */
  if ((req == 0u) || (nss_idle() != 0u))
  {
    count_transaction();
    slave_arm();
  }
}

uint32_t bsp_spi_slave_served(void)
{
  return s_served;
}

uint32_t bsp_spi_slave_partial(void)
{
  return s_partial;
}
