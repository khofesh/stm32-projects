#include "bsp_i2c.h"

#include <errno.h>
#include <stddef.h>

#include "bsp_time.h"
#include "main.h"

extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;

#define I2C_TIMEOUT_MS 10u
#define I2C_BUS_COUNT 2u

/* Bus clear: 9 SCL pulses at ~100 kHz, then a STOP (UM10204 §3.1.16). */
#define CLEAR_PULSES 9u
#define CLEAR_HALF_US 5u

typedef struct
{
  I2C_HandleTypeDef *h;
  GPIO_TypeDef *port;
  uint16_t scl;
  uint16_t sda;
  uint32_t errors;
} i2c_bus_t;

static i2c_bus_t s_bus[I2C_BUS_COUNT] = {
    {&hi2c1, GPIOB, GPIO_PIN_6, GPIO_PIN_7, 0u},
    {&hi2c2, GPIOA, GPIO_PIN_9, GPIO_PIN_10, 0u},
};

static i2c_bus_t *bus_get(uint8_t bus)
{
  if ((bus < 1u) || (bus > I2C_BUS_COUNT))
  {
    return NULL;
  }
  return &s_bus[bus - 1u];
}

static void wait_us(uint32_t us)
{
  uint32_t t0 = bsp_time_us();

  while ((bsp_time_us() - t0) < us)
  {
  }
}

/* Runs with the peripheral de-initialised; MspInit restores the AF pins. */
static void bus_clear(const i2c_bus_t *b)
{
  GPIO_InitTypeDef g = {0};

  HAL_GPIO_WritePin(b->port, b->scl | b->sda, GPIO_PIN_SET);
  g.Pin = b->scl | b->sda;
  g.Mode = GPIO_MODE_OUTPUT_OD;
  g.Pull = GPIO_PULLUP;
  g.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(b->port, &g);

  for (uint8_t i = 0u; i < CLEAR_PULSES; i++)
  {
    if (HAL_GPIO_ReadPin(b->port, b->sda) == GPIO_PIN_SET)
    {
      break;
    }
    HAL_GPIO_WritePin(b->port, b->scl, GPIO_PIN_RESET);
    wait_us(CLEAR_HALF_US);
    HAL_GPIO_WritePin(b->port, b->scl, GPIO_PIN_SET);
    wait_us(CLEAR_HALF_US);
  }

  /* STOP: SDA low -> high while SCL is high. */
  HAL_GPIO_WritePin(b->port, b->scl, GPIO_PIN_RESET);
  wait_us(CLEAR_HALF_US);
  HAL_GPIO_WritePin(b->port, b->sda, GPIO_PIN_RESET);
  wait_us(CLEAR_HALF_US);
  HAL_GPIO_WritePin(b->port, b->scl, GPIO_PIN_SET);
  wait_us(CLEAR_HALF_US);
  HAL_GPIO_WritePin(b->port, b->sda, GPIO_PIN_SET);
  wait_us(CLEAR_HALF_US);
}

static int32_t recover(const i2c_bus_t *b)
{
  (void)HAL_I2C_DeInit(b->h);
  bus_clear(b);
  return (HAL_I2C_Init(b->h) == HAL_OK) ? 0 : -EIO;
}

/* A NACK leaves the peripheral idle (HAL already sent STOP); anything else
 * (bus error, arbitration lost, timeout) may leave the bus or HAL stuck. */
static int32_t bus_fail(i2c_bus_t *b)
{
  uint32_t err = HAL_I2C_GetError(b->h);

  b->errors++;
  if (err != HAL_I2C_ERROR_AF)
  {
    (void)recover(b);
  }
  return -EIO;
}

int32_t bsp_i2c_read(uint8_t bus, uint8_t addr7, uint8_t reg, uint8_t *buf, uint16_t len)
{
  i2c_bus_t *b = bus_get(bus);

  if ((b == NULL) || (buf == NULL) || (len == 0u))
  {
    return -EINVAL;
  }
  if (HAL_I2C_Mem_Read(b->h, (uint16_t)addr7 << 1, reg, I2C_MEMADD_SIZE_8BIT, buf, len, I2C_TIMEOUT_MS) != HAL_OK)
  {
    return bus_fail(b);
  }
  return 0;
}

int32_t bsp_i2c_write(uint8_t bus, uint8_t addr7, uint8_t reg, const uint8_t *buf, uint16_t len)
{
  i2c_bus_t *b = bus_get(bus);

  if ((b == NULL) || (buf == NULL) || (len == 0u))
  {
    return -EINVAL;
  }
  /* HAL takes a non-const pointer but only reads from it. */
  if (HAL_I2C_Mem_Write(b->h, (uint16_t)addr7 << 1, reg, I2C_MEMADD_SIZE_8BIT, (uint8_t *)buf, len, I2C_TIMEOUT_MS) !=
      HAL_OK)
  {
    return bus_fail(b);
  }
  return 0;
}

int32_t bsp_i2c_recover(uint8_t bus)
{
  const i2c_bus_t *b = bus_get(bus);

  if (b == NULL)
  {
    return -EINVAL;
  }
  return recover(b);
}

uint32_t bsp_i2c_errors(uint8_t bus)
{
  const i2c_bus_t *b = bus_get(bus);

  return (b == NULL) ? 0u : b->errors;
}
