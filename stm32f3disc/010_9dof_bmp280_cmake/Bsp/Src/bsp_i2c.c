#include "bsp_i2c.h"

#include <errno.h>
#include <stddef.h>

#include "main.h"

extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;

#define I2C_TIMEOUT_MS 10u
#define I2C_BUS_COUNT 2u

typedef struct
{
  I2C_HandleTypeDef *h;
  uint32_t errors;
} i2c_bus_t;

static i2c_bus_t s_bus[I2C_BUS_COUNT] = {
  {&hi2c1, 0u},
  {&hi2c2, 0u},
};

static i2c_bus_t *bus_get(uint8_t bus)
{
  if ((bus < 1u) || (bus > I2C_BUS_COUNT))
  {
    return NULL;
  }
  return &s_bus[bus - 1u];
}

static void bus_fail(i2c_bus_t *b)
{
  b->errors++;
  (void)HAL_I2C_DeInit(b->h);
  (void)HAL_I2C_Init(b->h);
}

int32_t bsp_i2c_read(uint8_t bus, uint8_t addr7, uint8_t reg, uint8_t *buf, uint16_t len)
{
  i2c_bus_t *b = bus_get(bus);

  if ((b == NULL) || (buf == NULL) || (len == 0u))
  {
    return -EINVAL;
  }
  if (HAL_I2C_Mem_Read(b->h, (uint16_t)addr7 << 1, reg, I2C_MEMADD_SIZE_8BIT, buf, len,
                       I2C_TIMEOUT_MS) != HAL_OK)
  {
    bus_fail(b);
    return -EIO;
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
  if (HAL_I2C_Mem_Write(b->h, (uint16_t)addr7 << 1, reg, I2C_MEMADD_SIZE_8BIT, (uint8_t *)buf, len,
                        I2C_TIMEOUT_MS) != HAL_OK)
  {
    bus_fail(b);
    return -EIO;
  }
  return 0;
}

int32_t bsp_i2c_recover(uint8_t bus)
{
  i2c_bus_t *b = bus_get(bus);

  if (b == NULL)
  {
    return -EINVAL;
  }
  (void)HAL_I2C_DeInit(b->h);
  if (HAL_I2C_Init(b->h) != HAL_OK)
  {
    return -EIO;
  }
  return 0;
}

uint32_t bsp_i2c_errors(uint8_t bus)
{
  i2c_bus_t *b = bus_get(bus);

  return (b == NULL) ? 0u : b->errors;
}
