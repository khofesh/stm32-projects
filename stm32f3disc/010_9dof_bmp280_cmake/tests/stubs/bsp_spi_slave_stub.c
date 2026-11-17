#include <string.h>

#include "bsp_spi_slave.h"
#include "bsp_spi_slave_stub.h"

uint8_t stub_spi_pub[128];
uint16_t stub_spi_pub_len;
unsigned stub_spi_pub_calls;

void bsp_spi_slave_publish(const uint8_t *frame)
{
  stub_spi_pub_calls++;
  if (frame == NULL)
  {
    stub_spi_pub_len = 0u;
    return;
  }
  memcpy(stub_spi_pub, frame, BSP_SPI_SLAVE_FRAME_LEN);
  stub_spi_pub_len = BSP_SPI_SLAVE_FRAME_LEN;
}
