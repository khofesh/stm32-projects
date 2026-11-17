#ifndef __BSP_SPI_H
#define __BSP_SPI_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>

/* I3G4250D on SPI1 (mode 3), chip select PE3 driven by software.
 * Read bit 0x80, address auto-increment bit 0x40. */

int32_t bsp_spi_read(uint8_t reg, uint8_t *buf, uint16_t len);
int32_t bsp_spi_write(uint8_t reg, const uint8_t *buf, uint16_t len);

/* Failed transfers since reset. */
uint32_t bsp_spi_errors(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_SPI_H */
