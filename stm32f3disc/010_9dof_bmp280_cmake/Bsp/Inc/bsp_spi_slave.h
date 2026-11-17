#ifndef __BSP_SPI_SLAVE_H
#define __BSP_SPI_SLAVE_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>

/* SPI2 slave link for an external master (SPEC §23.6): PB13 SCK, PB14 MISO,
 * PB15 MOSI, PB12 NSS as EXTI rising edge, PB11 DRDY output. Software NSS. */

#define BSP_SPI_SLAVE_FRAME_LEN 84u

/* Arm the all-zero frame. Call once after MX_SPI2_Init(). */
void bsp_spi_slave_init(void);

/* Copy a frame into the free buffer, request a re-arm and toggle DRDY.
 * Main loop only. */
void bsp_spi_slave_publish(const uint8_t *frame);

/* From HAL_GPIO_EXTI_Callback(PB12): NSS rising edge or a publish request. */
void bsp_spi_slave_on_exti(void);

/* Full 84-byte and partial transactions, cumulative, wrapping. */
uint32_t bsp_spi_slave_served(void);
uint32_t bsp_spi_slave_partial(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_SPI_SLAVE_H */
