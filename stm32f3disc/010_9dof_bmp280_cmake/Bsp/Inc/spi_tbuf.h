#ifndef __SPI_TBUF_H
#define __SPI_TBUF_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>

/* Triple buffer for the SPI2 slave (SPEC §23.6.3): one buffer is armed for
 * the DMA, one holds the latest published frame, one is free. The main loop
 * fills the free buffer and publishes; the EXTI ISR arms the published one.
 * The armed buffer is never written and the published buffer is never torn. */

#define SPI_TBUF_COUNT 3u

typedef struct
{
  uint8_t write_idx;          /* free buffer, main loop only */
  volatile uint8_t pub_idx;   /* latest complete frame, read by the ISR */
  volatile uint8_t armed_idx; /* buffer the DMA reads, ISR only */
} spi_tbuf_t;

/* The first buffer is armed with the all-zero frame. */
void spi_tbuf_init(spi_tbuf_t *t);

/* Index of the buffer to fill; never the armed or the published one. */
uint8_t spi_tbuf_next_free(const spi_tbuf_t *t);

/* Make the buffer from spi_tbuf_next_free() the latest frame and pick a new
 * free one. Call after the frame has been written. */
void spi_tbuf_publish(spi_tbuf_t *t);

/* Make the published frame the armed one and return its index. ISR context. */
uint8_t spi_tbuf_arm(spi_tbuf_t *t);

#ifdef __cplusplus
}
#endif

#endif /* __SPI_TBUF_H */
