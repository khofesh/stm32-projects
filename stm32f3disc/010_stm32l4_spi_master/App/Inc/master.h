#ifndef __MASTER_H
#define __MASTER_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>

/* T-16 SPI master for the 010_9dof_bmp280_cmake SPI2 slave (SPEC §23.6).
 * SPI1 PA1/PA11/PA12 at 4 MHz, NSS PA4, DRDY PB0 (both edges), PROBE PB1.
 * USART2 (VCP) out: every valid slave data frame verbatim + a 1 Hz stats
 * frame (type 0x10). USART2 in: 'p' 100 partial/full pairs, 'f' full only,
 * 'r' reset stats. */

/* Order of the u32 fields in the stats frame; tools/t16_compare.py mirrors it. */
typedef enum
{
  ST_DRDY = 0,     /* DRDY edges */
  ST_MISSED_DRDY,  /* edges not served (previous read still busy) */
  ST_READS,        /* full 84-byte reads */
  ST_OK,
  ST_ZERO,
  ST_BAD_HDR,
  ST_BAD_CRC,
  ST_GAPS,         /* missing sequence numbers */
  ST_DUPS,
  ST_HAL_ERR,
  ST_PAIR_DONE,
  ST_PAIR_FAIL,    /* full read after a 40-byte read invalid or prefix differs */
  ST_UART_DROP,
  ST_MAX_LAT_US,   /* DRDY edge -> NSS low */
  ST_MAX_XFER_US,  /* NSS low -> NSS high, full read */
  ST_LAST_SEQ,
  ST_PAIRS_LEFT,
  ST_COUNT
} master_stat_t;

void master_init(void);
void master_poll(void);

#ifdef __cplusplus
}
#endif

#endif /* __MASTER_H */
