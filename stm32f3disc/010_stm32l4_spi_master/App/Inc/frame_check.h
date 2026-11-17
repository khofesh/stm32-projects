#ifndef __FRAME_CHECK_H
#define __FRAME_CHECK_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>

/* Slave frame format: 010_9dof_bmp280_cmake docs/SPEC.md §23.2/§23.3. */
#define FC_FRAME_LEN 84u
#define FC_PAYLOAD_LEN 78u
#define FC_SYNC0 0xAAu
#define FC_SYNC1 0x55u
#define FC_TYPE_DATA 0x01u
#define FC_TYPE_STATS 0x10u

#define FC_OFF_SEQ 4u
#define FC_OFF_TIME 6u
#define FC_OFF_BODY 10u
#define FC_OFF_CRC 82u

/* u32 fields that fit between FC_OFF_BODY and FC_OFF_CRC. */
#define FC_STATS_MAX 18u

typedef enum
{
  FC_OK = 0,
  FC_ZERO,    /* 84 zero bytes: slave has not published yet */
  FC_BAD_HDR, /* sync, type or length wrong */
  FC_BAD_CRC
} fc_result_t;

typedef struct
{
  uint8_t valid;
  uint16_t last;
} fc_seq_t;

/* CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection, no xorout. */
uint16_t crc16_ccitt(const uint8_t *data, uint32_t len);

fc_result_t fc_check(const uint8_t f[FC_FRAME_LEN], uint8_t type);
uint16_t fc_seq(const uint8_t f[FC_FRAME_LEN]);

/* Feed the sequence of a valid frame; adds missing frames to *gaps and a
 * repeated sequence to *dups. */
void fc_seq_track(fc_seq_t *s, uint16_t seq, uint32_t *gaps, uint32_t *dups);

/* Stats frame: header, seq, t_ms, n (<= FC_STATS_MAX) u32 values, zero pad, CRC. */
void fc_stats_pack(uint8_t out[FC_FRAME_LEN], uint16_t seq, uint32_t t_ms, const uint32_t *v, uint8_t n);

#ifdef __cplusplus
}
#endif

#endif /* __FRAME_CHECK_H */
