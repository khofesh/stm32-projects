#ifndef __CRC16_H
#define __CRC16_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>

/* CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection, no xorout. */
uint16_t crc16_ccitt(const uint8_t *data, uint32_t len);

#ifdef __cplusplus
}
#endif

#endif /* __CRC16_H */
