#ifndef __BMP280_REGS_STUB_H
#define __BMP280_REGS_STUB_H

#include "driver_bmp280.h"

/* Register file served by the LibDriver I2C hooks in bmp280_regs_stub.c. */
extern uint8_t stub_regs[256];

void bmp280_stub_link(bmp280_handle_t *h);
void bmp280_stub_calibration(const uint16_t t1, const uint16_t t2, const uint16_t t3,
                             const uint16_t p1, const uint16_t p2, const uint16_t p3,
                             const uint16_t p4, const uint16_t p5, const uint16_t p6,
                             const uint16_t p7, const uint16_t p8, const uint16_t p9);

#endif /* __BMP280_REGS_STUB_H */
