#ifndef SDIO_HOST_REG_H_
#define SDIO_HOST_REG_H_

#include "sdio_config.h"

#define SD_IO_CCCR_FN_ENABLE        0x02
#define SD_IO_CCCR_FN_READY         0x03
#define SD_IO_CCCR_INT_ENABLE       0x04
#define SD_IO_CCCR_BUS_WIDTH        0x07

#define CCCR_BUS_WIDTH_ECSI        (1<<5)

#define SD_IO_CCCR_BLKSIZEL         0x10
#define SD_IO_CCCR_BLKSIZEH         0x11

#define TX_BUFFER_MAX   0x1000
#define TX_BUFFER_MASK  0xFFF

#define ESP_SLAVE_CMD53_END_ADDR    0x1f800

#if TARGET_ESP32

#define ESP32_SLCHOST_BASE          0x3ff55000

#define ESP_SDIO_PKT_LEN            (ESP32_SLCHOST_BASE + 0x60)&0x3FF
#define ESP_SDIO_INT_CLR            (ESP32_SLCHOST_BASE + 0xD4)&0x3FF
#define ESP_SDIO_INT_RAW            (ESP32_SLCHOST_BASE + 0x50)&0x3FF
#define ESP_SDIO_INT_ST             (ESP32_SLCHOST_BASE + 0x58)&0x3FF
#define ESP_SDIO_TOKEN_RDATA        (ESP32_SLCHOST_BASE + 0x44)&0x3FF
#define ESP_SDIO_SEND_OFFSET        16
#define ESP_SDIO_CONF               (ESP32_SLCHOST_BASE + 0x8c)&0x3FF
#define ESP_SDIO_CONF_OFFSET        0

#define RX_BYTE_MAX                 0x100000
#define RX_BYTE_MASK                0xFFFFF

#else

/* ESP32-C6. Its SLCHOST block keeps the ESP32 offsets, checked against
   esp-idf components/soc/esp32c6/register/soc/sdio_slc_host_reg.h:
   TOKEN_RDATA 0x44, INT_RAW 0x50, INT_ST 0x58, PKT_LEN 0x60, INT_CLR 0xd4,
   CONF_W7 0x8c. Upstream esp-at's non-ESP32 branch (0x04/0x08/0x1c/0x20/0x28)
   is a different family and reads back 0 here - the token count never leaves 0
   and the host can never find a buffer to send into. */
#define ESP_SDIO_PKT_LEN            0x60
#define ESP_SDIO_INT_CLR            0xD4
#define ESP_SDIO_INT_RAW            0x50
#define ESP_SDIO_INT_ST             0x58
#define ESP_SDIO_TOKEN_RDATA        0x44
#define ESP_SDIO_SEND_OFFSET        16
#define ESP_SDIO_CONF               0x8C
#define ESP_SDIO_CONF_OFFSET        0

#define RX_BYTE_MAX                 0x100000
#define RX_BYTE_MASK                0xFFFFF

#endif

#endif /* SDIO_HOST_REG_H_ */
