#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "platform_os.h"
#include "sdio_host_reg.h"
#include "sdio_host_log.h"
#include "sdio_driver_port.h"
#include "sdio_host_transport.h"

static const char TAG[] = "sdio_transport";

static uint32_t tx_sent_buffers = 0;    ///< Counter hold the amount of buffers already sent to sdio slave. Should be set to 0 when initialization.
static uint32_t rx_got_bytes   = 0;       ///< Counter hold the amount of bytes already received from sdio slave. Should be set to 0 when initialization.

/* Bounce buffer for the sub-block tail of a CMD53 transfer. Static, not on the
 * stack: a local costs 512 bytes of frame in both the send and the receive
 * path, and the Cortex-M33 traps the resulting overflow through MSPLIM as a
 * UsageFault (CFSR.STKOF) that escalates to HardFault. Only ever touched from
 * the main context - the SDMMC and console ISRs do not call in here. */
static uint32_t sdio_tail_buf[512U / sizeof(uint32_t)];

static void sdio_host_reset_counters(void)
{
    tx_sent_buffers = 0U;
    rx_got_bytes = 0U;
}

/******************  Init SDIO slave *********************/
static sdio_err_t esp_slave_init_io(void)
{
    sdio_err_t err;
    uint8_t ioe;
printf("esp_slave_init_io\r\n");
    err = sdio_driver_read_byte(0, SD_IO_CCCR_FN_ENABLE, &ioe);

    if (err != SDIO_SUCCESS) {
        return err;
    }

    SDIO_LOGD(TAG, "IOE: 0x%02x", ioe);

    uint8_t ior = 0;
    err = sdio_driver_read_byte(0, SD_IO_CCCR_FN_READY, &ior);

    if (err != SDIO_SUCCESS) {
        return err;
    }

    SDIO_LOGD(TAG, "IOR: 0x%02x", ior);

    // enable function 1
#if TARGET_ESP32
    ioe = 6;
#else
    ioe = 2;
#endif
    err = sdio_driver_write_byte(0, SD_IO_CCCR_FN_ENABLE, ioe, &ioe);

    if (err != SDIO_SUCCESS) {
        return err;
    }

    SDIO_LOGD(TAG, "IOE: 0x%02x", ioe);

    ior = 6;
    err = sdio_driver_write_byte(0, SD_IO_CCCR_FN_READY, ioe, &ior);

    if (err != SDIO_SUCCESS) {
        return err;
    }

    SDIO_LOGD(TAG, "IOE: 0x%02x", ior);

    // get interrupt status
    uint8_t ie;
    err = sdio_driver_read_byte(0, SD_IO_CCCR_INT_ENABLE, &ie);

    if (err != SDIO_SUCCESS) {
        return err;
    }

    SDIO_LOGD(TAG, "IE: 0x%02x", ie);

    // enable function interrupts and master enable
#if TARGET_ESP32
    ie = 7;
#else
    ie = 3;
#endif
    err = sdio_driver_write_byte(0, SD_IO_CCCR_INT_ENABLE, ie, &ie);

    if (err != SDIO_SUCCESS) {
        return err;
    }

    SDIO_LOGD(TAG, "IE: 0x%02x", ie);

    uint8_t bsl, bsh;

    bsl = 0;
    err = sdio_driver_write_byte(0, SD_IO_CCCR_BLKSIZEL, bsl, &bsl);

    if (err != SDIO_SUCCESS) {
        return err;
    }

    SDIO_LOGD(TAG, "Function 0 BSL: 0x%02x", bsl);

    bsh = 2;
    err = sdio_driver_write_byte(0, SD_IO_CCCR_BLKSIZEH, bsh, &bsh);

    if (err != SDIO_SUCCESS) {
        return err;
    }

    SDIO_LOGD(TAG, "Function 0 BSH: 0x%02x", bsh);

    uint8_t func1_bsl, func1_bsh;

    func1_bsl = 0;
    err = sdio_driver_write_byte(0, 0x110, func1_bsl, &func1_bsl);

    if (err != SDIO_SUCCESS) {
        return err;
    }

    SDIO_LOGD(TAG, "Function 1 BSL: 0x%02x", func1_bsl);
 
    func1_bsh = 2;         // Set block size 512 (0x200)  
    err = sdio_driver_write_byte(0, 0x111, func1_bsh, &func1_bsh);
    if (err != SDIO_SUCCESS) {
        return err;
    }

    SDIO_LOGD(TAG, "Function 1 BSH: 0x%02x", func1_bsh);

#if TARGET_ESP32
    uint8_t func2_bsl, func2_bsh;

    func2_bsl = 0;
    err = sdio_driver_write_byte(0, 0x210, func2_bsl, &func2_bsl);
    if (err != SDIO_SUCCESS) {
        return err;
    }

    SDIO_LOGD(TAG, "Function 2 BSL: 0x%02x", func2_bsl);

    func2_bsh = 2;
    err = sdio_driver_write_byte(0, 0x211, func2_bsh, &func2_bsh);

    if (err != SDIO_SUCCESS) {
        return err;
    }

    SDIO_LOGD(TAG, "Function 2 BSH: 0x%02x", func2_bsh);
#endif

    return SDIO_SUCCESS;
}

//host use this to initialize the slave as well as SDIO registers
sdio_err_t sdio_host_init(void)
{
    sdio_err_t ret;

    sdio_host_reset_counters();
    ret = sdio_driver_init();

    if (ret != SDIO_SUCCESS) {
        SDIO_LOGE(TAG, "sdio host init error, err: %d", ret);
        return ret;
    }

    ret = esp_slave_init_io();

    if (ret != SDIO_SUCCESS) {
        SDIO_LOGE(TAG, "esp slave init error, err: %d", ret);
        return ret;
    }

    return SDIO_SUCCESS;
}

sdio_err_t sdio_host_reinit(void)
{
    sdio_err_t ret;

    sdio_host_reset_counters();
    ret = sdio_driver_reinit();

    if (ret != SDIO_SUCCESS) {
        SDIO_LOGE(TAG, "sdio host reinit error, err: %d", ret);
        return ret;
    }

    ret = esp_slave_init_io();

    if (ret != SDIO_SUCCESS) {
        SDIO_LOGE(TAG, "esp slave reinit error, err: %d", ret);
        return ret;
    }

    return SDIO_SUCCESS;
}

/************************* RECEIVE ****************************/
// HOST receive data
/* The queue length is a difference between two free-running 20-bit counters, so
   a host counter that has run ahead of the slave's does not read as a negative
   number - it wraps to just under RX_BYTE_MAX. Nothing the slave can legally
   have queued comes near half the counter range, so that is where a backlog
   stops and a desync begins. */
#define RX_BYTE_DESYNC_MIN (RX_BYTE_MAX / 2)

/* Ceiling on one CMD53 out of the packet window, whatever the caller's buffer
   allows. A backlog of tens of kilobytes builds up during an HTTP GET, and a
   4096-byte read of it is answered with no data phase at all. */
#define SDIO_HOST_MAX_READ 2048U

static sdio_err_t esp_sdio_slave_get_rx_data_size(uint32_t* rx_size, uint32_t* raw)
{
    uint32_t len;
    sdio_err_t err = sdio_driver_read_bytes(1, ESP_SDIO_PKT_LEN, &len, 4);
    if (err != SDIO_SUCCESS) {
        return err;
    }
    len &= RX_BYTE_MASK;

    /* The absolute counter goes back to the caller as well: it is what a
       desynchronised rx_got_bytes has to be re-anchored on. */
    if (raw != NULL) {
        *raw = len;
    }
    len = (len + RX_BYTE_MAX - rx_got_bytes)%RX_BYTE_MAX;
    *rx_size = len;
    return SDIO_SUCCESS;
}

sdio_err_t sdio_host_get_packet(void* out_data, size_t size, size_t* out_length, uint32_t wait_ms)
{
    sdio_err_t err = SDIO_SUCCESS;
    uint32_t len = 0;
    uint32_t wait_time = 0;

    uint32_t raw = 0;

    if (size == 0) {
        SDIO_LOGE(TAG, "Invalid size:%u", (unsigned int)size);
        return ERR_INVALID_ARG;
    }

    for (;;) {
        err = esp_sdio_slave_get_rx_data_size(&len, &raw);

        if (err != SDIO_SUCCESS) {
            return err;
        }

        /* A backlog larger than the read buffer is normal - the slave queues
           several packets and an HTTP GET leaves kilobytes pending - and is
           served in ``size`` sized pieces by the truncation below. Only a
           wrapped difference is a fault: it means rx_got_bytes was credited
           bytes the slave never sent, and no amount of waiting brings it back
           down. Re-anchoring on the slave's own counter costs the bytes still
           queued at that moment, which are lost either way, and gets the link
           back instead of leaving it logging the same length every millisecond
           for as long as the console is up. */
        if (len >= RX_BYTE_DESYNC_MIN) {
            SDIO_LOGE(TAG, "rx counter desync: slave=%lu host=%lu, resyncing",
                      (unsigned long)raw, (unsigned long)rx_got_bytes);
            rx_got_bytes = raw;
            len = 0;
        }

        if (len > 0) {
            break;
        }

        //not error and no data, retry ``timeout_cnt`` times.
        wait_time++;

        if (wait_time >= wait_ms) {
            return ERR_TIMEOUT;
        }

        //vTaskDelay(1);
        platform_os_delay(1);
    }

    SDIO_LOGD(TAG, "get_packet: slave len=%lu, max read size=%u (raw=%lu got=%lu)",
              (unsigned long)len, (unsigned int)size,
              (unsigned long)raw, (unsigned long)rx_got_bytes);

    sdio_err_t truncated = SDIO_SUCCESS;

    if (len > size) {
        len = size;
        truncated = ERR_NOT_FINISHED;
    }

    if (len > SDIO_HOST_MAX_READ) {
        len = SDIO_HOST_MAX_READ;
        truncated = ERR_NOT_FINISHED;
    }

    /* The whole packet in one CMD53, its transfer rounded up to the block the
       slave pads to - 1024 clocked out for a 583-byte packet.
       Splitting it is what the log in todo.md was full of. The slave takes the
       length it is to serve from the address, reading it back as 0x1f800 -
       addr, and once a read has started on a packet it is done with the rest:
       reading 512 of a 583-byte packet leaves the remaining 71 counted by the
       length register but no longer available, and the second CMD53 that goes
       for them ends in "CMD53 read error, addr 0x1f7b9 count 512 ... TIMEOUT".
       Measured both ways round here - as 512 then 71, and as 512 with the
       address left at 583 - and the tail fails identically. */
    const uint32_t block_size = 512U; /* the driver supports no other block size */
    uint32_t xfer = ((len + block_size - 1U) / block_size) * block_size;

    if (xfer <= size) {
        err = sdio_driver_read_blocks(1, ESP_SLAVE_CMD53_END_ADDR - len, out_data, xfer);
    } else if (xfer <= sizeof(sdio_tail_buf)) {
        /* One block, and the caller's buffer is smaller than it: the padding
           goes into the bounce buffer rather than over the caller's. */
        err = sdio_driver_read_blocks(1, ESP_SLAVE_CMD53_END_ADDR - len,
                                      sdio_tail_buf, xfer);
        if (err == SDIO_SUCCESS) {
            memcpy(out_data, sdio_tail_buf, len);
        }
    } else {
        /* Caller's buffer is not block-aligned and more than a block short.
           Take whole blocks and let the next poll report the rest. */
        len = (size / block_size) * block_size;
        truncated = ERR_NOT_FINISHED;
        err = (len != 0U)
              ? sdio_driver_read_blocks(1, ESP_SLAVE_CMD53_END_ADDR - len, out_data, len)
              : ERR_INVALID_ARG;
    }

    if (err != SDIO_SUCCESS) {
        /* Give up on this packet, but credit it. The slave took the requested
           length out of the CMD53 address and moved its send FIFO on
           regardless of how the data phase ended, so bytes left uncredited
           here are asked for again on the next poll, fail the same way, and
           lock the link into a retry loop it never leaves - one dropped frame
           on this bus otherwise costs every AT command that follows it. */
        rx_got_bytes += len;
        return err;
    }

    *out_length = len;
    rx_got_bytes += len;
    /* ERR_NOT_FINISHED tells the caller more of this packet is still queued */
    return truncated;
}

sdio_err_t sdio_host_clear_intr(uint32_t intr_mask)
{
    SDIO_LOGD(TAG, "clear_intr: %08lX", (unsigned long)intr_mask);
    return sdio_driver_write_bytes(1, ESP_SDIO_INT_CLR, (uint8_t*)&intr_mask, 4);
}

sdio_err_t sdio_host_get_intr(uint32_t* intr_raw, uint32_t* intr_st)
{
    sdio_err_t r;
    SDIO_LOGV(TAG, "get_intr");

    if (intr_raw == NULL && intr_st == NULL) {
        return ERR_INVALID_ARG;
    }

    if (intr_raw != NULL) {
        r = sdio_driver_read_bytes(1, ESP_SDIO_INT_RAW, (uint8_t*)intr_raw, 4);
        if (r != SDIO_SUCCESS) {
            return r;
        }
    }

    if (intr_st != NULL) {
        r = sdio_driver_read_bytes(1, ESP_SDIO_INT_ST, (uint8_t*)intr_st, 4);
        if (r != SDIO_SUCCESS) {
            return r;
        }
    }

    return SDIO_SUCCESS;
}

sdio_err_t sdio_host_wait_int(uint32_t wait)
{
    return sdio_driver_wait_int(wait);
}

/*********************** SEND ***************************/

/* A slave that has buffers to give hands them over at once, so anything past a
 * couple of seconds here is a link that has gone away - most often the slave
 * rebooting under an AT+RESTORE. The old bound was 0xffff retries of 1 ms,
 * which parked the main loop for over a minute with nothing on the console. */
#define SDIO_TX_WAIT_MS         2000U
/* The token register is read over the same bus that loses the occasional frame,
 * so one failure is retried; a run of them is the link, not a frame. */
#define SDIO_TX_TOKEN_ERR_MAX   8U

static sdio_err_t esp_sdio_host_get_buffer_size(uint32_t *out_num)
{
    sdio_err_t ret;
    uint32_t len;
    ret = sdio_driver_read_bytes(1, ESP_SDIO_TOKEN_RDATA, &len, 4);
    if (ret != SDIO_SUCCESS) {
        SDIO_LOGE(TAG, "Read length error, ret=%d\r\n", ret);
        return ret;
    }
    SDIO_LOGD(TAG, " Read ESP32 len: %lu\r\n", (unsigned long)len);
    len = (len >> ESP_SDIO_SEND_OFFSET) & TX_BUFFER_MASK;
    len = (len + TX_BUFFER_MAX - tx_sent_buffers) % TX_BUFFER_MAX;
    *out_num = len;
    return SDIO_SUCCESS;
}

sdio_err_t sdio_host_send_intr(uint8_t intr_no)
{
    int ret = 0;
    uint32_t intr_mask = 0;
    if (intr_no >= 8) {
        SDIO_LOGE(TAG, " Error interrupt number\r\n");
        return ERR_INVALID_ARG;
    }

    intr_mask = 0x1 << (intr_no + ESP_SDIO_CONF_OFFSET);
    ret = sdio_driver_write_byte(1, ESP_SDIO_CONF, intr_mask, NULL);

    return ret;
}

sdio_err_t sdio_host_send_packet(const void* start, size_t length)
{
    sdio_err_t err;
    uint8_t* start_ptr = (uint8_t*)start;
    uint32_t len_remain = length;
    uint32_t block_size = 512;
    int buffer_used = (length + block_size - 1) / block_size;

    uint32_t waited = 0;
    uint32_t token_err = 0;

    while (1) {
        uint32_t num = 0;
        sdio_err_t token_ret = esp_sdio_host_get_buffer_size(&num);

        if (token_ret != SDIO_SUCCESS) {
            token_err++;

            if (token_err >= SDIO_TX_TOKEN_ERR_MAX) {
                SDIO_LOGE(TAG, "token read failed %lu times, err=%d",
                          (unsigned long)token_err, token_ret);
                return token_ret;
            }
        } else {
            token_err = 0;
            SDIO_LOGD(TAG, "Buffer size %lu can be send", (unsigned long)num);

            if (num * block_size >= length) {
                break;
            }

            SDIO_LOGD(TAG, "buffer is not enough: %lu, %d required. Retry...",
                      (unsigned long)num, buffer_used);
        }

        if (waited >= SDIO_TX_WAIT_MS) {
            SDIO_LOGI(TAG, "buffer is not enough: %lu, %d required.",
                      (unsigned long)num, buffer_used);
            return ERR_TIMEOUT;
        }

        platform_os_delay(1);
        waited++;
    }

    do {
        /* Though the driver supports to split packet of unaligned size into
         * length of 4x and 1~3, we still send aligned size of data to get
         * higher effeciency. The length is determined by the SDIO address, and
         * the remainning will be discard by the slave hardware.
         */
        int block_n = len_remain / block_size;
        int len_to_send;

        if (block_n) {
            len_to_send = block_n * block_size;
            err = sdio_driver_write_blocks(1, ESP_SLAVE_CMD53_END_ADDR - len_remain, start_ptr, len_to_send);
        } else {
            /* Same as the receive side: the packet window only answers block
               mode. The address still carries the real length, so the slave
               keeps len_remain bytes and discards the padding. */
            len_to_send = len_remain;
            memset(sdio_tail_buf, 0, sizeof(sdio_tail_buf));
            memcpy(sdio_tail_buf, start_ptr, len_remain);
            err = sdio_driver_write_blocks(1, ESP_SLAVE_CMD53_END_ADDR - len_remain,
                                           sdio_tail_buf, block_size);
        }

        if (err != SDIO_SUCCESS) {
            return err;
        }

        start_ptr += len_to_send;
        len_remain -= len_to_send;
    } while (len_remain);

    if (tx_sent_buffers >= TX_BUFFER_MAX) {
        tx_sent_buffers -= TX_BUFFER_MAX;
    }

    tx_sent_buffers += buffer_used;
    return SDIO_SUCCESS;
}
