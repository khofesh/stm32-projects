/**
 ******************************************************************************
 * @file    display_app.c
 * @brief   SSD1315 OLED rendering of the BME280 measurement
 *
 * The frame is composed in the driver GRAM and then pushed to the panel one
 * page per DISPLAY_APP_Process() call. ssd1315_gram_update() is deliberately
 * not used: it issues one I2C transaction per byte, so a single full frame is
 * 1048 transactions (~283 ms at 100 kHz) of blocking bus time, which starves
 * the sequencer and the BLE stack.
 ******************************************************************************
 */

#include "display_app.h"
#include "driver_ssd1315_basic.h"
#include "main.h"
#include <stdio.h>
#include <string.h>

/* page addressing commands, mirroring the private defines in driver_ssd1315.c */
#define DISPLAY_CMD_PAGE_ADDR      0xB0
#define DISPLAY_CMD_COL_LOW        0x00
#define DISPLAY_CMD_COL_HIGH       0x10

#define DISPLAY_PAGES              8
#define DISPLAY_COLUMNS            128

/* font 16 is 8 px wide and 16 px tall, so 16 columns by 4 rows on a 128x64 panel.
   only 15 are usable: ssd1315_gram_write_string() bounds checks with
   x > (127 - font / 2), so the 16th glyph at x = 120 trips the wrap instead of
   being drawn. The wrap then advances y, and for the last row it also trips
   y > (63 - font) and resets to 0,0 - so a full width bottom row silently
   overwrites the first character of the top row. */
#define DISPLAY_LINE_CHARS         15
/* snprintf needs slack for an out of range reading, display_row() clips */
#define DISPLAY_LINE_BUF           24
#define DISPLAY_ROW_T              0
#define DISPLAY_ROW_P              16
#define DISPLAY_ROW_H              32

static char    s_line[DISPLAY_LINE_BUF];
static uint8_t s_page_buf[DISPLAY_COLUMNS];
static uint8_t s_inited;
static uint8_t s_pending;        /**< frame waiting to be pushed */
static uint8_t s_page;           /**< next page to push */
static uint8_t s_powered;        /**< panel is lit */

/**
 * @brief     compose one padded row into the gram, no bus traffic
 * @param[in] y row origin
 * @return    0 on success, 1 on failure
 * @note      padding to the full width overwrites the previous value, so the
 *            frame never has to be cleared between updates
 */
static uint8_t display_row(uint8_t y)
{
    size_t n = strlen(s_line);

    if (n > DISPLAY_LINE_CHARS)
    {
        n = DISPLAY_LINE_CHARS;
    }

    while (n < DISPLAY_LINE_CHARS)
    {
        s_line[n++] = ' ';
    }
    s_line[DISPLAY_LINE_CHARS] = '\0';

    return ssd1315_gram_write_string(ssd1315_basic_get_handle(), 0, y, s_line,
                                     DISPLAY_LINE_CHARS, 1, SSD1315_FONT_16);
}

/**
 * @brief     push one gram page to the panel
 * @param[in] page page index, 0..7
 * @return    0 on success, 1 on failure
 * @note      one 3 byte command transaction plus one 128 byte data transaction
 */
static uint8_t display_push_page(uint8_t page)
{
    ssd1315_handle_t *handle = ssd1315_basic_get_handle();
    uint8_t cmd[3];
    uint8_t n;

    cmd[0] = (uint8_t)(DISPLAY_CMD_PAGE_ADDR + page);
    cmd[1] = DISPLAY_CMD_COL_LOW;
    cmd[2] = DISPLAY_CMD_COL_HIGH;

    if (ssd1315_write_cmd(handle, cmd, sizeof(cmd)) != 0)
    {
        return 1;
    }

    /* the gram is column major, so a page has to be gathered before sending */
    for (n = 0; n < DISPLAY_COLUMNS; n++)
    {
        s_page_buf[n] = handle->gram[n][page];
    }

    return ssd1315_write_data(handle, s_page_buf, DISPLAY_COLUMNS);
}

uint8_t DISPLAY_APP_Init(void)
{
    uint8_t page;

    /* basic_init ends with a full clear, so the panel and the gram start blank */
    if (ssd1315_basic_init(SSD1315_INTERFACE_IIC, SSD1315_ADDR_SA0_0) != 0)
    {
        return 1;
    }

    s_inited = 1;

    (void)snprintf(s_line, sizeof(s_line), "BME280");
    if (display_row(DISPLAY_ROW_T) != 0)
    {
        return 1;
    }

    (void)snprintf(s_line, sizeof(s_line), "waiting...");
    if (display_row(DISPLAY_ROW_P) != 0)
    {
        return 1;
    }

    /* splash runs before the BLE stack is up, so pushing it in one go is fine */
    for (page = 0; page < DISPLAY_PAGES; page++)
    {
        if (display_push_page(page) != 0)
        {
            return 1;
        }
    }

    s_pending = 0;
    s_page    = 0;
    s_powered = 1;

    return 0;
}

uint8_t DISPLAY_APP_ShowMeasurement(const bme280_app_data_t *data)
{
    int32_t  t;
    uint32_t p;
    uint32_t h;
    char     sign;

    /* composing for a dark panel would only queue I2C traffic nobody sees */
    if ((data == NULL) || (s_inited == 0) || (s_powered == 0))
    {
        return 1;
    }

    /* temperature is 0.01 degC */
    t    = data->temperature;
    sign = (t < 0) ? '-' : ' ';
    if (t < 0)
    {
        t = -t;
    }
    (void)snprintf(s_line, sizeof(s_line), "T %c%ld.%02ld C",
                   sign, (long)(t / 100), (long)(t % 100));
    if (display_row(DISPLAY_ROW_T) != 0)
    {
        return 1;
    }

    /* pressure is Pa, shown as hPa with one decimal */
    p = data->pressure;
    (void)snprintf(s_line, sizeof(s_line), "P %lu.%01lu hPa",
                   (unsigned long)(p / 100U), (unsigned long)((p % 100U) / 10U));
    if (display_row(DISPLAY_ROW_P) != 0)
    {
        return 1;
    }

    /* humidity is 1024 * %RH, shown with one decimal */
    h = data->humidity;
    (void)snprintf(s_line, sizeof(s_line), "H %lu.%01lu %%",
                   (unsigned long)(h / 1024U), (unsigned long)(((h % 1024U) * 10U) / 1024U));
    if (display_row(DISPLAY_ROW_H) != 0)
    {
        return 1;
    }

    /* queue the frame, DISPLAY_APP_Process() walks it out one page at a time */
    s_pending = 1;
    s_page    = 0;

    return 0;
}

void DISPLAY_APP_Process(void)
{
    if ((s_inited == 0) || (s_pending == 0))
    {
        return;
    }

    /* one page per call keeps the longest bus hold at roughly 12 ms */
    if (display_push_page(s_page) != 0)
    {
        s_pending = 0;

        return;
    }

    s_page++;
    if (s_page >= DISPLAY_PAGES)
    {
        s_pending = 0;
        s_page    = 0;
    }
}

uint8_t DISPLAY_APP_HasPendingFrame(void)
{
    return s_pending;
}

uint8_t DISPLAY_APP_PowerOn(void)
{
    if (s_inited == 0)
    {
        return 1;
    }

    if (s_powered != 0)
    {
        return 0;
    }

#if defined(OLED_PWR_Pin)
    /* rail first, then the controller, so the panel never sees commands
       before VCC has settled */
    HAL_GPIO_WritePin(OLED_PWR_GPIO_Port, OLED_PWR_Pin, GPIO_PIN_SET);
#endif

    if (ssd1315_basic_display_on() != 0)
    {
        return 1;
    }

    s_powered = 1;

    return 0;
}

uint8_t DISPLAY_APP_PowerOff(void)
{
    if (s_inited == 0)
    {
        return 1;
    }

    if (ssd1315_basic_display_off() != 0)
    {
        return 1;
    }

#if defined(OLED_PWR_Pin)
    HAL_GPIO_WritePin(OLED_PWR_GPIO_Port, OLED_PWR_Pin, GPIO_PIN_RESET);
#endif

    /* nothing composed so far survives a rail cut, drop any queued frame */
    s_pending = 0;
    s_page    = 0;
    s_powered = 0;

    return 0;
}

uint8_t DISPLAY_APP_IsPowered(void)
{
    return s_powered;
}
