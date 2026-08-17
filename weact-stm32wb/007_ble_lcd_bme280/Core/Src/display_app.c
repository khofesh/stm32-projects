/**
 ******************************************************************************
 * @file    display_app.c
 * @brief   SSD1315 OLED rendering of the BME280 measurement
 ******************************************************************************
 */

#include "display_app.h"
#include "driver_ssd1315_basic.h"
#include <stdio.h>
#include <string.h>

/* font 16 is 8 px wide and 16 px tall, so 16 columns by 4 rows on a 128x64 panel */
#define DISPLAY_LINE_CHARS   16
#define DISPLAY_ROW_T        0
#define DISPLAY_ROW_P        16
#define DISPLAY_ROW_H        32

static char s_line[DISPLAY_LINE_CHARS + 1];

/**
 * @brief     write one padded row
 * @param[in] y row origin
 * @return    0 on success, 1 on failure
 * @note      padding to the full width overwrites the previous value, so the
 *            frame never has to be cleared between updates
 */
static uint8_t display_row(uint8_t y)
{
    size_t n = strlen(s_line);

    while (n < DISPLAY_LINE_CHARS)
    {
        s_line[n++] = ' ';
    }
    s_line[DISPLAY_LINE_CHARS] = '\0';

    return ssd1315_basic_string(0, y, s_line, DISPLAY_LINE_CHARS, 1, SSD1315_FONT_16);
}

uint8_t DISPLAY_APP_Init(void)
{
    if (ssd1315_basic_init(SSD1315_INTERFACE_IIC, SSD1315_ADDR_SA0_0) != 0)
    {
        return 1;
    }

    if (ssd1315_basic_clear() != 0)
    {
        return 1;
    }

    (void)snprintf(s_line, sizeof(s_line), "BME280");
    if (display_row(DISPLAY_ROW_T) != 0)
    {
        return 1;
    }

    (void)snprintf(s_line, sizeof(s_line), "waiting...");

    return display_row(DISPLAY_ROW_P);
}

uint8_t DISPLAY_APP_ShowMeasurement(const bme280_app_data_t *data)
{
    int32_t  t;
    uint32_t p;
    uint32_t h;
    char     sign;

    if (data == NULL)
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

    return display_row(DISPLAY_ROW_H);
}
