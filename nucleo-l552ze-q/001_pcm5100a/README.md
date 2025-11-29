# NUCLEO-L552ZE-Q + pcm5100a

## pins

### ST7789 pins

- GND - Ground
- VCC - Power (3.3V)
- SCK (you wrote SDK) - SPI Clock
- SDA - SPI Data (MOSI - Master Out Slave In)
- RES - Reset
- DC - Data/Command selection
- BLK (or BL) - Backlight control

| ST7789 Pin | NUCLEO-L552ZE-Q Pin | Function                      |
| ---------- | ------------------- | ----------------------------- |
| GND        | GND                 | Ground                        |
| VCC        | 3.3V                | Power supply                  |
| SCK        | D13 (PA5)           | SPI1_SCK                      |
| SDA        | D11 (PA7)           | SPI1_MOSI                     |
| RES        | D9 (PC7)            | GPIO for Reset                |
| DC         | D8 (PA9)            | GPIO for Data/Command         |
| BLK        | D7 (PA8) or 3.3V    | Backlight (GPIO or always-on) |
