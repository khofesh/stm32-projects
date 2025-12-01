# ST7789

- SCL = SPI Clock (SCLK/SCK) - clock signal
- SDA = SPI Data (MOSI) - Master Out Slave In (data FROM Nucleo TO LCD)
- There is no MISO pin on ST7789 - the display only receives data, it doesn't send data back to the microcontroller

![SPI func](./IPS-154-ST7789-SPI-FUNC.webp)

![SPI pin](./IPS-154-ST7789-SPI-PIN.webp)

## nucleo connection

- GND -> GND
- VCC -> 3.3V
- SCL -> SCK
- SDA -> MOSI
- RST -> any GPIO pin
- DC -> GPIO pin
- CS -> SPI NSS or any GPIO pin
- BL -> 3.3v (always-on backlight) or PWM-capable GPIO for brightness control

## failed module

- `gmt130 v1 0 schematic`
- https://goldenmorninglcd.com/tft-display-module/1.3-inch-240x240-st7789-gmt130-v1.0/
- https://github.com/423locked/GMT130-ESP8266/tree/main
- https://www.youtube.com/watch?v=EpuysRCbmqQ
- https://forum.arduino.cc/t/st7789-gmt-130-v1-0-not-working-well/1270332
