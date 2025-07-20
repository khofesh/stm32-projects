# L3GD20H + stm32

links:

- https://github.com/STMicroelectronics/l3gd20h-pid
- example: https://github.com/STMicroelectronics/STMems_Standard_C_drivers/blob/master/l3gd20h_STdC/README.md
- https://www.adafruit.com/product/1032

pin connection:

- SCL -> PA5 (SCK)
- SDA -> PA7 (MOSI)
- SA0 -> PA6 (MISO)
- CS -> PA4
- DRDY -> PA1 (gpio input)
- INT1 -> PA2 (gpio input)
- GND
- 3Vo
- Vin -> none, leave unconnected (this is for input power if you want to power the board with 5V)
