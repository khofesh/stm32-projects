# STM32 + SSD1681

module used: https://www.tokopedia.com/syalis-electrical/weact-studio-1-54-inch-black-white-epaper-module-epaper-display-driver-ic-ssd1681-syalis-1731140617723020668

description:

```
WeAct Studio 1.54 Inch
Black-White Epaper Module
bisa aktif tanpa daya


WeAct Studio 1.54 Inch Black-White
-1.54 inches Display
-200x200 Resolution
-Black-White Display color
-3.3~5V Supply
-I/O 3.3V
-4-PIN SPI Interface
-Driver IC SSD1681
```

SSD1681 pin connection to STM32F446RE

- BUSY -> any gpio input pin (PA0)
- RES -> any gpio output pin (PA1)
- D/C -> any gpio output pin (PA2)
- CS -> PA4 (output)
- SCL -> PA5 (SCK)
- SDA -> PA7 (MOSI)
- GND -> GND
- VCC -> 3.3V
