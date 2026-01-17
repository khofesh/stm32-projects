# Pipe directly from arm-none-eabi-size

arm-none-eabi-size 00_aht_lcd.elf | ./stm32_size.py STM32G031F8

# Or paste the output as argument

./stm32_size.py STM32G031F8 ' 37632 468 2700 40800 9f60 00_aht_lcd.elf'

# List supported chips

./stm32_size.py --list

```

Output will look like:
```

============================================================
Memory Analysis: 00_aht_lcd.elf
Target: STM32G031F8
============================================================

Section Breakdown:
.text (code + const) : 36.8 KB
.data (initialized) : 468 B
.bss (uninitialized) : 2.6 KB

Memory Usage:
FLASH: 37.2 KB / 64.0 KB [█████████████████░░░░░░░░░░░░░] 58.1%
RAM: 3.1 KB / 8.0 KB [███████████░░░░░░░░░░░░░░░░░░░] 38.7%

Remaining:
FLASH: 26.8 KB
RAM: 4.9 KB
