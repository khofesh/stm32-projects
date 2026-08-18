# STM32WB55 + BME280 + SSD1315 OLED

WeAct STM32WB55CG board. Reads temperature, pressure and humidity from a BME280
over I2C1 and renders them on a 128x64 SSD1315 OLED over I2C3, alongside the
STM32_WPAN BLE stack running on CPU2.

- https://www.tokopedia.com/syalis-electrical/display-oled-0-96-biru-blue-i2c-ss1315-syalis

## pin connections

BME280

- PB8 (I2C1) -> BME280 SCL
- PB9 (I2C1) -> BME280 SDA

Display OLED 0.96 biru blue I2C SS1315

- PA7 (I2C3) -> LCD SCL
- PB4 (I2C3) -> LCD SDA

## i2c addresses

| device  | bus  | 7 bit | 8 bit write | note |
| ------- | ---- | ----- | ----------- | ---- |
| BME280  | I2C1 | 0x77  | 0xEE        | secondary address, SDO strapped high on this module. `BME280_APP_Init()` tries 0x76 first and falls back |
| SSD1315 | I2C3 | 0x3C  | 0x78        | `SSD1315_ADDR_SA0_0` |

Both buses run at ~104 kHz, not 400 kHz: `Timing = 0x10B17DB5` with PCLK1 =
64 MHz decodes to PRESC=1, SCLH=125, SCLL=181, so tSCL = 9.625 us.

## software layout

| file | role |
| ---- | ---- |
| `Core/Src/bme280_app.c` | forced mode sampling state machine on I2C1, one sample per second, non blocking |
| `Core/Src/display_app.c` | frame composition into the driver GRAM plus page at a time push to the panel |
| `Core/Src/driver_ssd1315_interface.c` | LibDriver SSD1315 port layer on I2C3 |
| `Drivers/BME280` | Bosch BME280_SensorAPI |
| `Drivers/SSD1315` | LibDriver SSD1315 |

`main()` runs `MX_APPE_Process()` (the sequencer), then `BME280_APP_Process()`,
then `DISPLAY_APP_Process()`. Nothing in the loop blocks for more than about
12 ms, which is the longest single I2C hold.

### display update path

`ssd1315_gram_update()` is deliberately not used. It issues one I2C transaction
per byte through `a_ssd1315_write_byte()`, so a full frame is
8 pages x (3 commands + 128 data) = 1048 transactions, roughly 283 ms of
blocking bus time, which starves `UTIL_SEQ_Run()` and the BLE stack.

Instead `DISPLAY_APP_ShowMeasurement()` only composes into the GRAM (no bus
traffic) and sets a pending flag. `DISPLAY_APP_Process()` then pushes one page
per main loop pass as one 3 byte command transaction plus one 128 byte data
transaction: **16 transactions per frame instead of 1048**. This needed
`ssd1315_basic_get_handle()` added to `Drivers/SSD1315/driver_ssd1315_basic.c`.

## build and flash

The project builds in STM32CubeIDE. From the command line, use the ST toolchain,
not the distro `arm-none-eabi-gcc` - CubeIDE passes `-fcyclomatic-complexity`,
which upstream GCC rejects:

```sh
export PATH=/opt/st/stm32cubeide_2.1.1/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.14.3.rel1.linux64_1.0.100.202602081740/tools/bin:$PATH
make -C Debug -j8 all
```

Flash and run with an ST-LINK:

```sh
openocd -f interface/stlink.cfg -c "transport select hla_swd" -f target/stm32wbx.cfg \
  -c init -c halt -c "program Debug/007_ble_lcd_bme280.elf verify" -c "reset run" -c shutdown
```

`Error: target voltage may be too low` is expected when the board is powered
over its own USB and VTref is not wired to the probe. SWD still works.

## bugs fixed

Three defects were found and fixed. The first was the one that made the board
appear to die a few seconds after reset; see `PROBLEM.md` for the full trace.

### 1. missing `RTC_WKUP_IRQHandler` - the lockup

The STM32_WPAN timer server enables `RTC_WKUP_IRQn` at runtime
(`hw_timerserver.c:665`), but the RTC wake-up interrupt is not ticked in the
CubeMX NVIC tab, so no handler was generated and vector 19 pointed at
`Default_Handler`, which is `b Infinite_Loop`. The first timer server timeout
after the BLE stack came up trapped the core there forever, leaving
`IPCC_C1_RX` pending and killing the radio along with everything else.

Fixed in `Core/Src/stm32wbxx_it.c` inside `USER CODE BEGIN 1`, with
`#include "hw_if.h"` in `USER CODE BEGIN Includes`:

```c
void RTC_WKUP_IRQHandler(void)
{
  HW_TS_RTC_Wakeup_Handler();
}
```

Do **not** also tick the RTC wake-up interrupt in the CubeMX NVIC tab. That
generates a second `RTC_WKUP_IRQHandler` and collides with this one.

### 2. full width text wrapped and ate the top left character

`ssd1315_gram_write_string()` bounds checks with `x > (127 - font / 2)`
(`driver_ssd1315.c:706`). At `SSD1315_FONT_16` the 16th glyph sits at x = 120,
which trips that check, so it is never drawn - the function wraps to x = 0 and
advances y instead. For the bottom row the wrap also trips `y > (63 - font)` and
resets to `0,0`, so the humidity line's trailing pad character was overwriting
the first character of the temperature line on every frame. The panel showed
`  30.89 C` instead of `T 30.89 C`.

A 128 px wide panel therefore holds **15 usable characters at font 16, not 16**.
`DISPLAY_LINE_CHARS` is 15, and `display_row()` clips before padding.

### 3. init failures called `Error_Handler()`

`Error_Handler()` hangs with interrupts disabled, which from the outside is
indistinguishable from bug 1. Both inits are non fatal now, and
`BME280_APP_Process()` is gated on an `s_inited` flag so a failed init does not
burn two 100 ms I2C timeouts on every pass through the loop.

## verifying on the bench

Halt the running target rather than resetting it, and read the application state
straight out of RAM. Symbol addresses move between builds, so resolve them first:

```sh
arm-none-eabi-nm Debug/007_ble_lcd_bme280.elf | grep -E " uwTick$| gs_handle| s_data"
```

A live board shows Thread mode, an advancing `uwTick`, and fresh values in
`s_data`:

```
t=100s  Thread mode  uwTick 100004  temp 0x0c17 press 0x18aaf hum 0x1150d
t=110s  Thread mode  uwTick 110006  temp 0x0c17 press 0x18aae hum 0x1150d
```

`temperature` is 0.01 degC, `pressure` is Pa, `humidity` is 1024 * %RH.

The frame itself can be checked without looking at the glass by dumping the
driver GRAM and rendering it. `gram` is at `gs_handle + 59` (14 function
pointers, then `inited`, `iic_addr`, `iic_spi`), laid out column major as
`gram[128][8]`:

```sh
openocd -f interface/stlink.cfg -c "transport select hla_swd" -f target/stm32wbx.cfg \
  -c init -c halt -c "dump_image /tmp/gram.bin 0x200009B7 1024" -c resume -c shutdown

python3 -c "
d = open('/tmp/gram.bin','rb').read()
for y in range(64):
    p, b = divmod(y, 8)
    print(''.join('#' if d[c*8+p] >> b & 1 else '.' for c in range(128)))
"
```

```
#######....####.....##.......##.....####......#####      <- "T  30.93 C"
######.....#.......##.......#.......##........####       <- "P 1005.3 hPa"
###..###...###.....##.......####..............#...#      <- "H 69.4 %"
```

## still open

- **BLE advertising of the measurement.** The original goal, not implemented.
  `BME280_APP_GetData()` is the seam it should hang off.
- **I2C at 104 kHz.** Bumping to 400 kHz takes a frame from ~96 ms to ~24 ms and
  each page push from ~12 ms to ~3 ms. In CubeMX: Connectivity -> I2C3 ->
  Parameter Settings -> I2C Speed Mode = Fast Mode, I2C Speed Frequency =
  400 kHz. Same for I2C1.
- **`ssd1315_interface_debug_print()` does a blocking UART transmit** with a
  100 ms timeout on USART1. Fine while errors are rare, but a hazard if a bus
  fault ever starts a per page error storm.
