# lockup a few seconds after boot (fixed)

Original report: the board runs for several seconds after reset, then stops.
Display frozen, BLE dead, no fault reported. Build was clean and well within
budget (~20% of the 512K flash region, ~3% of RAM), so size was never involved.

Caught with an ST-LINK V3 by halting the hung target rather than resetting it:

```
[stm32wbx.cpu] halted due to debug-request, current mode: Handler External Interrupt(3)
xPSR: 0x01000013  pc: 0x08003614  msp: 0x2002ffc0
CFSR 0x00000000  HFSR 0x00000000        <- no fault
ICSR 0x04C3C813  -> VECTACTIVE=19, VECTPENDING=60, ISRPENDING=1
```

## what it was

**Handler mode, exception 19, `pc = 0x08003614`.** Exception 19 is IRQ 3,
`RTC_WKUP_IRQn`. That PC resolves to `Default_Handler`, which in the CubeMX
startup file is `b Infinite_Loop`. The core was not faulting and had not
overflowed its stack (MSP sat 64 bytes below the top of RAM) - it had simply
vectored into the catch-all spin and could never leave.

The vector table confirmed it directly. Entry 19 lives at `0x0800004C`:

```
0x0800004c: 08003615        <- Default_Handler | thumb bit
```

**`RTC_WKUP_IRQHandler` did not exist anywhere in the project.**
`HW_TS_RTC_Wakeup_Handler()` is defined in `Core/Src/hw_timerserver.c:487` and
declared in `hw_if.h`, and `app_conf.h:494` even carries the ST redirect

```c
#define HAL_RTCEx_WakeUpTimerIRQHandler(...)  HW_TS_RTC_Wakeup_Handler( )
```

but nothing ever called it. That redirect only pays off if CubeMX generates an
`RTC_WKUP_IRQHandler` into `stm32wbxx_it.c`, and it generates one only when the
RTC wake-up interrupt is ticked in the NVIC tab. It is not ticked in this `.ioc`.

The interrupt gets enabled anyway, at runtime, by the timer server the BLE stack
depends on - `hw_timerserver.c:665`, `HAL_NVIC_EnableIRQ(CFG_HW_TS_RTC_WAKEUP_HANDLER_ID)`.
So the sequence is: BLE stack comes up, timer server arms an RTC wake-up a few
seconds out, the interrupt fires into `Default_Handler`, and the core spins there
forever. `VECTPENDING=60` in the dump is IRQ 44, `IPCC_C1_RX` - the BLE mailbox
from CPU2, left pending forever because the core never returns to thread mode.
That is why the radio dies along with everything else.

This was a pre-existing defect in the project. It had nothing to do with the
BME280 or the OLED; it would have shown up the moment the BLE stack ran.

## the fix

`Core/Src/stm32wbxx_it.c`, inside `USER CODE BEGIN 1` (so it survives CubeMX
regeneration), with `#include "hw_if.h"` added to `USER CODE BEGIN Includes`:

```c
void RTC_WKUP_IRQHandler(void)
{
  HW_TS_RTC_Wakeup_Handler();
}
```

Vector 19 now reads `0x08003591` -> `RTC_WKUP_IRQHandler`.

## also found along the way (not the cause)

**`ssd1315_gram_update()` issues one I2C transaction per byte.**
`a_ssd1315_write_byte()` does a full START / address / control byte / data /
STOP for each byte, so one full-frame push is 8 pages x (3 commands + 128 data)
= 1048 transactions. The first cut of `display_app.c` called
`ssd1315_basic_string()` three times per frame, so 3144 transactions.

I2C3 is at ~104 kHz, not 400 kHz: `Timing = 0x10B17DB5` with PCLK1 = 64 MHz
decodes to PRESC=1, SCLH=125, SCLL=181, giving tSCL = 9.625 us. Each single-byte
transaction is then ~270 us and a frame cost roughly **850 ms of blocking bus
time per second**, starving `UTIL_SEQ_Run()`.

That was never what hung the board - the RTC vector was - but it was bad enough
to be worth fixing, and it is fixed: `display_app.c` now composes the frame in
GRAM with no bus traffic and pushes it with page-sized block writes (one 3-byte
command transaction plus one 128-byte data transaction per page, **16 per frame
instead of 3144**), one page per `DISPLAY_APP_Process()` call so the longest
single bus hold is ~12 ms. Needed `ssd1315_basic_get_handle()` added to
`Drivers/SSD1315/driver_ssd1315_basic.c`.

**The BME280 answers on 0x77, not 0x76.** The secondary-address fallback in
`BME280_APP_Init()` is what makes it work; read back from RAM, `s_addr = 0x77`.
README now documents both bus addresses.

**Init failures used to call `Error_Handler()`**, which hard-hangs with
interrupts disabled and looks identical to this bug from the outside. Both inits
are non-fatal now, and `BME280_APP_Process()` is gated on an `s_inited` flag so a
failed init does not burn two 100 ms I2C timeouts every pass through the loop.

## verified on the bench

Flashed and verified over SWD, then sampled the running target four times:

```
Thread mode, pc = DISPLAY_APP_Process   display_app.c:182
Thread mode, pc = UTIL_SEQ_Run          stm32_seq.c:244
Thread mode, pc = BME280_APP_Process    bme280_app.c:170
Thread mode, pc = BME280_APP_Process    bme280_app.c:117
```

Thread mode every time, four different PCs, sequencer being serviced. Live state
read out of RAM:

```
BME280 s_inited = 1   display s_inited = 1   s_have_data = 1   s_addr = 0x77
temperature 0x0c47 =   3143  -> 31.43 degC
pressure    0x18a94 = 100996 -> 1009.96 hPa
humidity    0x10e61 =  69217 -> 67.6 %RH
```

`display s_inited = 1` means `ssd1315_basic_init()` returned 0, which required
dozens of I2C writes to be ACKed, so the panel is wired and responding.

## what is still open

- ~~The panel has not been confirmed visually.~~ Confirmed working. One more bug
  had to be fixed first: `ssd1315_gram_write_string()` bounds checks with
  `x > (127 - font / 2)`, so at font 16 a full width 16 character line wraps
  instead of drawing its last glyph, and on the bottom row that wrap resets to
  `0,0` and eats the first character of the top row. See README.md.
- **BLE advertising of the measurement.** The original goal. `BME280_APP_GetData()`
  is the seam it should hang off.
- **I2C3 is at 104 kHz.** Bumping it to 400 kHz takes a frame from ~96 ms to
  ~24 ms and each page push from ~12 ms to ~3 ms. CubeMX: Connectivity -> I2C3 ->
  Parameter Settings -> I2C Speed Mode = Fast Mode, I2C Speed Frequency =
  400 kHz, regenerate. Same for I2C1 if the sensor reads should be quicker.
- **Do not also tick the RTC wake-up interrupt in the CubeMX NVIC tab.** That
  would generate a second `RTC_WKUP_IRQHandler` and collide with the one in
  `USER CODE BEGIN 1`. Pick one or the other.
- ~~`Debug/` needs a clean rebuild in CubeIDE.~~ Done. The tree now builds with
  the ST toolchain shipped in `/opt/st/stm32cubeide_2.1.1`; see README.md for the
  `PATH` to export. Do not build with the Fedora `arm-none-eabi-gcc`, it rejects
  `-fcyclomatic-complexity`.
- **`ssd1315_interface_debug_print()` does a blocking UART transmit** with a
  100 ms timeout on USART1. Fine now that errors are rare, but it is a hazard if
  a bus fault ever starts a per-page error storm.
