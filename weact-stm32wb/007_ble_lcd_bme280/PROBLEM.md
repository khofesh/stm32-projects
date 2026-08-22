# WeEnv does not advertise

Status: **resolved**. The board was advertising the whole time; the host could
not hear it. Closed 2026-08-22.

---

## Symptom

No advertisement from the board in any BLE scan on the Linux host, under any
name, while ST BLE Toolbox on an iPhone had seen it before.

## Root cause

Two independent problems, stacked:

1. **The RF link is very weak.** At the CubeMX default `CFG_TX_POWER` (`0x18`,
   -0.15 dBm) the board reads about **-86 dBm at 30 cm** on this host's
   adapter -- weaker than phones several metres away, and below the level at
   which a 15-30 s scan reliably catches a 1 s advertising interval. The radio,
   the stack and the advertising payload were all fine.

   At `0x1F` (+6 dBm, the stack's maximum) the board is seen at -80..-82 dBm,
   about three reports per 45 s scan, and connects.

2. **Debug trace was dead**, which is why nothing about (1) could be observed.
   See "Trace" below.

Confirmed by A/B: same build, `CFG_TX_POWER 0x18` -> 0 sightings in a 45 s
scan; `0x1F` -> consistently found, `name='WeEnv'`, manufacturer data `0x0030`
updating with each reading.

## Fixes applied

| file | change |
| ---- | ------ |
| `STM32_WPAN/App/app_ble.c` | `#undef`/`#define CFG_TX_POWER (0x1F)` in the `USER CODE BEGIN PD` block, same override pattern already used for the advertising interval |
| `Core/Src/app_entry.c` | `APPD_Init()` was **never called** -- the `APPE_Init_1` user block was empty, so `DbgTraceInit()` never ran and every `APP_DBG_MSG` went nowhere even with `CFG_DEBUG_*_TRACE 1` |
| `Core/Src/app_debug.c` | `DbgOutputInit()` called `MX_USART1_UART_Init()`, which is `static` in `main.c` -> link error. Removed; `main()` already brings USART1 up before `MX_APPE_Init()` |
| `Core/Src/main.c` | take the CLK48 semaphore (`LL_HSEM_1StepLock(HSEM, 5)`) before the stack starts, as `004_sen55_ble_app` does. Latent bug noted earlier; not the cause of the silence (A/B tested), fixed on its merits |

## Trace

Trace now works and prints the whole boot sequence, ending in:

```text
 [app_ble.c][Adv_Request][1017] ==>> aci_gap_set_discoverable - Success
 [app_ble.c][Adv_Request][1032] ==>> Success: Start Fast Advertising
```

**PA9 does not reach the ST-Link V3 VCP on this bench.** `/dev/ttyACM0` is the
V3's VCP and stayed silent through every test, while the MCU was provably
transmitting (USART1 `ISR` = `0x6000D0`, TC and TXE set; PA9/PA10 in AF7;
`BRR 0x226` = 115107 baud). A Waveshare USB-serial adapter (CH340,
`/dev/ttyACM1`) on the same PA9 pin printed everything immediately.

```bash
stty -F /dev/ttyACM1 115200 raw -echo -crtscts && cat /dev/ttyACM1
```

`CFG_DEBUG_TRACE != 0` force-clears `CFG_LPM_SUPPORTED`, so trace and Phase 6's
STOP2 remain mutually exclusive. Turn trace off for power work.

## Dead ends, recorded so they are not repeated

- **`aci_hal_set_radio_activity_mask` is useless on this coprocessor firmware**
  (wireless FW 1.22.1, FUS 2.0.0). Even with mask `0xFFFF`, zero
  `ACI_HAL_END_OF_RADIO_ACTIVITY` events arrive while the board is provably
  advertising. Absence of those events is not evidence of a dead radio.
- **`hci_le_set_scan_parameters` / `hci_le_set_scan_enable` return `0x01`**
  (unknown HCI command) -- this stack build has no central/observer role, so
  the board cannot be used as a scanner to test its own RX path.
- Clocks were never the problem: `RCC->CR` HSERDY set, `BDCR` LSERDY set with
  RTCSEL=LSE, `CSR` RFWKPSEL=LSE.
- Halting the core with `STM32_Programmer_CLI -coreReg` leaves it halted; a
  following `st-flash reset` is needed before any on-air test.

## Still open

The RF path itself. -80 dBm at 30 cm with +6 dBm of transmit power is roughly
50 dB worse than expected for this module; the antenna, its matching network or
this host's adapter placement is the next thing to look at if the link needs to
be more than "works on the desk".

## Bug fixed along the way

`app_ble_policy.c` sized the advertising buffer from the *value* of each AD
length byte instead of the field footprint (`payload + 2`), writing 18 bytes
into `s_adv_data[17]`. Now derived through `APP_BLE_AD_FIELD()` with a
`_Static_assert` on the 31-byte limit and a runtime length check. Unrelated to
the silence; the payload is now confirmed good on air
(`mfr={48: 'bc0b051a'}`, changing per reading).

## Build note

Building from `Debug/` needs ST's toolchain -- the CubeIDE `subdir.mk` files
pass `-fcyclomatic-complexity`, which the Fedora `arm-none-eabi-gcc` rejects:

```bash
export PATH=/opt/st/stm32cubeide_2.1.1/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.14.3.rel1.linux64_1.0.100.202602081740/tools/bin:$PATH
cd Debug && make all
arm-none-eabi-objcopy -O binary 007_ble_lcd_bme280.elf fw.bin
st-flash --reset write fw.bin 0x08000000
```

---

# OLED frozen on the "waiting..." splash

Status: **resolved**, confirmed on hardware 2026-08-22. The panel now blanks
after the splash, wakes on the button and shows live rows.

---

## Symptom

After turning the Phase 6 power configuration on, the panel showed the boot
splash

```text
BME280
waiting...
```

and never updated. No measurement row ever appeared, and the panel never went
dark on the 10 s display timeout either.

## Root cause

Not `CFG_LPM_SUPPORTED`, despite being noticed right after it was set to 1.
The culprit was `CFG_DEBUGGER_SUPPORTED 0`, flipped in the same pass.

The `#else` branch of `APPD_Init()` puts the debug pins in analog mode. Its
CubeMX default list is `GPIO_PIN_4 | GPIO_PIN_3` on GPIOB -- JTRST and JTDO on
a board that uses them for nothing else. **On this board `PB4` is `I2C3_SDA`,
the OLED bus** (`.ioc`: `PB4.Signal=I2C3_SDA`, `PA7.Signal=I2C3_SCL`).

The ordering is what makes it look like a display bug rather than a pin bug:

```text
main()
  DISPLAY_APP_Init()      splash composed and pushed   <- I2C3 alive
  MX_APPE_Init()
    APPD_Init()           PB4 -> GPIO_MODE_ANALOG      <- I2C3 dead
  APP_ENV_Init()
    APP_DISPLAY_Init()
      DISPLAY_APP_PowerOff()   fails silently
```

Every I2C3 transfer after `APPD_Init()` fails, so the splash that was already
in the panel's GRAM stays lit forever. `DISPLAY_APP_PowerOff()` returning
non-zero is discarded with `(void)`, and `app_display_render()` gives up before
touching the bus, so nothing upstream notices.

This is the same class of bug as the PA15 one already commented in that
function: the CubeMX pin list assumes the JTAG-only pinout.

## Fix applied

| file | change |
| ---- | ------ |
| `Core/Src/app_debug.c` | `GPIO_PIN_4` dropped from the GPIOB analog list; only `PB3` (SWO, unused in the `.ioc`) goes analog |
| `Core/Src/app_debug.c` | `__HAL_RCC_GPIOA_CLK_DISABLE()` / `__HAL_RCC_GPIOB_CLK_DISABLE()` removed -- I2C1 (PB8/PB9), I2C3 (PA7/PB4) and the button (PA15) all live on those ports, and STOP2 gates peripheral clocks anyway, so gating them in RUN buys no power and only invites the same silent failure |

Both edits are inside `/* USER CODE BEGIN APPD_Init */` and survive
regeneration.

Before touching this list again, check the pin against the `.ioc`. The pins
that are genuinely free to go analog on this board are **PA13, PA14 and PB3**.

## Flashing with `CFG_DEBUGGER_SUPPORTED 0`

The same switch puts `PA13`/`PA14` (SWD) in analog mode and disables the debug
domain in STOP2, so a hotplug attach cannot work on a running target. Connect
under reset, and press the board reset button if the probe still does not see
the core:

```bash
export PATH=/opt/st/stm32cubeide_2.1.1/plugins/com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.linux64_2.2.400.202601091506/tools/bin:$PATH
STM32_Programmer_CLI -c port=SWD mode=UR -w Debug/007_ble_lcd_bme280.elf -v -rst
```

Read the `Voltage:` line before blaming the firmware. `0.00 V` there is VTREF,
not the target -- the board is unpowered or the target-VCC wire of the ribbon
is not connected, and no `mode=` will help:

```text
Board       : STLINK-V3SET
Voltage     : 0.01V
Error: Unable to get core ID
Error: No STM32 target found!
```

For bring-up work, leaving `CFG_DEBUGGER_SUPPORTED` at 1 keeps SWD alive
through STOP2 and does **not** disable `CFG_LPM_SUPPORTED`. It costs roughly a
milliamp, which matters only for the final current measurement.

---

# GATT discovery fails from the Linux host

Status: **not a firmware bug**. Understood 2026-08-22.

---

## Symptom

`scripts/bme280_notify.py` finds the board every time and then fails to
connect, three attempts out of three:

```text
found WeEnv at 00:80:E1:26:06:2E
attempt 1/3 failed:
```

The empty reason is the script printing `str(exc)` of a `BleakError`. The real
message is

```text
bleak.exc.BleakError: failed to discover services, device disconnected
```

and on a longer run it degrades to a bare `TimeoutError`.

## What actually happens

The link comes up and dies before discovery finishes. From BlueZ's D-Bus
properties:

```text
17:38:02.400  Connected = True
17:38:07.882  Connected = False     dropped after 5.5 s
17:38:11.921  Connected = True      BlueZ retried
17:38:33.356  Connected = False     dropped after 21.4 s
ServicesResolved: never True
Paired / Bonded: False
```

## Cause

The weak RF path already recorded above, not the firmware. A 60 s scan hears
**5 advertisement reports at -78..-84 dBm** — about one per 12 s against a
1 s advertising interval, so this host receives well under 1 % of advertising
events. That is enough to be discovered, which needs one packet, and nowhere
near enough for a service discovery, which needs dozens of round trips inside
the supervision timeout.

**nRF Connect on an iPhone at the same distance connects, discovers the
service and receives notifications for minutes.** The firmware is fine.

## Ruled out, so it is not re-tested

- Not `CFG_LPM_SUPPORTED`. The advertised payload keeps updating through the
  scan (32.18 -> 32.16 -> 32.13 degC), so sampling, the RTC timer server and
  `aci_gap_update_adv_data()` all run with STOP2 enabled.
- Not a stale bond. `bluetoothctl devices Paired` and `Bonded` are empty and
  the device properties report `Paired: False`.
- Not the connection parameter policy. `app_ble_policy_publish_task()` gates
  `aci_l2cap_connection_parameter_update_req()` behind `s_notify_enabled`, so
  no relaxed interval is applied while the central is walking the attribute
  table.
- Not a missing CPU1 wakeup. `Init_Exti()` (IPCC line 36, HSEM line 38) is
  called from `System_Init()`, `app_entry.c:262`.

## If it has to work from this host

Move the board within a few centimetres of the adapter, or fix the RF path
itself -- see "Still open" above. To get the disconnect reason rather than
guessing, capture HCI:

```bash
sudo btmon -w /tmp/weenv.btsnoop     # then run the script in another terminal
```

`Disconnect Complete` separates the cases: `0x08` supervision timeout is
RF/timing, `0x3E` connection failed to establish means the peripheral never saw
the connect indication, `0x13`/`0x16` means one side terminated deliberately.

Until then, **verify BLE behaviour from a phone**, and use this host only for
`--scan-only`, which the weak link is still good enough for.
