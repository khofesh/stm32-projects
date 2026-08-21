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
