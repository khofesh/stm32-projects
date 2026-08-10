# SDIO bring-up: slave never answers CMD5

Status: **unresolved**. The STM32 host enumerates correctly as far as software can
be checked; the ESP32 boots correctly and is built with SDIO AT enabled; the two
do not talk.

## Symptom

```
host ready, start initializing slave...
I sdio_driver: [Clock] kernel=250000000Hz, div=312, SDMMC_CK=400641Hz
E sdio_driver: HAL_SDIO_Init failed after 25 attempts, err=0x00000000
E sdio_driver: probe: CMD0 err=0x00000000
E sdio_driver: probe: CMD5 err=0x00000004 resp=0x00000000 nbr_of_func=0
E sdio_driver: probe: slave did not answer CMD5 - check that the ESP is running SDIO AT firmware, the CMD/D0-D3 pull-ups, and GND
E sdio_transport: sdio host init error, err: -1
SDIO init error
```

## Environment

| Item           | Value                                                                 |
| -------------- | --------------------------------------------------------------------- |
| Host           | NUCLEO-H533RE, SDMMC1 in SDIO mode, 400 kHz, 4-bit configured         |
| Slave          | ESP32-D0WD rev3, WROOM-32, 4 MB flash                                 |
| Slave firmware | esp-at `master` (d7126a88), reports v4.2.0.0, ESP-IDF v5.4.4          |
| Reference      | `esp-at/examples/at_sdio_host/STM32` (STM32F103ZET, hand-rolled SDIO) |

## Reading the error codes

- `CMD0 err=0x00000000` is **not** evidence of a working bus. CMD0 (GO_IDLE_STATE)
  is a broadcast command with no response, so the HAL only confirms it wrote the
  command register. This line reads identically with the ESP32 unplugged.
- `CMD5 err=0x00000004` is `SDMMC_ERROR_CMD_RSP_TIMEOUT`. Critically, this is the
  **hardware** CTIMEOUT flag, set by the CPSM when no response start bit arrives
  within 64 clocks — not the driver's software timeout, which is
  `SDMMC_ERROR_TIMEOUT` (`0x80000000`). See `SDMMC_GetCmdResp4()` in
  `stm32h5xx_ll_sdmmc.c:1572`.
- `HAL_SDIO_Init failed, err=0x00000000` is expected noise: `SDIO_InitCard()`
  returns `HAL_ERROR` without ever writing `ErrorCode`, which is why
  `SdioProbeSlave()` exists.

So the CMD line is electrically idle when the response is due. Either nothing is
reaching the slave, or the slave is not driving CMD.

## Narrowing: only three wires matter

CMD5 fails before any bus-width negotiation, and its R4 response returns on the
CMD line. The failing path is therefore **CK, CMD, and GND only**.

D0-D3, their pull-ups, and the 4-bit configuration are all irrelevant to this
specific failure. Time spent on the D2/GPIO12 strapping problem was necessary
for 4-bit operation later, but it was never going to fix this.

## Eliminated

| Hypothesis                     | How it was ruled out                                                                                                                                                  |
| ------------------------------ | --------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| ESP32 not booting              | Full boot log captured: POWERON_RESET, app loaded from `ota_0`, `at-init` completes, `module_name: WROOM-32`, `v4.2.0.0`                                              |
| GPIO12 strapping / 1.8 V flash | eFuse burned: `XPD_SDIO_FORCE`/`REG`/`TIEH` now force 3.3 V. Module is WROOM-32, so 3.3 V flash was correct                                                           |
| SDIO not in the slave build    | `CONFIG_AT_BASE_ON_SDIO=y` in the generated `sdkconfig`                                                                                                               |
| Wrong pull-up value            | 10 kΩ, which is exactly what the esp-at example README specifies (10 kΩ on CMD and D0-D3)                                                                             |
| Boot-order skew                | Enumeration now retries 25 × 200 ms = 5 s; ESP32 reaches `at-init` at ~1.1 s. Still fails                                                                             |
| Conflicting SD-mode init       | `MX_SDMMC1_SD_Init()` returns early (`main.c:359`); `HAL_SD_Init` never runs                                                                                          |
| Wrong IRQ routing              | `SDMMC1_IRQHandler` calls `SdioPortIrqHandler()` (`stm32h5xx_it.c:223`)                                                                                               |
| Kernel clock dead              | Reads 250 MHz; PLL1Q enabled explicitly in `SdioKernelClockConfig()` before init                                                                                      |
| CMD5 CRC quirk                 | The F1 reference notes R4 has no CRC. The H5 `SDMMC_GetCmdResp4()` already treats `CCRCFAIL` as "response received" and only errors on `CTIMEOUT`, so this is handled |

## Not yet verified

**Nobody has confirmed a single edge leaves the STM32.** Every diagnosis so far
has been inference from a log that, as noted above, cannot distinguish a dead
host from a silent slave. This is the gap.

1. **Is CK toggling at PC12?** At 400 kHz with `SDMMC_CLOCK_POWER_SAVE_DISABLE`
   the clock should run continuously once powered.
2. **Is CMD framing appearing on PB2?** CMD0 then CMD5 should be visible as
   48-bit frames.
3. **Do those signals arrive at ESP32 GPIO14 / GPIO15?** Continuity through the
   actual jumper wires, not just presence at the MCU pin.
4. **Is GND common between the two boards?**

## Open hypotheses, in order

1. **Wiring fault.** A single bad jumper on CK or CMD produces exactly this.
   Now the leading hypothesis, since board routing (below) came back clean for
   the three pins that matter.
2. **Slave-side SDIO init failed silently.** See below.
   The captured boot log ends at `v4.2.0.0 (unknown)` with nothing after it. If
   `sdio_slave_initialize()` or `sdio_slave_start()` failed, the message would
   appear after that point, and the log may simply have been truncated before
   it.

## Board routing check (UM3121 / MB1814)

Per the NUCLEO-H533RE pin function table:

| Pin  | Signal | Board function                         | Verdict                          |
| ---- | ------ | -------------------------------------- | -------------------------------- |
| PB2  | CMD    | plain IO                               | free                             |
| PB13 | D0     | plain IO                               | free                             |
| PC9  | D1     | `ARD_D10-SPIx_CS/TIM3_CH4`             | free, also on the Arduino header |
| PC10 | D2     | **`USB_FS_PWR_EN` / `USB_Disconnect`** | **conflict, see below**          |
| PC11 | D3     | plain IO                               | free                             |
| PC12 | CK     | plain IO                               | free                             |

**CK, CMD and GND — the only three wires involved in the CMD5 failure — are all
unencumbered.** So board routing does not explain the current symptom, and
hypothesis "PB2 is an unusual mapping" is dead. PB2 is at CN10 pin 22 and PB13
at CN10 pin 30; confirm the rest against the ST morpho table in UM3121.

**PC10 is a separate, real problem for 4-bit.** The board assigns it to the USB
full-speed power-enable / disconnect function, so there may be onboard circuitry
on that net.

There is **no alternative pin**. Per the CubeMX device database for
`STM32H533RETx` (LQFP64), the complete set of SDMMC1-capable pins is:

| Signal       | Pins available on LQFP64 |
| ------------ | ------------------------ |
| CMD          | PB2, PD2                 |
| CK           | PC12                     |
| D0           | PB13, PC8, PA10          |
| D1           | PC9                      |
| **D2**       | **PC10 only**            |
| D3           | PC11                     |
| CKIN / D4    | PB8                      |
| D0DIR / D6   | PC6                      |
| D123DIR / D7 | PC7                      |

So D2 cannot be relocated. The options are: confirm empirically whether PC10 is
actually loaded (configure it as a GPIO output, toggle, and check it swings rail
to rail on the morpho pin; or measure resistance to GND/3V3 unpowered), or stay
in 1-bit where D2 is unused.

This does not affect the present failure — only what happens after the link
comes up.

## Next steps

1. Logic analyzer or scope on PC12 (CK) and PB2 (CMD) during init.
   - Flat CK → host-side problem; the pins are known free, so suspect wiring.
   - Live CK and CMD frames → host is fine, the problem is at the ESP32.
2. No analyzer: reconfigure PB2/PB13/PC12 as GPIO outputs, toggle them, and
   check continuity all the way to GPIO15/GPIO2/GPIO14 on the ESP32 header.
3. Rebuild esp-at with `CONFIG_LOG_DEFAULT_LEVEL_DEBUG` and capture the boot log
   to the end. The IDF `sdio_slave` driver logs at init; its presence or absence
   settles hypothesis 3.
4. Once CK and CMD are proven good, re-check GND bonding before suspecting
   anything else.

## Notes for later

- Reflashing the ESP32 over serial needs GPIO2 low. The D0 pull-up defeats
  download mode, so lift it (or hold GPIO2 down) when flashing.
- The `esp-at` example's own advice is to bring up in 1-bit at the lowest
  practical clock first, then widen. `SDIO_PORT_BUS_WIDTH` in
  `Core/Src/at_sdio/platform/include/sdio.h` switches this; 1-bit needs only
  CK, CMD, D0 and D1 wired.
- README still records esp-at v4.1.1.0; the hardware reports v4.2.0.0.

## AT init

```shell
13:24:08.159 -> ets Jul 29 2019 12:21:46
13:24:08.159 ->
13:24:08.159 -> rst:0x1 (POWERON_RESET),boot:0x3f (SPI_FAST_FLASH_BOOT)
13:24:08.159 -> configsip: 0, SPIWP:0xee
13:24:08.159 -> clk_drv:0x00,q_drv:0x00,d_drv:0x00,cs0_drv:0x00,hd_drv:0x00,wp_drv:0x00
13:24:08.159 -> mode:DIO, clock div:2
13:24:08.159 -> load:0x3fff0030,len:5172
13:24:08.159 -> load:0x40078000,len:15908
13:24:08.159 -> load:0x40080400,len:4
13:24:08.159 -> ho 8 tail 4 room 4
13:24:08.159 -> load:0x40080404,len:3600
13:24:08.159 -> entry 0x400805fc
13:24:08.159 -> I (31) boot: ESP-IDF v5.4.4-dirty 2nd stage bootloader
13:24:08.159 -> I (31) boot: compile time Aug  8 2026 22:42:51
13:24:08.159 -> W (31) boot: Unicore bootloader
13:24:08.159 -> I (33) boot: chip revision: v3.0
13:24:08.223 -> I (35) boot.esp32: SPI Speed      : 40MHz
13:24:08.223 -> I (39) boot.esp32: SPI Mode       : DIO
13:24:08.223 -> I (43) boot.esp32: SPI Flash Size : 4MB
13:24:08.223 -> I (46) boot: Enabling RNG early entropy source...
13:24:08.223 -> I (51) boot: Partition Table:
13:24:08.223 -> I (53) boot: ## Label            Usage          Type ST Offset   Length
13:24:08.223 -> I (60) boot:  0 phy_init         RF data          01 01 0000f000 00001000
13:24:08.223 -> I (66) boot:  1 otadata          OTA data         01 00 00010000 00002000
13:24:08.223 -> I (73) boot:  2 nvs              WiFi data        01 02 00012000 0000e000
13:24:08.255 -> I (79) boot:  3 at_customize     unknown          40 00 00020000 000e0000
13:24:08.255 -> I (86) boot:  4 ota_0            OTA app          00 10 00100000 00180000
13:24:08.255 -> I (92) boot:  5 ota_1            OTA app          00 11 00280000 00180000
13:24:08.255 -> I (99) boot: End of partition table
13:24:08.255 -> I (102) esp_image: segment 0: paddr=00100020 vaddr=3f400020 size=13e68h ( 81512) map
13:24:08.320 -> I (137) esp_image: segment 1: paddr=00113e90 vaddr=3ffbdb60 size=05268h ( 21096) load
13:24:08.320 -> I (145) esp_image: segment 2: paddr=00119100 vaddr=40080000 size=06f18h ( 28440) load
13:24:08.320 -> I (157) esp_image: segment 3: paddr=00120020 vaddr=400d0020 size=13609ch (1269916) map
13:24:08.739 -> I (589) esp_image: segment 4: paddr=002560c4 vaddr=40086f18 size=141bch ( 82364) load
13:24:08.771 -> I (622) esp_image: segment 5: paddr=0026a288 vaddr=50000000 size=00094h (   148) load
13:24:08.803 -> I (635) boot: Loaded app from partition at offset 0x100000
13:24:08.803 -> I (635) boot: Disabling RNG early entropy source...
13:24:09.030 -> I (913) at-init: at param mode: 1
13:24:09.189 -> I (1032) at-init: module_name: WROOM-32
13:24:09.189 -> I (1035) at-init: max tx power=78, ret=0
13:24:09.189 -> I (1038) at-init: v4.2.0.0 (unknown)
```
