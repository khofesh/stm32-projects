# STM32 with esp-at via SDIO

- https://github.com/espressif/esp-at/tree/release/v4.1.0.0/examples/at_sdio_host
- https://docs.espressif.com/projects/esp-idf/en/latest/esp32c6/api-reference/peripherals/sdio_slave.html

## boards used

- nucleo-H533RE
- ESP32 with esp-at (master) - https://github.com/espressif/esp-at/blob/master/examples/at_sdio_host/README.md

## building esp-at

```shell
git clone --recursive https://github.com/espressif/esp-at.git
cd esp-at
HAS_IDF_PREREQUISITES=1 ./build.py install
. ./esp-idf/export.sh
./build.py menuconfig
# enable sdio
./build.py build
./build.py -p /dev/ttyACM0 flash
```

## pin mapping

Host is SDMMC1 in SDIO mode. The bus is currently held at 1-bit for bring-up
(see `### bus width`). In 1-bit mode only CK, CMD and D0 carry data; D1 is still
needed for the SDIO interrupt. D2 and D3 may remain wired, but they are not used
until 4-bit mode is enabled. The ESP32 SDIO slave pins are fixed by the chip and
cannot be remapped.

| Signal | STM32H533 | AF   | ESP32  | ESP32C6 |
| ------ | --------- | ---- | ------ | ------- |
| CK     | PC12      | AF12 | GPIO14 | GPIO19  |
| CMD    | PB2       | AF12 | GPIO15 | GPIO18  |
| D0     | PB13      | AF12 | GPIO2  | GPIO20  |
| D1     | PC9       | AF12 | GPIO4  | GPIO21  |
| D2     | PC10      | AF12 | GPIO12 | GPIO22  |
| D3     | PC11      | AF12 | GPIO13 | GPIO23  |
| RST    | PB15      | -    | EN     | EN      |
| GND    | GND       | -    | GND    | GND     |

PB15 (CN10-26) is a plain open-drain output, not an SDMMC signal. The host
pulses EN low for 20 ms at every `sdio_driver_init()` and then waits 1.5 s for
esp-at to reach `sdio_slave_start`, so a host restart always meets a freshly
booted slave. Without it the slave keeps the RCA and enabled function from the
previous session and ignores the CMD0 that opens enumeration - an I/O-only card
is not required to honour CMD0 - which is why the link came up only sometimes.
Open-drain matters: the devkit pulls EN up and its USB bridge drives EN low for
its own auto-reset, so the host must only ever pull down.

see the sdio pins for esp32c6: https://docs.espressif.com/projects/esp-at/en/latest/esp32c6/Compile_and_Develop/How_to_implement_SDIO_AT.html#introduction

For ESP32-C6, D3 is **GPIO23 only**. Do not connect STM32 PC11/D3 to GPIO15. If
GPIO15 is not connected on your Waveshare ESP32-C6 board, there is no GPIO15
bridge to remove.

D1 also carries the SDIO card interrupt the slave uses to signal the host, so it
has to be wired even though it is not used for 1-bit data.

### pull-ups

10 kOhm to 3.3 V on CMD, D0, D1, D2 and D3, as the esp-at example specifies. CK
needs none.

`SDIO_PORT_PIN_DIAG` in `Core/Src/at_sdio/platform/include/sdio.h` checks these
at every boot and `sdio_driver_init()` stops before clocking the bus if a net is
floating or cannot be driven low. It currently passes; the slave answers CMD5
but the response comes back bit-shifted, see `docs/ERROR.md`. The resistors are
soldered in place, so the two strapping consequences below are permanent.

### ESP32 strapping pins

Three of the SDIO pins are sampled at reset:

- **GPIO12 (D2)** selects VDD_SDIO. The pull-up required by SDIO reads as 1.8 V
  flash and a 3.3 V module then fails to boot. Burn the eFuse once so the
  strapping is ignored:

  ```
  espefuse.py --port <port> set_flash_voltage 3.3V
  ```

  Irreversible, and load-bearing: with the pull-up soldered on, the module does
  not boot without it. Check the module is a 3.3 V flash part (WROOM-32/-32D/
  -32E/-32U, PICO-D4, WROVER-B/-E) before burning; the original 1.8 V WROVER
  would be destroyed.

- **GPIO2 (D0)** must be low or floating to enter serial download mode, and the
  soldered pull-up holds it high. **Short GPIO2 to GND while flashing the ESP32**
  or the bootloader will not come up.
- **GPIO15 (CMD)** pulled low silences the ROM boot log; the SDIO pull-up keeps
  it high, which is the normal case.

### bus width

`SDIO_PORT_BUS_WIDTH` in `Core/Src/at_sdio/platform/include/sdio.h` selects the
width. Currently `HAL_SDIO_1_WIRE_MODE` for bring-up.

Enumeration always runs in 1-bit regardless; for 4 wires the port widens the
link afterwards by writing CCCR 0x07 itself, because `HAL_SDIO_Init()` compares
`Init.BusWide` (an `SDMMC_BUS_WIDE_*` register value) against
`HAL_SDIO_4_WIRES_MODE` (1) and so never writes 4-bit into the slave.

Switching between the two needs no hardware change — D1-D3 keep their pull-ups
and stay wired, the SDMMC just never drives them in 1-bit.

Before moving to 4 wires, note that D2 lands on **PC10**, which the
NUCLEO-H533RE assigns to `USB_FS_PWR_EN`/`USB_Disconnect`. PC10 is the only
SDMMC1_D2 pin on the LQFP64 package, so D2 cannot be relocated — check what is
actually on that net first. See `ERROR.md`.

## why enumeration used to fail

`HAL_SDIO_Init()` recomputes `Init.ClockDiv` from a private `SDIO_INIT_FREQ` of
400 kHz and runs the whole CMD0/CMD5/CMD3/CMD7 identification there, restoring
`hsdio1.Init.ClockDiv` only afterwards. This interconnect does not carry a frame
at 400 kHz - a bit-banged CMD5 gets a valid R4 at 25 kHz and nothing at 400 kHz -
so identification always timed out, and every host-side sweep (pad slew, CLKDIV,
NEGEDGE, HWFC) changed nothing because all of them lived outside the window the
HAL re-initialises.

The fix is `SdioIdentifyCardSlow()`, registered on the `SDIO_IdentifyCard` hook,
which puts the clock back to `SDIO_PORT_CLOCK_HZ`, gives the slave its 74
power-up clocks, and retries each command instead of failing the whole attempt.
Measured 12/12 clean inits, including the 4-bit widen.

Ruled out along the way, each by measurement rather than argument: wiring (all
six verified pad-to-pad by reading `GPIO_IN` on the slave), stale slave state
(the PB15 EN pulse reboots it every run), host transmission, DAT3/SPI-mode
latch, sampling phase, and CMD push-pull contention.

25 kHz is a crutch. The slave is fine and the host is fine; the wires are the
limit. Shorten them, give each signal a ground return, then walk
`SDIO_PORT_CLOCK_HZ` back up and set `SDIO_PORT_SLOW_KERNEL_CLK` to 0.

For 4-bit ESP32-C6 traffic, verify D3 before enabling it: STM32 PC11 must have
continuity to ESP32-C6 GPIO23 and no continuity to GPIO15. GPIO15 is the
JTAG-source-select strap and must not be tied to D3.

## if successful

```shell

host ready, start initializing slave...
E sdio_driver: pin diag: control CTRL(PA5) drive0=ok drive1=ok
E sdio_driver: pin diag: pull-up census (expect 'high' on CMD and D0-D3)
E sdio_driver:   CMD(PB2): high  (something pulls this net up)
E sdio_driver:   D0(PB13): high  (something pulls this net up)
E sdio_driver:   D1(PC9): high  (something pulls this net up)
E sdio_driver:   D2(PC10): high  (something pulls this net up)
E sdio_driver:   D3(PC11): high  (something pulls this net up)
E sdio_driver: pin diag: short matrix
E sdio_driver: pin diag: wiring looks sane from the host side
I sdio_driver: [Clock] kernel=250000000Hz, div=312, SDMMC_CK=400641Hz
I sdio_driver: slave enumerated after 1 retries (500 ms)
esp_slave_init_io
D sdio_transport: IOE: 0x02
D sdio_transport: IOR: 0x06
D sdio_transport: IOE: 0x06
D sdio_transport: IOE: 0x06
D sdio_transport: IE: 0x03
D sdio_transport: IE: 0x07
D sdio_transport: Function 0 BSL: 0x00
D sdio_transport: Function 0 BSH: 0x02
D sdio_transport: Function 1 BSL: 0x00
D sdio_transport: Function 1 BSH: 0x02
D sdio_transport: Function 2 BSL: 0x00
D sdio_transport: Function 2 BSH: 0x02
Sdio init done
```
