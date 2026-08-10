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
./build.py menuconfig
# enable sdio
./build.py build
./build.py -p /dev/ttyACM0 flash
```

## pin mapping

Host is SDMMC1 in SDIO mode. All six lines are wired; the bus is currently held
at 1-bit for bring-up (see `### bus width`). The ESP32 SDIO slave pins are fixed
by the chip and cannot be remapped.

| Signal | STM32H533 | AF   | ESP32  |
| ------ | --------- | ---- | ------ |
| CK     | PC12      | AF12 | GPIO14 |
| CMD    | PB2       | AF12 | GPIO15 |
| D0     | PB13      | AF12 | GPIO2  |
| D1     | PC9       | AF12 | GPIO4  |
| D2     | PC10      | AF12 | GPIO12 |
| D3     | PC11      | AF12 | GPIO13 |
| GND    | GND       | -    | GND    |

D1 also carries the SDIO card interrupt the slave uses to signal the host, so
it has to be wired even though it is not needed for 1-bit data.

### pull-ups

10 kOhm to 3.3 V on CMD, D0, D1, D2 and D3, as the esp-at example specifies. CK
needs none. These are soldered in place, so the two strapping consequences below
are permanent.

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
