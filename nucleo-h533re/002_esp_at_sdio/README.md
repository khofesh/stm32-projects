# STM32 with esp-at via SDIO

- https://github.com/espressif/esp-at/tree/release/v4.1.0.0/examples/at_sdio_host
- https://docs.espressif.com/projects/esp-idf/en/latest/esp32c6/api-reference/peripherals/sdio_slave.html

## boards used

- nucleo-H533RE
- ESP32 with esp-at (master) - https://github.com/espressif/esp-at/blob/master/examples/at_sdio_host/README.md

## pin mapping

Host is SDMMC1 in SDIO mode, 4-bit. The ESP32 SDIO slave pins are fixed by the
chip and cannot be remapped.

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

51 kOhm to 3.3 V on CMD, D0, D1, D2 and D3. CK needs none.

### ESP32 strapping pins

Three of the SDIO pins are sampled at reset:

- **GPIO12 (D2)** selects VDD_SDIO. The 51 kOhm pull-up required by SDIO reads
  as 1.8 V flash and a 3.3 V module then fails to boot. Burn the eFuse once so
  the strapping is ignored:

  ```
  espefuse.py --port <port> set_flash_voltage 3.3V
  ```

  This is irreversible. Without it, 4-bit mode cannot work on ESP32.

- **GPIO2 (D0)** must be low to enter serial download mode, so the pull-up has
  to be removed (or the pin held low) when flashing.
- **GPIO15 (CMD)** pulled low silences the ROM boot log; the SDIO pull-up keeps
  it high, which is the normal case.

### bus width

`SDIO_PORT_BUS_WIDTH` in `Core/Src/at_sdio/platform/include/sdio.h` selects the
width. Enumeration always runs in 1-bit; the port widens the link afterwards by
writing CCCR 0x07 itself, because `HAL_SDIO_Init()` compares `Init.BusWide` (an
`SDMMC_BUS_WIDE_*` register value) against `HAL_SDIO_4_WIRES_MODE` (1) and so
never writes 4-bit into the slave.
