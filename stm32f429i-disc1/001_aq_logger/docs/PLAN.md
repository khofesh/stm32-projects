# PLAN — implementing `docs/SPEC.md`

Target: STM32F429I-DISC1 + SEN55 + STCC4 + USB stick (MSC host).
Each milestone ends with: build green, `ctest` green, size check, entry in `docs/test-log.md`, commit.

## 1. Current state

- Only `docs/` exists. No `.ioc`, no code.
- The board attached to the bench right now is the STM32F3DISCOVERY (st-info: chip id 0x422). Connect the F429I-DISC1 (chip id 0x419) before M0.
- Reusable from the repo: SEN55 driver logic (`nucleo-h533re/001_SEN55`), CMake/presets/tools/tests skeleton (`stm32f3disc/010_9dof_bmp280_cmake`), FreeRTOS (`freertos/`).

## 2. Decisions

| # | Decision | Why |
|---|---|---|
| D1 | FreeRTOS (CMSIS v2) | FatFs on a USB stick blocks for 10–200 ms. The UI and 1 Hz sampling must not wait on it |
| D2 | USB **host** MSC, stick is the only archive | User choice. Gives persistence without flash wear |
| D3 | SDRAM holds display history and the backlog in separate rings | Display history must not depend on stick state. Surprise removal loses nothing |
| D4 | Bare LTDC + DMA2D + own widgets, no LVGL (for now) | 3 fixed screens. Keeps flash small. Can migrate later |
| D5 | Raw sensor scales stored, units applied on output | Smaller records. No float rounding in the stored data |
| D6 | CubeMX board preset STM32F429I-DISC1 | Pins come pre-labelled. Avoids LTDC/FMC pin mistakes |
| D7 | 168 MHz, not 180 | The F429 USB 48 MHz clock comes only from PLLQ, and 168/336/7 gives exactly 48 MHz |

## 3. Milestones

### M0 — Build skeleton (AC-01…AC-04)
- CubeMX project from the board preset, CMake output, peripherals per SPEC §7.1. I write the step-by-step CubeMX guide; you click through it.
- Copy the CMake skeleton, presets, tools and tests tree from `010`, then rename.
- User-owned linker script with a `.sdram` NOLOAD section.
- LD3 blink from a FreeRTOS task, VCP banner with build id.
- Close the open hardware checks H1–H3.

### M1 — SDRAM (T-01)
- `bsp_sdram_init()`: FMC init sequence (clock enable, PALL, 8 auto-refresh, load mode register CAS 3 / burst 1, refresh count 1292 @ 84 MHz).
- Boot test + console `M` full memtest + throughput measurement.

### M2 — Display (T-02)
- ILI9341 RGB-mode init over SPI5, LTDC layer on FB0/FB1, vblank swap.
- DMA2D fill/blit, A8 font rendering, frame-time measurement on a GPIO.

### M3 — Sensors (T-03, T-04)
- I2C3 bus wrapper with bus recovery. Sensirion CRC-8 (host-tested).
- SEN55 driver (port), STCC4 driver (new, from the datasheet). `sense` task builds records and prints them on the VCP.
- Probe STMPE811. Polled touch.

### M4 — History + dashboard (T-05, T-06)
- `Store/`: SPSC rings, 1-min decimator (host-tested).
- Now / Trend / Status screens, colour bands (host-tested), B1 + touch navigation.

### M5 — USB stick logging (T-07…T-11, T-13, T-14)
- USB host MSC + FatFs mount/unmount on connect events, port power, OC handling.
- CSV formatter (host-tested against golden lines), file naming and roll-over, `TIME.TXT`.
- Backlog drain with the sync-then-advance rule. `tools/csv_check.py` for gaps and duplicates.

### M6 — Hardening (T-12)
- 72 h endurance with remove/insert cycles, FreeRTOS stack high-water marks, heap watermark on the Status screen.
- Fault-injection: unplug sensors, full stick, stuck I2C bus.

## 4. Spec updates to make along the way

- After M0: fill in the H1–H3 answers (PC4 polarity, pull-up value, STCC4 supply and compensation).
- After M1: the measured SDRAM throughput goes into T-01.
- After M3: STCC4 measurement interval and warm-up time from real data.

## 5. Risks specific to this plan

- STCC4 is a new part with no driver in the repo. Budget extra time in M3, and check whether Sensirion publishes an `embedded-i2c-stcc4` driver before writing one.
- The CubeMX F4 USB host + FatFs + FreeRTOS combination has known stack-size pitfalls. Start `usbh`/`logger` at 2 KB stacks and measure.
- Some USB sticks will not enumerate at full speed. Keep a tested-stick list in `hardware.md`.
