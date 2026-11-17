# SPEC — Air-Quality Dashboard + USB-Stick Logger on STM32F429I-DISC1
## Build variant: STM32CubeMX + CMake + arm-none-eabi-gcc

| Field | Value |
|---|---|
| Document | `docs/SPEC.md` |
| Revision | 0.1 (draft) |
| Status | Proposed |
| Toolchain | STM32CubeMX → CMake → Ninja → arm-none-eabi-gcc (same as `stm32f3disc/010_9dof_bmp280_cmake`) |
| Target | STM32F429I-DISC1 (MB1075, STM32F429ZIT6) |
| External parts | Sensirion SEN55 (PM/RH/T/VOC/NOx), Sensirion STCC4 (CO₂/T/RH), USB flash drive + Micro-AB OTG adapter |
| Reference | `../docs/um1670-discovery-kit-with-stm32f429zi-mcu-stmicroelectronics.pdf` (UM1670 Rev 6) |

---

## 1. Purpose

A stand-alone indoor air-quality station that:

1. Shows live readings and history graphs on the on-board 2.4" QVGA LCD.
2. Keeps recent history in the 8 MB external SDRAM.
3. Archives every sample to CSV files on a USB flash drive plugged into the OTG port (USB **host**, MSC class).
4. Never loses samples when the stick is removed: SDRAM holds a backlog that is flushed when a stick is re-inserted.

### 1.1 Goals

- G1. 1 Hz sampling of SEN55 and STCC4, timestamped.
- G2. Display stays responsive (≥ 20 fps on trend screen) while the stick is being written.
- G3. ≥ 24 h of 1 Hz data survives stick removal (held in SDRAM).
- G4. CSV files readable by any spreadsheet / `pandas.read_csv` without post-processing.
- G5. Same build/test discipline as `010`: CMake presets, host unit tests, size check, reproducible build.

### 1.2 Non-goals

- Persistence across power loss other than the USB stick (SDRAM is volatile; no battery-backed RTC on this board).
- Network connectivity, cloud upload.
- USB device mode (CDC/MSC device) — possible later, not in this spec.
- Calibrated AQI per any national standard; colour bands are indicative (§24.3).

---

## 2. Document structure

| Part | Sections | Scope |
|---|---|---|
| A — Build system | §3–§12 | hardware, CubeMX, CMake, tests. Anything not stated here is inherited from `010` SPEC §6–§17 |
| B — Product | §20–§27 | architecture, sensors, storage, UI, logging, test cases |

---

# Part A — Platform and build

## 3. System overview

```
          I2C3 (100 kHz, shared)
 SEN55 ──┐   0x69
 STCC4 ──┼── 0x64 ──┐
 STMPE811 (touch) ──┘ 0x41            ┌────────────── SDRAM 8 MB @0xD000_0000 ─────────────┐
                    │                  │ FB0 | FB1 | ring_1s (24 h) | ring_1m (30 d) | backlog │
                    ▼                  └──────────────▲───────────────────────────────────────┘
             ┌─────────────┐  FMC (SDCLK 84 MHz)      │
             │ STM32F429ZI │──────────────────────────┘
             │  168 MHz    │── LTDC + DMA2D ──► ILI9341 240×320 RGB565
             │  FreeRTOS   │── SPI5 (ILI9341 init only)
             │             │── OTG_HS core, internal FS PHY ──► CN6 ──► USB stick (FatFs)
             │             │── USART1 (VCP via ST-LINK) ──► console / time set
             └─────────────┘── PG13/PG14 LEDs, PA0 button
```

## 4. Hardware

### 4.1 On-board resources used (UM1670)

| Function | Peripheral | Pins | Notes |
|---|---|---|---|
| HSE | 8 MHz from ST-LINK MCO | PH0 | Fixed (UM1670 §7.12.1) |
| LSE | — | — | **Not fitted** (X2 "not provided", §7.12.2). RTC runs from LSI |
| SDRAM IS42S16400J | FMC SDRAM bank 2 | SDNE1 PB6, SDCKE1 PB5, + FMC data/addr | 1 M × 16 × 4 banks, 12-bit row, 8-bit col |
| LCD RGB | LTDC | per CubeMX board preset | 240×320, ILI9341 in RGB mode |
| LCD control | SPI5 + GPIO | SCK PF7, MOSI PF9, CSX PC2, WRX/DCX PD13 | Register init only; pixels go via LTDC |
| Touch STMPE811 | I2C3 | SCL PA8, SDA PC9, INT PA15 | Present on this board revision; optional input (§24.4) |
| USB OTG FS | OTG_HS core, embedded FS PHY | DM PB14, DP PB15, VBUS PB13 (LD5), OC PC5 (LD6), power-switch enable PC4 | Full-speed only (§7.7). PC4 polarity: check MB1075 schematic |
| VCP | USART1 | TX PA9, RX PA10 | Enabled by default via SB11/SB15 (§7.3.3) |
| LEDs | GPIO | LD3 green PG13, LD4 red PG14 | |
| Button | GPIO/EXTI | B1 PA0 | |

### 4.2 External wiring

| Sensor | Supply | Logic | I2C addr | Connect to |
|---|---|---|---|---|
| SEN55 | 5 V (board 5V pin) | 3.3 V I2C (SEL pin to GND) | 0x69 | PA8 / PC9 on P1/P2 headers |
| STCC4 | per datasheet (breakout) — verify | 3.3 V | 0x64 (default, configurable) | same bus |

- The STMPE811's I2C3 pull-ups are on the board; verify their value before adding more on the breakouts (target total ≈ 2.2–4.7 kΩ).
- Bus speed 100 kHz (SEN55 limit).
- USB stick connects through a Micro-AB → USB-A OTG adapter. The board must be powered over the ST-LINK USB (UM1670 §7.7 note).

### 4.3 Open hardware checks (close in M0)

- H1. PC4 power-switch enable: active level.
- H2. I2C3 pull-up value.
- H3. STCC4 supply range and whether it needs an external RH/T compensation input.

## 5. Non-functional requirements

| ID | Requirement |
|---|---|
| NFR-01 | Flash use ≤ 512 KB (of 2 MB) |
| NFR-02 | Internal SRAM use ≤ 160 KB (of 192 KB + 64 KB CCM); framebuffers and history must live in SDRAM |
| NFR-03 | Sample period 1000 ms ± 5 ms jitter (measured with GPIO + logic analyser) |
| NFR-04 | UI frame time ≤ 50 ms while a FatFs write is in progress |
| NFR-05 | Stick write: `f_sync` at least every 60 s; ≤ 60 s of data lost on surprise removal (it stays in backlog, §23.4) |
| NFR-06 | 72 h endurance with ≥ 10 stick remove/insert cycles, zero lost rows (T-12) |
| NFR-07 | No `malloc` outside FreeRTOS heap_4; FreeRTOS heap ≤ 32 KB |

## 6. Toolchain

Inherited from `010` SPEC §6: CubeMX ≥ 6.15, CMake ≥ 3.22, Ninja, arm-none-eabi-gcc (Fedora package), clang-format 23.1.5, Python 3 for tools. Flashing: `st-flash` or STM32_Programmer_CLI (chip id 0x419).

## 7. CubeMX generation

- Start from **Board Selector → STM32F429I-DISC1**, "initialize all peripherals with default mode": **No**. Use the board pin labels, then enable only the peripherals in §7.1.
- Toolchain/IDE: **CMake**. Generate peripheral init as pairs of `.c/.h`.
- Never edit the `.ioc` by hand (see `AGENTS.md`); changes come as a step-by-step guide.

### 7.1 Peripheral configuration

| Peripheral | Setting |
|---|---|
| RCC | HSE bypass (8 MHz MCO). PLLM 8, PLLN 336, PLLP 2 → 168 MHz, PLLQ 7 → 48 MHz USB |
| PLLSAI | N 192, R 4, DIVR 8 → LTDC pixel clock 6 MHz |
| FMC | SDRAM2, 4 banks, 12 row / 8 col bits, 16-bit, CAS 3, SDCLK = HCLK/2, read burst on, read pipe delay 1. Timings: TMRD 2, TXSR 7, TRAS 4, TRC 7, TWR 2, TRP 2, TRCD 2 (in SDCLK cycles) |
| LTDC | 240×320, HSYNC 10, HBP 20, HFP 10, VSYNC 2, VBP 2, VFP 4, layer 0 RGB565 @ 0xD000_0000 |
| DMA2D | enabled, interrupt on |
| SPI5 | master, TX only, 8-bit, ≤ 5.6 MHz |
| I2C3 | 100 kHz, DMA RX/TX optional |
| USB_OTG_HS | Host_Only, internal FS PHY, VBUS sensing on. Middleware: USB_HOST class MSC |
| FATFS | USB Disk, LFN on (static buffer), `_FS_REENTRANT` on, code page 437 |
| FREERTOS | CMSIS_V2, heap_4, 32 KB, `configUSE_NEWLIB_REENTRANT` on |
| RTC | LSI clock |
| USART1 | 115200 8N1, DMA RX to idle line |
| CRC | on (for record checksums) |
| SYS | Timebase TIM6 (SysTick goes to FreeRTOS) |

## 8. Repository layout

```
001_aq_logger/
├── 001_aq_logger.ioc          CubeMX (never by hand)
├── CMakeLists.txt             user-owned root
├── CMakePresets.json
├── cmake/                     toolchain, build_info, stm32cubemx/ (generated)
├── Core/ Drivers/ Middlewares/ FATFS/ USB_HOST/   generated; USER CODE blocks only
├── Bsp/{Inc,Src}              sdram, lcd (ili9341+ltdc), dma2d, i2c bus, usb power, time
├── Sensors/{Inc,Src}          sen55, stcc4, stmpe811 (portable, HAL only via Bsp)
├── Store/{Inc,Src}            record, ring buffers, decimator, backlog
├── Log/{Inc,Src}              csv formatter, file rotation, FatFs glue
├── Gui/{Inc,Src}              font, widgets, screens
├── App/{Inc,Src}              tasks, console, state machine
├── tests/                     host CTest tree
├── tools/                     size_check.py, csv_check.py, plot.py, regenerate.txt
└── docs/                      SPEC.md, PLAN.md, hardware.md, test-log.md
```

## 9. CMake

As `010` §9: generated `stm32cubemx` INTERFACE target; our STATIC libraries `bsp`, `sensors`, `store`, `log`, `gui`, `app`; post-build `.hex/.bin`, size check against NFR-01/02; `flash` custom target.

The linker script gains an `.sdram (NOLOAD)` section at `0xD000_0000`, length 8 MB, so that SDRAM buffers are declared as `__attribute__((section(".sdram")))` statics. Nothing in `.sdram` is touched before `bsp_sdram_init()` returns. The linker-script edit sits in a user-owned copy, `STM32F429ZITX_FLASH_AQ.ld`, referenced from the root `CMakeLists.txt`, so that CubeMX regeneration does not overwrite it.

## 10. Host tests

Portable modules (`Sensors/` parsing + CRC, `Store/`, `Log/` formatter, `Gui/` layout math and band classification) build under `HOST_TEST` with stubs. Every numeric routine gets test vectors.

## 11. Acceptance criteria (build)

- AC-01 Clean clone → `cmake --preset Debug && cmake --build --preset Debug` succeeds with zero warnings in our code.
- AC-02 `ctest` green.
- AC-03 Size check enforces NFR-01/02.
- AC-04 CubeMX regeneration (`tools/regenerate.txt`) leaves `git diff` empty outside generated dirs.

## 12. Risks

| Risk | Mitigation |
|---|---|
| FatFs/USB host stalls block the UI | Separate logger task at lower priority; UI never waits on FatFs |
| Cheap sticks fail to enumerate at FS / draw too much current | Keep a known-good stick list in `hardware.md`; LD6 overcurrent is surfaced on the status screen |
| LSI RTC drift (±5 %) | Time set at boot from `TIME.TXT` or VCP; status shows "time unset"; rows also carry uptime |
| I2C3 shared with touch | Single bus owner task (§21.3); touch is polled through the same task |
| SDRAM not initialised before use | `.sdram` is NOLOAD, init in `main` before the scheduler starts, memtest at boot (§22.2) |

---

# Part B — Product

## 20. Data model

### 20.1 Record (32 bytes, little-endian, stored in SDRAM)

| Off | Type | Field | Unit / scale |
|---|---|---|---|
| 0 | u32 | `t_epoch` | s since 1970 UTC (0 if time unset) |
| 4 | u32 | `uptime_s` | s since boot |
| 8 | u16 ×4 | `pm1, pm2_5, pm4, pm10` | µg/m³ × 10 (SEN55 native) |
| 16 | i16 | `t_sen` | °C × 200 (SEN55 native) |
| 18 | i16 | `rh_sen` | % × 100 |
| 20 | i16 | `voc_idx` | × 10 |
| 22 | i16 | `nox_idx` | × 10 |
| 24 | u16 | `co2_ppm` | ppm |
| 26 | i16 | `t_stcc` | °C × 100 |
| 28 | u16 | `rh_stcc` | % × 100 |
| 30 | u16 | `flags` | §20.2 |

Raw sensor scales are kept; conversion to engineering units happens only in the GUI and the CSV formatter.

### 20.2 Flags

| Bit | Meaning |
|---|---|
| 0 | SEN55 invalid (CRC, NACK, or not ready) |
| 1 | STCC4 invalid |
| 2 | Time unset |
| 3 | SEN55 warming up (first 60 s; NOx index needs a longer warm-up) |
| 4 | STCC4 warming up |
| 5 | Record is a 1-min average (only in `ring_1m`) |
| 15..8 | Reserved = 0 |

## 21. Software architecture

### 21.1 Layers

`Bsp` (HAL wrappers) ← `Sensors` / `Store` / `Log` / `Gui` (portable) ← `App` (tasks, wiring).

### 21.2 Tasks (FreeRTOS)

| Task | Prio | Period | Job |
|---|---|---|---|
| `sense` | high | 1 s (`vTaskDelayUntil`) | Read SEN55 + STCC4, build record, push to `ring_1s`, decimator, `backlog`; poll touch at 20 Hz in between |
| `ui` | normal | 50 ms | Render the current screen to the back buffer with DMA2D, swap on LTDC reload at vblank |
| `usbh` | normal | event | `USBH_Process`, connect/disconnect events |
| `logger` | low | event + 1 s | Drain `backlog` to the CSV file, rotate files, `f_sync` (NFR-05) |
| `console` | low | event | VCP commands (§26) |

### 21.3 Ownership rules

- I2C3 belongs to `sense` only. Touch events reach `ui` through a queue.
- `ring_1s`, `ring_1m`: single writer (`sense`), readers take a snapshot of head/count under a short critical section.
- `backlog`: single writer (`sense`), single reader (`logger`), lock-free SPSC ring.
- FatFs is used by `logger` only.

## 22. SDRAM

### 22.1 Map

| Region | Offset | Size | Content |
|---|---|---|---|
| FB0 | 0x00_0000 | 150 KB (240×320×2) | front/back buffer |
| FB1 | 0x02_5800 | 150 KB | front/back buffer |
| `ring_1s` | 0x04_B000 | 86 400 × 32 = 2.64 MB | last 24 h at 1 Hz |
| `ring_1m` | 0x2E_E000 | 43 200 × 32 = 1.32 MB | last 30 d of 1-min averages |
| `backlog` | 0x43_F800 | 86 400 × 32 = 2.64 MB | records not yet written to the stick |
| spare | 0x6E_2800 | ≈ 1.1 MB | glyph cache, future |

`ring_1s` and `backlog` are separate on purpose: the display history must not depend on whether the stick has caught up.

### 22.2 Boot test

A fast address-line and data-line test (≤ 200 ms) runs before the scheduler starts. On failure: LD4 on, VCP message, halt. A full memtest runs on the `M` console command only.

## 23. Logging to USB stick

### 23.1 Files

- Directory `/AQLOG/`.
- Time set: one file per UTC day, `/AQLOG/YYYYMMDD.CSV`.
- Time unset: one file per boot, `/AQLOG/BOOTnnnn.CSV` (`nnnn` = first unused number).
- On first creation, write a header line:
  `epoch,uptime_s,pm1,pm2_5,pm4,pm10,t_sen_c,rh_sen,voc_idx,nox_idx,co2_ppm,t_stcc_c,rh_stcc,flags`
- Values are written in engineering units with fixed decimals (PM 1 dp, T 2 dp, RH 2 dp, indices 1 dp). An invalid sensor leaves its fields empty.

### 23.2 Write policy

- `logger` drains up to 512 records per wake into a 16 KB static line buffer, then calls `f_write`.
- `f_sync` every 60 s or after 512 records.
- Day roll-over: close, open the new file, continue.

### 23.3 Stick state machine

`NO_STICK → ENUMERATING → MOUNTED → (WRITING ⇄ IDLE) → ERROR | NO_STICK`

- `ERROR` (mount failure, `FR_DISK_ERR`, disk full): stop writing, keep filling the backlog, retry the mount every 10 s, show the reason on the status screen.
- Overcurrent (PC5): disable the port power, enter `ERROR`, light LD4.

### 23.4 Backlog semantics

- A record leaves `backlog` only after the `f_sync` that covers it returns `FR_OK`. Until then the tail pointer stays put, so unsynced records are rewritten after surprise removal. Duplicates are possible after removal; downstream deduplicates on `uptime_s` (`tools/csv_check.py`).
- Backlog full (> 24 h without a stick): drop the oldest records and count them in a `dropped` counter shown on the status screen.

### 23.5 Time

- At mount, if `/TIME.TXT` exists and holds an ISO-8601 UTC timestamp, set the RTC from it, then rename the file to `TIME.OLD` so the time is not re-applied on the next mount.
- The VCP command `T<epoch>` also sets the time.
- Records captured before the time was set keep `t_epoch = 0`. No back-filling.

## 24. Display

### 24.1 Screens

| Screen | Content |
|---|---|
| **Now** | Large tiles: PM2.5, CO₂, VOC idx, NOx idx; small tiles: PM1/4/10, T, RH. Each tile colour-banded (§24.3). |
| **Trend** | One channel, line graph, window 1 h / 24 h (from `ring_1s`) or 7 d / 30 d (from `ring_1m`), min/max/avg labels |
| **Status** | Stick state, current file, rows written, backlog depth, dropped count, free space, sensor status, time source, uptime, build id |

### 24.2 Navigation

- B1 short press: next screen. B1 long press (> 1 s) on Trend: next channel.
- Touch (if the STMPE811 responds at boot): tap the left/right third to change screen; tap the graph to change window.

### 24.3 Colour bands (indicative)

| Channel | Good | Moderate | Poor | Bad |
|---|---|---|---|---|
| PM2.5 µg/m³ | < 12 | 12–35 | 35–55 | ≥ 55 |
| CO₂ ppm | < 800 | 800–1200 | 1200–2000 | ≥ 2000 |
| VOC idx | < 150 | 150–250 | 250–400 | ≥ 400 |
| NOx idx | < 20 | 20–150 | 150–300 | ≥ 300 |

Thresholds live in one table in `Gui/` and are unit-tested.

### 24.4 Rendering

- RGB565 double buffer. Swap with an LTDC shadow reload at vertical blanking.
- DMA2D for rectangle fills and glyph blits (A8 font with colour fill).
- Fonts: two bitmap sizes generated offline to C arrays in flash.

## 25. Sensors

### 25.1 SEN55

- Start measurement once at boot. Read data-ready, then measured values, every second.
- Sensirion CRC-8 (poly 0x31, init 0xFF) on every word.
- Fan cleaning: use the default weekly interval.
- Port from the existing `nucleo-h533re/001_SEN55` work, keeping all HAL calls behind `Bsp/`.

### 25.2 STCC4

- Default address 0x64. Continuous measurement mode.
- If H3 confirms the STCC4 accepts external RH/T compensation, feed it the SEN55 RH/T each cycle.
- Same CRC-8 scheme (verify in datasheet).

### 25.3 Fault handling

- 3 consecutive failures: flag the sensor invalid, then attempt a re-init every 10 s. Other sensors keep running.
- I2C bus stuck (SDA low): 9-clock recovery on PA8, then re-init I2C3.

## 26. Console (USART1 VCP, line-based)

| Cmd | Action |
|---|---|
| `T<epoch>` | Set RTC |
| `S` | Print status |
| `L` | Print last record |
| `M` | Run full SDRAM memtest (stops logging while it runs, then resumes) |
| `E` | Safely eject stick (`f_sync`, unmount, power off port) |
| `V` | Version / build id |

## 27. Test cases

| ID | Test | Pass |
|---|---|---|
| T-01 | SDRAM boot test + full memtest | no errors, ≥ 60 MB/s sequential write |
| T-02 | LCD tear-free double buffering | no tearing on a moving bar, 60 s |
| T-03 | SEN55 + STCC4 at 1 Hz on VCP | values plausible vs reference; NFR-03 jitter |
| T-04 | Sensor unplug/replug | flags set within 3 s, recovery within 15 s |
| T-05 | Ring buffers + decimator | host tests; on target 1-min average matches a PC recomputation |
| T-06 | Now/Trend/Status screens | NFR-04 frame time while logging |
| T-07 | Stick enumerate + mount + CSV header | file opens on PC, header correct |
| T-08 | Day roll-over | set time to 23:59:30, two files with continuous rows |
| T-09 | Surprise removal | no rows lost (duplicates allowed), `csv_check.py` passes |
| T-10 | No stick for 2 h, then insert | full backlog written, ordered |
| T-11 | Stick full | `ERROR` shown, backlog grows, no crash |
| T-12 | 72 h endurance, 10 remove/insert cycles | NFR-06 |
| T-13 | `TIME.TXT` time set | RTC set, file renamed |
| T-14 | Overcurrent | port off, LD4, status shows OC |

## 28. Change log

| Rev | Date | Change |
|---|---|---|
| 0.1 | 2026-11-13 | First draft |
