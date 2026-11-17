# Test log

Hardware records for `010_9dof_bmp280_cmake`. Host tests are summarised from
`ctest`; commands and build sizes are in the entries below.

## 2026-11-17 — M6 SPI2 slave link

F3 firmware: `74d3873` (Release, `-O2`), flashed with STM32_Programmer_CLI
(ST-LINK SN `066BFF494877514867130941`). Master: NUCLEO-L432KC running
`../010_stm32l4_spi_master` at `74d3873` (Release), SPI1 at 4 MHz, mode 0.
Wiring per `docs/hardware.md`, with SCK, MISO, MOSI, NSS, DRDY and GND
connected. The D2–GND parking jumper on the Nucleo was removed: D2 is the
master's MOSI.

### Build, host tests, resources

| Item | Result |
|---|---|
| `ci` build, `-Werror` | pass; FLASH 34 180 B, RAM 4 776 B |
| Release resources | FLASH 41 788 B, RAM 4 812 B |
| `ctest --test-dir build/tests` | 2 625 checks, 0 failures |
| Master host tests (`frame_check`) | 25 checks, 0 failures |

### T-16 SPI2 slave link (10 min)

`tools/t16_compare.py --f3 /dev/ttyACM0 --l4 /dev/ttyACM1 --duration 600 --pairs-at 300`
(in the master project). The master reads 84 bytes on every DRDY edge and
forwards each valid frame on its own VCP. The script matches those frames to
the F3 UART frames by sequence number. At 300 s it starts 100 pairs: a 40-byte
read, then a full read of the same frame. Frames from the first 2 s and the
last 5 s are left out of the matching.

| Check | Result |
|---|---|
| DRDY edges / full reads | 29 987 / 29 987 (50 Hz) |
| Valid frames | 29 987; 0 CRC, 0 header, 0 zero-frame errors |
| Sequence gaps / duplicates | 0 / 0 |
| Missed DRDY edges / master HAL errors | 0 / 0 |
| SPI vs UART frames, same sequence | 29 843 byte-identical, 0 mismatches, 0 unmatched on either side |
| Partial (40 B) → full read pairs | 100 done, 0 failures |
| DRDY edge → NSS low (master) | max 13 µs |
| NSS low duration, full read | max 196 µs |
| F3 debug counters | full transactions +50/s; partial stayed at 4, all counted before the run started |
| F3 CPU load | 6.6 % (all 600 debug frames) |
| F3 max loop time | 1 034 µs |
| F3 free stack (high-water) | 448 B |

Data integrity, sequence continuity, frame identity with the UART and
recovery after partial reads all pass.

Not closed:

| Item | Reason |
|---|---|
| NFR-17 (NSS rise → re-armed < 20 µs) | Needs the logic analyser on NSS + F3 PC0 and the PC0 probe CubeMX step (`docs/toolchain.md`) |
| CPU/T-12 "unchanged ±0.5 %" | No like-for-like baseline. M5's 7.4 % was a Debug (`-Og`) build; this run is Release (`-O2`). Repeat with the same build and the master disconnected |
| Free stack 448 B vs 520 B at M5 | Cause not measured; the same build-type caveat applies. Still above zero with margin |

### Bench notes

- The first master run read every frame with byte 0 corrupted (`0xAA` → `0xFA`/`0xFE`/`0xFF`, MISO high for the first 2–7 bits) and the other 83 bytes correct. The same build read cleanly after a reflash, and no timing change was needed. Not reproduced; cause unconfirmed.
- The L432KC often keeps `FLASH_SR.PEMPTY` set after programming, so a reset boots the ROM bootloader and the master sends nothing. Clear the bit (write 1 to `0x40022010` bit 17) and reset; steps are in the master's `AGENTS.md`.

## 2026-11-13 — M5 hardening

Firmware: `688e322-dirty` (Debug), flashed with OpenOCD. Board flat on the
bench, BMP280 at `0x77`; stored calibration (gyro bias + baro `p0`) was loaded
at boot, so `status` = `0x849F` (sensors OK, gyro calibrated, baro reference,
IMU-only, debug build). The run started with a late `R` command arriving when
the capture port opened; the pre-reset 16 frames are excluded, the stable
epoch below is continuous.

### Build, host tests, resources

| Item | Result |
|---|---|
| `cmake --preset ci && cmake --build --preset ci` | pass, `-Werror` clean |
| `ctest --test-dir build/tests --output-on-failure` | 621 checks, 0 failures, 0.06 s |
| `format-check`, `cppcheck` (2.13 and 2.22) | pass |
| `analyze` (`-fanalyzer`) | pass, no diagnostics in our sources |
| Debug resources | FLASH 42 272 B (64.5 %), RAM 4 284 B (34.9 %) |
| `ci` resources | FLASH 33 216 B (50.7 %), RAM 4 248 B (34.6 %) |
| `tools/repro_check.sh` | byte-identical `.elf/.hex/.bin` from two paths |

### T-12 endurance / T-13 resources (30 min)

Stable epoch: `t = 2.9 … 1812.5 s` (30.16 min), 90 478 data frames, 1 811 debug
frames.

| Check | Result |
|---|---|
| Telemetry rate | 50.0000 Hz (exact over 1 809.5 s) |
| Sequence gaps within epoch | 0 |
| CRC / length errors within epoch | 0 / 0 |
| Frames skipped / task overruns | 0 / 0 |
| Bus errors (I2C1, I2C2, SPI1) | 0 / 0 / 0 |
| CPU load | 7.4 % (mean and max) |
| Max loop time | 1 202 µs |
| Free stack (high-water) | 520 B |
| Flash / RAM | 33.2 KB / 4.2 KB (`ci`) |

The whole capture (including the reset boundary) reports 1 CRC error and the
sequence discontinuity at the reset; neither occurs inside the stable epoch.
T-12 and T-13 pass. Free-stack value is the untouched part of the 1 KB stack
reservation (`_sstack`), painted at boot.

### T-09 altitude noise

60 s window, 300–360 s after the epoch start: mean 0.146 m, σ = 0.053 m, range
0.268 m (NFR-06 < 0.1 m). `B` reference was already stored (status bit 7), so
this is relative altitude against the saved `p0`. Pass.

### T-10 altitude drift — fail

15 min window after settling (means of the first and last 60 s):

| Metric | Result |
|---|---|
| Altitude change | −2.35 m (NFR-07 < 0.5 m) |
| Linear slope | −2.68 m per 15 min |
| Pressure change | +36.9 Pa per 15 min (100 620 → 100 657 Pa) |
| Temperature | 34.30 °C, slope +0.008 °C per 15 min |

3-minute altitude σ stays between 0.06 and 0.37 m, so the filter is not noisy;
the drift follows a real ambient pressure rise. NFR-07 is not met on this bench
without a pressure-stable enclosure or a drift-compensation requirement change.

### Pending hardware checks

| Test | Reason |
|---|---|
| T-11 latency | PC0 `LAT_PROBE` needs the CubeMX step in `docs/toolchain.md`; logic analyser capture not run |
| T-14 fault recovery | Requires physically disconnecting BMP280 SDA for 5 s; re-init code is in place but the fault injection was not performed |
| T-15 persistence (full) | Gyro bias and `p0` survived reset (`status` bits 4 and 7); six-pose accel + mag calibration and power-cycle check still to do |
| T-05, T-06, T-07, T-08 | Carried over from M4: still/vibration fixture, tilt fixture, reference compass, mag-disabled build |

## 2026-11-13 — M4 fusion

Firmware: `f1b4578-dirty` (Debug), flashed with OpenOCD. Board flat on the
bench, USB cable attached, BMP280 at `0x77`. No `B` reference was captured, so
relative altitude uses the ISA sea-level pressure.

### Build and host tests

| Item | Result |
|---|---|
| `cmake --preset ci && cmake --build --preset ci` | pass, `-Werror` clean |
| `ctest --test-dir build/tests --output-on-failure` | 609 checks, 0 failures, 0.06 s |
| Debug resources | FLASH 41 484 B (63.3 % of budget), RAM 4 204 B (34.2 %) |
| `ci` resources | FLASH 32 556 B (49.7 %), RAM 4 168 B (33.9 %) |

### T-05 static attitude (partial)

Capture: 59.7 s, 2 983 data frames, 0 CRC errors, 0 sequence gaps after
discarding bytes left in the VCP buffer from the previous image. 50.0 Hz data,
1.0 Hz debug; 0 bus errors.

| Segment | Roll σ | Pitch σ | Yaw σ |
|---|---|---|---|
| 25–47 s (quiet) | 0.24° | 0.18° | 0.69° |
| 15–47 s | 0.21° | 0.17° | 0.58° |

Roll and pitch meet NFR-02. Yaw does not (< 0.3° in this run). The gyro and
accelerometer show continuous bench vibration (gyro σ 0.03–0.07 rad/s,
`|a|` σ 0.34 m/s²) and the board was physically disturbed at 48.5 s, so the
run does not satisfy T-05's "flat and still" condition. Repeat on a rigid,
vibration-free surface before closing M4.

### T-09 altitude noise (partial)

`B` reference captured; after the 2 s averaging the status flag `baro reference
set` (bit 7) is on. A 20 s capture (987 frames) gives a filtered relative
altitude mean 0.022 m and σ = 0.069 m (NFR-06 < 0.1 m); raw pressure σ 0.79 Pa.
A formal 60 s run is still needed.

A second capture without `B` (25–47 s quiet window) had σ = 0.051 m, mean
60.8 m, consistent with the ISA fallback pressure.

### CPU load and loop time

Debug frames report CPU load 7.4 % and max loop time 1.16 ms at 100 Hz IMU /
25 Hz baro / 50 Hz telemetry. This is 11.6 % of the 10 ms IMU slot and well
below NFR-11 (30 %).

### β tuning

Replaying the quiet 25–47 s segment at 50 Hz with β = 0.2, 0.1, 0.05, 0.03,
0.02 and 0.01 changed the yaw σ only from 0.65° to 0.58°. The residual noise is
dominated by motion, not magnetometer correction, so β stays at the SPEC
default 0.1. Revisit after a clean T-05 run.

### Pending

| Test | Reason |
|---|---|
| T-05 (formal) | bench vibration and the disturbance at 48.5 s |
| T-06 | needs a ±30°/±60° reference fixture |
| T-07 | needs a reference compass and a non-magnetic location |
| T-08 | needs a mag-disabled build and 5 min still; not run yet |

## 2026-11-13 — M1/M2/M3

See `docs/PLAN.md` for the M1 telemetry run, the T-03 axis measurement and the
M3 calibration/save implementation notes.
