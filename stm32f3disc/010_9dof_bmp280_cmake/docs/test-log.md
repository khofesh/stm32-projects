# Test log

Hardware records for `010_9dof_bmp280_cmake`. Host tests are summarised from
`ctest`; commands and build sizes are in the entries below.

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
