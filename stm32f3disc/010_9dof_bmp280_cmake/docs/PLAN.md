# PLAN — implementing `docs/SPEC.md`

| Field | Value |
|---|---|
| Spec | `docs/SPEC.md` rev 1.1 |
| Board | MB1035-F303C-E02 (I3G4250D, LSM303AGR) + Adafruit BMP280 @ `0x77` on I2C2 |
| Baseline | `d94912b` — CubeMX config matches §7.3; builds clean (FLASH 18.9 KB, RAM 2.9 KB) |
| Created | 2026-11-12 |

---

## 1. Current state

| Area | State |
|---|---|
| CubeMX config (§7.3) | Done: clock, I2C1/I2C2 400 kHz, SPI1 mode 3, USART1 460800 + TX DMA, TIM6 1 kHz, PE3 init high, NVIC group 4 |
| CMake | `stm32cubemx` + `bsp` + `sensors` + `fusion` + `app` static libs; `.hex/.bin`, size check, flash target, `ci` preset |
| Bsp | `bsp_time`, `bsp_uart`, `bsp_i2c` (bus 1/2, per-bus error counters, DeInit/Init recovery), `bsp_spi` (CS PE3) |
| Sensors | Own `i3g4250d`, `lsm303agr` (accel + mag) + LibDriver `bmp280` wrapper in `Sensors/vendor/bmp280/` |
| App | Cooperative scheduler, raw telemetry at 25 Hz, debug frame at 1 Hz, CRC16, frame packer |
| Tests, tools | Host tree with CRC16/axis map/BMP280 compensation/frame tests; `tools/log.py`, `tools/size_check.py` |
| Docs | `SPEC.md`, `PLAN.md` |

M1 verified on hardware (2026-11-13): IDs `0xD3`/`0x33`/`0x40`/`0x58`, 25.03 Hz over 31.8 s with 0 CRC errors, 0 sequence gaps, 0 skipped frames, 0 task overruns, 0 bus errors; FLASH 21.0 KB, RAM 3.3 KB (ci).

M1 review fixes re-verified (2026-11-13, Debug build, board flat on the bench, 60 s): 25.000 Hz, 1500 data + 60 debug frames, 0 CRC errors, 0 sequence gaps, 0 skipped, 0 overruns, 0 bus errors; FLASH 21.4 KB, RAM 3.3 KB (ci).

M2 code implemented (2026-11-13): `axis_map_is_signed_permutation`, 48-permutation host test, `board_axes` applied in `imu_task`, `docs/hardware.md`. T-03 measured the accel matrix `{{0,1,0},{-1,0,0},{0,0,-1}}` (left-handed triad, det -1); gyro/mag reuse it, gyro signs to be confirmed by the M4 yaw check. FLASH 21.7 KB, RAM 3.4 KB (ci).

M2 closed (2026-11-13): T-03 steps 4–6 on hardware. `M_gyro = {{0,1,0},{1,0,0},{0,0,-1}}` (det +1), `M_mag = M_accel` (det -1), both measured; results in `docs/hardware.md`. Host tests pass.

| Check | Result | Note |
|---|---|---|
| Max loop time (IMU + baro + telemetry in one pass) | 1.12 ms | 11 % of the 10 ms slot; blocking I/O is fine for M4 |
| \|a\| raw | 9.46 m/s² (σ 0.06) | 1.95 mg/LSB; offset/scale left for M3 |
| \|B\| raw | 100 µT (σ 0.3) | 96 µT before temperature compensation; hard iron, M3 |
| Pressure | 100 587 Pa (σ 1.2 Pa), 34.5 °C | first frame flagged baro-not-OK (ADC reset value) |
| Gyro at rest | 0.45 °/s | bias, M3 |


---

## 2. Decisions

| ID | Decision | Rationale |
|---|---|---|
| D-1 | **Write our own I3G4250D and LSM303AGR drivers** to the §21.4 API on top of `bsp_spi`/`bsp_i2c`. Keep **LibDriver BMP280**, wrapped by `Sensors/Src/bmp280.c`. Remove the ST BSP and ST component drivers at the end of M1. | ST drivers return `void` and use global `*_IO_*` callbacks → cannot report bus faults (§21.5, NFR-15) or be host-tested cleanly; LSM303AGR mag read is missing anyway. LibDriver BMP280 is handle-based with status returns and already compiles warning-free. |
| D-2 | Layout `Bsp/{Inc,Src}`, `Sensors/{Inc,Src}`, `Fusion/{Inc,Src}`, `App/{Inc,Src}`; vendored LibDriver moves to `Sensors/vendor/bmp280/`. | AGENTS.md / spec §8. |
| D-3 | Keep executable name `010_9dof_bmp280_cmake`; update spec references to `10dof`. | Name is CubeMX-generated; renaming fights regeneration. |
| D-4 | Flash/debug with **OpenOCD** (installed); no STM32_Programmer_CLI. | Q-2 default; matches bench. |
| D-5 | Debug `-Og -g3`, Release `-O2`, ci `-Os` (edit `cmake/gcc-arm-none-eabi.cmake`). | §9.4, Q-5. Currently `-O0` / `-Os`. |
| D-6 | Host tests use a tiny in-repo assert harness (`tests/test_main.c`), no external framework. | No dependency to pin; ctest < 5 s. |
| D-7 | Board-specific sensor→body matrices live in `App/Src/board_axes.c`; `Fusion/axis_map.c` keeps only the generic apply/validate code. | Fusion stays portable; T-03 values are board data, not algorithm. |

---

## 3. Milestones

Each task lists files and how it is verified. Hardware checks run on the attached board via OpenOCD + VCP.

### M0 — Build skeleton (AC-01…AC-03; LED blink + VCP banner)

| # | Task | Files | Verify |
|---|---|---|---|
| 0.1 | `-Og`/`-O2`/`-Os` per D-5; `CMAKE_C_STANDARD 11`; `ci` preset with `-Werror` on our targets | `cmake/gcc-arm-none-eabi.cmake`, `CMakePresets.json`, `CMakeLists.txt` | all presets build |
| 0.2 | Post-build `.hex`/`.bin`, map in build dir, `BUILD_GIT_DESC` define from `git describe --always --dirty` | `CMakeLists.txt` | BR-01 |
| 0.3 | `tools/size_check.py` (flash/RAM per §9.5) as post-build step, fails build on overflow | `tools/size_check.py`, `CMakeLists.txt` | BR-05: temporarily set budget 1 KB → build fails |
| 0.4 | `flash` custom target via OpenOCD `program … verify reset exit` | `CMakeLists.txt` | BR-04 / AC-06 |
| 0.5 | Restructure to D-2 layout; register `fusion`, `app` libs | `Bsp/`, `Sensors/`, `Fusion/`, `App/`, `CMakeLists.txt` | build clean |
| 0.6 | `bsp_time` (ms from HAL tick, µs from HAL tick + SysTick->VAL) | `Bsp/Src/bsp_time.c` | — |
| 0.7 | `bsp_uart`: DMA TX (`-EBUSY` if in flight), RX interrupt into 64-byte ring; `__io_putchar` for Debug-only printf | `Bsp/Src/bsp_uart.c` | banner on VCP |
| 0.8 | `app_init()`/`app_run()`; TIM6 start + `HAL_TIM_PeriodElapsedCallback` → tick; LD3 heartbeat 1 Hz; banner with git desc | `App/Src/app.c`, `App/Src/sched.c`, `Core/Src/main.c` (USER CODE only) | LED blinks, banner seen |
| 0.9 | `tests/` tree, harness, first test: CRC16-CCITT-FALSE `"123456789"` → `0x29B1` | `tests/CMakeLists.txt`, `tests/test_main.c`, `tests/test_crc16.c`, `App/Src/crc16.c` | AC-03 |
| 0.10 | `tools/regenerate.txt`, `docs/toolchain.md` (gcc 15.2.0, CMake, Ninja, OpenOCD, CubeMX versions) | — | — |

### M1 — Raw sensors (25 Hz over UART; AC-04/AC-05)

| # | Task | Files | Verify |
|---|---|---|---|
| 1.1 | `bsp_i2c` (bus 1 → `hi2c1`, bus 2 → `hi2c2`), timeouts, per-bus error counters, DeInit/Init recovery | `Bsp/Src/bsp_i2c.c` | — |
| 1.2 | `bsp_spi` (CS PE3, read bit `0x80`, auto-increment `0x40`) | `Bsp/Src/bsp_spi.c` | — |
| 1.3 | `i3g4250d.c`: ID check `0xD3`, config §21.2, read → rad/s | `Sensors/Src/i3g4250d.c` | T-02 |
| 1.4 | `lsm303agr.c`: accel ID `0x33` + config + read → m/s²; mag ID `0x40` (`0x4F`), `CFG_REG_A_M` continuous 100 Hz, offset cancel, read `OUTX_L_REG_M` (0x68…0x6D) → µT | `Sensors/Src/lsm303agr.c` | T-02 |
| 1.5 | `bmp280.c` wrapper over LibDriver: link `iic_*` to `bsp_i2c` bus 2, addr `0x77`, config §21.2, chip ID `0x58` | `Sensors/Src/bmp280.c`, `Sensors/vendor/bmp280/` | T-02 |
| 1.6 | Frame packer §23 (data + debug frames), `telemetry_task` at 25 Hz with raw values | `App/Src/telemetry.c` | — |
| 1.7 | `tools/log.py`: sync, length, CRC, sequence check → CSV | `tools/log.py` | live capture |
| 1.8 | Host tests: BMP280 compensation (datasheet worked example, ±1 Pa), frame pack/unpack round-trip + offsets | `tests/test_bmp280.c`, `tests/test_frame.c`, `tests/stubs/` | ctest |
| 1.9 | Remove ST BSP (`stm32f3_discovery*`), ST component drivers, `Sensors/Common` | — | build clean, size drops |
| 1.10 | `-Werror` clean in `ci` preset; size budget enforced | — | AC-04, AC-05 |

### M2 — Axis + identity (T-02, T-03)

| # | Task | Files | Verify |
|---|---|---|---|
| 2.1 | `axis_map_apply` + per-sensor signed permutation constants | `Fusion/Src/axis_map.c` | — |
| 2.2 | Host test: all 48 signed permutations (24 proper rotations + 24 reflections), sign/permutation correctness | `tests/test_axis_map.c` | ctest |
| 2.3 | Run T-03 on hardware; fill matrices for FRD body frame | `App/Src/board_axes.c` | T-03 |
| 2.4 | `docs/hardware.md`: board rev, IDs, BMP280 wiring (Adafruit, `0x77`), axis mapping | `docs/hardware.md` | — |

### M3 — Calibration (T-04, T-15)

| # | Task | Files | Verify |
|---|---|---|---|
| 3.1 | Command parser (§22.4) on the RX ring | `App/Src/cmd.c` | send `P`… bytes from host |
| 3.2 | Calibration state machine: gyro bias (manual + boot auto), accel 6-position, mag min/max, baro `p0` (§22.2 acceptance limits) | `App/Src/calibration.c`, `Fusion/Src/calib_math.c` | T-04 |
| 3.3 | `bsp_flash`: erase/program page `0x0803F800`; storage format §22.3 with CRC | `Bsp/Src/bsp_flash.c`, `App/Src/cal_store.c` | T-15 |
| 3.4 | Host tests: still detection, 6-position offset/scale, mag hard/soft-iron, storage serialize + CRC reject | `tests/test_calibration.c` | ctest |
| 3.5 | `docs/calibration.md`: operator procedure | `docs/calibration.md` | — |

### M4 — Fusion (T-05…T-08 at 100 Hz)

| # | Task | Files | Verify |
|---|---|---|---|
| 4.1 | Madgwick (MARG + IMU-only), β default 0.1 | `Fusion/Src/madgwick.c` | — |
| 4.2 | `altitude_from_pressure`, α-β `baro_filter` | `Fusion/Src/altitude.c`, `Fusion/Src/baro_filter.c` | — |
| 4.3 | Host tests: ‖q‖ = 1 after 10⁶ updates; gyro-only vs analytic rotation; altitude pairs (sea level, 500 m); filter step + steady state | `tests/test_madgwick.c`, `tests/test_baro_filter.c`, `tests/test_altitude.c` | ctest, BR-08 |
| 4.4 | Scheduler at full rates (imu 100 Hz, baro 25 Hz, telemetry 50 Hz, debug 1 Hz); overrun + skipped-frame counters; CPU load via DWT | `App/Src/sched.c`, `App/Src/app.c` | debug frame |
| 4.5 | `tools/plot.py` (attitude, altitude, rates) | `tools/plot.py` | — |
| 4.6 | Tune β; run T-05, T-06, T-07, T-08 | `docs/test-log.md` | NFR-02…05 |

### M5 — Hardening (T-09…T-15; AC-06…AC-10; CI green)

| # | Task | Files | Verify |
|---|---|---|---|
| 5.1 | Fault handling §21.5: sensor OK flags, 1 s re-init, IMU-only fallback, bus recovery | `App/Src/app.c`, `Bsp/Src/bsp_i2c.c` | T-14 |
| 5.2 | Latency probe GPIO (CubeMX step: one free header pin as output) toggled at IMU read | CubeMX guide, `App/Src/app.c` | T-11 |
| 5.3 | Endurance + resources runs: T-09, T-10, T-12, T-13 | `docs/test-log.md` | NFR-06…13 |
| 5.4 | `.clang-format`, cppcheck config, `-fanalyzer` advisory target | repo files | §13 |
| 5.5 | `.vscode/launch.json` + `tasks.json` (cortex-debug, OpenOCD, SVD, SWO) | `.vscode/` | AC-07 |
| 5.6 | GitHub Actions workflow (monorepo root, path filter on this project): ci build, ctest, size check, artifacts | `.github/workflows/` | AC-09 |
| 5.7 | Reproducibility: `-ffile-prefix-map`, double clean build checksum compare; Dev Container (Q-4) | `CMakeLists.txt`, `.devcontainer/` | AC-10 |
| 5.8 | CubeMX regeneration check documented as a release step (Q-3) | `docs/toolchain.md` | AC-08 |

---

## 4. Spec updates to make along the way

| Section | Change | When |
|---|---|---|
| §3, §9.1, §10, §14 | `10dof` → `010_9dof_bmp280_cmake` (D-3) | M0 |
| §9.2, §10, §11.2 | Flash target uses OpenOCD (D-4) | M0 |
| §8 | Add `docs/PLAN.md`; vendored path `Sensors/vendor/` | M0 |
| §4 | Adafruit BMP280 wiring (VIN, CSB/SDO pulled up on breakout) | M2 |
| §21.4 | Any API change found during implementation | as found |

---

## 5. Risks specific to this plan

| Risk | Mitigation |
|---|---|
| Own drivers miss a register quirk the ST drivers handled | Port register defines and init sequences from the ST headers; verify IDs and plausibility in T-02/T-03 |
| LSM303AGR mag at 100 Hz + accel + gyro reads don't fit the 10 ms slot | Measured in M1: 1.12 ms worst loop pass (all tasks); stay blocking, revisit only if M4 fusion pushes it past ~5 ms |
| Flash page `0x0803F800` writes stall the scheduler (~40 ms erase) | Save only on `S` command; mark overrun expected, exclude from NFR-10 run |
| Debug `printf` float pushes flash over budget | Debug-only, size check catches it |
| NFR-02/03/04 not met with Madgwick defaults | β tuning in M4; fall back to gyro-bias online estimation |
