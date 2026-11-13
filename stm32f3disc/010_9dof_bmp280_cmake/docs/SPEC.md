# SPEC — 10-DOF IMU Node on STM32F3DISCOVERY + BMP280
## Build variant: STM32CubeMX + CMake + arm-none-eabi-gcc

| Field | Value |
|---|---|
| Document | `docs/SPEC.md` |
| Revision | 1.1 (draft) |
| Status | Proposed |
| Toolchain | STM32CubeMX → CMake → Ninja → arm-none-eabi-gcc |
| Target | STM32F3DISCOVERY (STM32F303VCT6) + external BMP280 breakout |
| Board | MB1035-F303C-E02 (I3G4250D + LSM303AGR, ST-LINK/V2-1 with VCP) |
| Relationship | Self-contained: build system (Part A) + product (Part B). See §2 |

---

## 1. Purpose

Produce a 10-DOF inertial measurement node (Part B) with a fully IDE-independent, command-line-first build (Part A):

- **STM32CubeMX** generates the project, startup code, HAL and `CMakeLists.txt`.
- **CMake + Ninja** drive the build; no `.cproject`, no Eclipse metadata.
- **arm-none-eabi-gcc** compiles and links.
- **VS Code** (or any LSP-capable editor) is a convenience layer, not a dependency.
- **CTest** runs host-side unit tests for the parts that are portable logic.

The deliverable is a repository that a clean machine can clone, configure, build, test and flash with four commands.

### 1.1 Goals

1. Reproducible builds from a clean clone, byte-identical for a given tag and toolchain.
2. No GUI required for build, test or CI.
3. CubeMX regeneration is scripted and idempotent.
4. Portable logic (sensor math, fusion, calibration) is unit-tested on the host at full speed.
5. Debug and flash work from VS Code and from the shell.

### 1.2 Non-goals

- Product behaviour. Requirements §5 are inherited unchanged.
- Supporting multiple MCU targets in one build tree (single target, `STM32F303VCTx`).
- Bootloader, OTA, RTOS.
- Windows-native tooling as a *requirement* (it must work, but Linux is the reference platform).

---

## 2. Document structure

The CubeIDE baseline `SPEC.md` is not available; this document is self-contained.

| Part | Sections | Scope |
|---|---|---|
| A — Build system | §1–§20 | toolchain, CubeMX, CMake, debug, tests, CI |
| B — Product | §21–§25 | software architecture, calibration, telemetry, test cases T-01…T-15 |

Earlier references to "`SPEC.md` §x" map as: §3/§4 hardware → §4; §5 architecture → §21; §6 calibration → §22; §7 NFRs → §5; §8 telemetry → §23; §11 tests → §24.

---

## 3. System overview

```
                        ┌────────────────────────────────────────┐
                        │        STM32F303VCT6 @ 72 MHz           │
  LSM303AGR  ──I2C1─────►  accel + mag                             │
  I3G4250D   ──SPI1─────►  gyro                                    │
  BMP280     ──I2C2─────►  pressure / temperature                  │
                        │  calibration → fusion → telemetry        │
                        └───────────┬────────────────────────────┘
                                    │ USART1 (PC4/PC5)
                                    ▼
                        ST-LINK VCP → host tools/log.py
```

Repository is built by `cmake --preset Release`; output is `build/Release/010_9dof_bmp280_cmake.elf` plus `.bin`, `.hex`, `.map`.

---

## 4. Hardware (inherited)

| BMP280 pin | F3DISCOVERY | Note |
|---|---|---|
| VCC | **3V** | 3.3 V only (or 5V if using a 5 V clone board) |
| GND | GND | — |
| SCL | PA9 | I2C2_SCL (AF4), 400 kHz, external 4.7 kΩ pull-up on breakout |
| SDA | PA10 | I2C2_SDA (AF4), 400 kHz, external 4.7 kΩ pull-up on breakout |
| SDO | pulled high on breakout | address `0x77` (Adafruit BMP280) |
| CSB | **3V** | must be high |

| Device | Bus | Address |
|---|---|---|
| LSM303AGR accel | I2C1 | `0x19` (WHO_AM_I_A `0x0F` = `0x33`) |
| LSM303AGR mag | I2C1 | `0x1E` (WHO_AM_I_M `0x4F` = `0x40`) |
| BMP280 | I2C2 | `0x77` (chip ID `0xD0` = `0x58`) |
| I3G4250D | SPI1, CS=PE3 | WHO_AM_I `0x0F` = `0xD3` |

Board revision MB1035-F303C-E02, identity confirmed by reading WHO_AM_I on hardware (2026-11-12). Earlier revisions (L3GD20 `0xD4`, LSM303DLHC) are not supported.

Peripheral parameters: §7.3. Telemetry goes over USART1 PC4/PC5 to the ST-LINK/V2-1 virtual COM port.

---

## 5. Product requirements (inherited, summary)

| ID | Requirement | Target |
|---|---|---|
| NFR-01 | Attitude update rate | 100 Hz |
| NFR-02 | Static attitude noise | < 0.3° RMS |
| NFR-03 | Dynamic roll/pitch error | < 2° RMS |
| NFR-04 | Calibrated heading error | < 3° |
| NFR-05 | Gyro-only yaw drift (mag disabled, calibrated) | < 10°/min |
| NFR-06 | Relative altitude noise | < 0.1 m RMS |
| NFR-07 | Relative altitude drift (15 min) | < 0.5 m |
| NFR-08 | Telemetry frame rate | 50 Hz ± 0.1 % |
| NFR-09 | Sensor-to-UART latency | < 20 ms |
| NFR-10 | 30 min run, 100 Hz, dropped frames | 0 |
| NFR-11 | CPU load | < 30 % |
| NFR-12 | Flash footprint | < 64 KB |
| NFR-13 | Static RAM footprint | < 12 KB |
| NFR-14 | Power-on to first valid frame (no calibration run) | < 2 s |
| NFR-15 | Recovery from a sensor bus fault, no MCU reset | < 1 s |

Functional behaviour §21, calibration §22, telemetry frame (84 bytes, 50 Hz, CRC16-CCITT) §23, test cases T-01…T-15 §24.

---

## 6. Toolchain and versions

The build must be reproducible from a pinned set. Two supported acquisition paths:

| Component | Version | Source |
|---|---|---|
| STM32CubeMX | **6.15.x** (min 6.12, see note) | st.com, or CubeCLT-adjacent install |
| STM32CubeCLT | **1.16.0 or newer, pinned** | st.com — bundles GCC, CMake, Ninja, ST-LINK GDB server, CubeProgrammer CLI |
| CMake | **≥ 3.25** (from CubeCLT or system) | — |
| Ninja | ≥ 1.11 | — |
| arm-none-eabi-gcc | the one bundled in the pinned CubeCLT | — |
| Python | ≥ 3.9 (host tools, size check) | — |

> **Version notes.** CubeMX 6.11 was the first release with native CMake generation; 6.12 added multi-context support. Avoid **6.12.1** (linker-script regression). From **6.15** the toolchain file is referenced only from `CMakePresets.json`, not duplicated in `CMakeLists.txt` — this document assumes 6.15+ behaviour. Pin the exact version in `docs/toolchain.md` and in CI.

`docs/toolchain.md` must record, for each release:

```
CubeMX ............ 6.15.0
CubeCLT ........... 1.16.0
arm-none-eabi-gcc .. <output of --version>
CMake ............. <output of --version>
Ninja ............. <output of --version>
```

### 6.1 Toolchain requirements

| ID | Requirement |
|---|---|
| TR-01 | The build **shall** use the GCC bundled with the pinned CubeCLT (or an explicitly documented equivalent). |
| TR-02 | The project **shall not** contain `.cproject`, `.project`, `.mxproject`-dependent build state, or any Eclipse metadata. |
| TR-03 | `CMAKE_TOOLCHAIN_FILE` **shall** be supplied by `CMakePresets.json`, with no absolute paths. |
| TR-04 | C standard **shall** be C11 with GNU extensions **off** (`-std=c11`). |
| TR-05 | Floating point **shall** use `-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard`. |
| TR-06 | The build **shall** succeed from a clean clone with no manual IDE step. |
| TR-07 | CubeMX tool-owned files **shall not** be edited; user changes go in the root `CMakeLists.txt`. |
| TR-08 | Regeneration **shall** be scriptable and idempotent (§7.4). |

---

## 7. Project generation with STM32CubeMX

### 7.1 What CubeMX 6.15+ emits

| Path | Ownership | May edit |
|---|---|---|
| `<name>.ioc` | Project | Yes (via GUI, then commit) |
| `CMakeLists.txt` (root) | **User** — created once | **Yes** |
| `CMakePresets.json` | **User** | **Yes** |
| `cmake/gcc-arm-none-eabi.cmake` | User-modifiable, usually untouched | Yes |
| `cmake/starm-clang.cmake` | Generated, unused here | No |
| `cmake/stm32cubemx/CMakeLists.txt` | **Tool** — interface library with HAL + Core sources | **No** |
| `STM32F303VCTx_FLASH.ld` | Tool | No |
| `Core/`, `Drivers/`, `startup_stm32f303xc.s` | Tool | Only inside `USER CODE` blocks |

The root `CMakeLists.txt` creates the executable target and links the generated `stm32cubemx` interface library. All our code is attached there.

### 7.2 CubeMX project settings

| Setting | Value |
|---|---|
| Board | STM32F3DISCOVERY |
| Project name | `010_9dof_bmp280_cmake` (no spaces, ASCII) |
| Toolchain / IDE | **CMake** |
| Compiler | **GCC** |
| Application structure | Advanced |
| Generate under root | Yes |
| Keep user code | Yes |
| Delete previously generated files | **No** (protects user files) |

### 7.3 Peripheral configuration

| Peripheral | Setting |
|---|---|
| RCC | HSE bypass 8 MHz (ST-LINK MCO on PF0), PREDIV /1, PLL ×9 → 72 MHz; AHB /1; APB1 /2 (36 MHz); APB2 /1; USB /1.5 (48 MHz); I2C1/I2C2 ← HSI; USART1 ← PCLK2 |
| Flash | 2 wait states, prefetch on, I-cache on |
| I2C1 | PB6/PB7, fast mode 400 kHz, analog filter on, digital filter 0 (LSM303AGR, on-board) |
| I2C2 | PA9 SCL / PA10 SDA (AF4, open-drain, no internal pull), fast mode 400 kHz, analog filter on, digital filter 0 (BMP280) |
| SPI1 | PA5/PA6/PA7, master, 8-bit, MSB first, mode 3, prescaler /16 (4.5 MHz), SW NSS |
| GPIO | PE3 output push-pull, initial high (I3G4250D CS) |
| USART1 | PC4 TX, PC5 RX, 460800 8N1; TX on DMA1 Channel 4 (normal mode); RX interrupt (1-byte commands) |
| TIM6 | 1 kHz update interrupt |
| SYS | SWD, no stop debug |
| NVIC | TIM6 prio 3; I2C1/I2C2/SPI1 event, USART1, DMA1 Ch4 prio 2; nothing at prio 0 |

### 7.4 Headless regeneration

CubeMX can regenerate from a script, which makes the `.ioc` reviewable in CI:

```text
# tools/regenerate.txt
config load 010_9dof_bmp280_cmake.ioc
project toolchain CMake
project compiler GCC
project generate
exit
```

```bash
STM32CubeMX -q tools/regenerate.txt
git diff --exit-code -- Core Drivers cmake/stm32cubemx CMakeLists.txt
```

The `git diff --exit-code` step is the guard: if regeneration changes generated files, either the `.ioc` was edited out-of-band or the CubeMX version drifted. Both are failures.

---

## 8. Repository layout

```
/
├── README.md
├── 010_9dof_bmp280_cmake.ioc
├── CMakeLists.txt           # user-owned, generated once
├── CMakePresets.json        # user-owned
├── STM32F303VCTx_FLASH.ld
├── cmake/
│   ├── gcc-arm-none-eabi.cmake
│   └── stm32cubemx/CMakeLists.txt     # tool-owned
├── Core/                    # tool-owned (USER CODE blocks only)
├── Drivers/                 # HAL + CMSIS (tool-owned)
├── App/                     # our code, {Inc,Src}
├── Sensors/                 # our drivers, {Inc,Src} + vendor/ (LibDriver BMP280)
├── Fusion/                  # our code, {Inc,Src}
├── Bsp/                     # our code, {Inc,Src}
├── tests/                   # host-only unit tests (separate CMake tree)
│   ├── CMakeLists.txt
│   ├── test_bmp280.c
│   ├── test_axis_map.c
│   ├── test_madgwick.c
│   └── test_baro_filter.c
├── tools/
│   ├── log.py               # telemetry parser → CSV
│   ├── plot.py
│   ├── size_check.py
│   └── regenerate.txt
└── docs/
    ├── PLAN.md
    ├── toolchain.md
    ├── hardware.md
    ├── calibration.md
    └── test-log.md
```

---

## 9. CMake architecture

### 9.1 Targets

| Target | Type | Contents |
|---|---|---|
| `stm32cubemx` | INTERFACE | generated HAL + Core, include dirs, defines (tool-owned) |
| `bsp` | STATIC | `Bsp/Src/*.c` |
| `sensors` | STATIC | `Sensors/Src/*.c` |
| `fusion` | STATIC | `Fusion/Src/*.c` |
| `app` | STATIC | `App/Src/*.c` |
| `010_9dof_bmp280_cmake` | EXECUTABLE | links `app`, `fusion`, `sensors`, `bsp`, `stm32cubemx` |
| `010_9dof_bmp280_cmake.hex` / `.bin` | custom | post-build `objcopy` |

Rationale: static libraries keep include directories honest, make host-testing trivial (link the same `fusion`/`sensors` sources natively), and give `--gc-sections` something to do.

### 9.2 Root `CMakeLists.txt` (user-owned, illustrative)

```cmake
cmake_minimum_required(VERSION 3.25)
project(010_9dof_bmp280_cmake C ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

# Tool-owned: defines the stm32cubemx interface target, HAL sources, defines
include(cmake/stm32cubemx/CMakeLists.txt)

# --- our code ---------------------------------------------------------
add_library(bsp     STATIC Bsp/Src/bsp_i2c.c Bsp/Src/bsp_spi.c Bsp/Src/bsp_time.c)
add_library(sensors STATIC Sensors/Src/bmp280.c Sensors/Src/lsm303agr.c Sensors/Src/i3g4250d.c)
add_library(fusion  STATIC Fusion/Src/madgwick.c Fusion/Src/baro_filter.c)
add_library(app     STATIC App/Src/app.c App/Src/telemetry.c App/Src/calibration.c)

foreach(t bsp sensors fusion app)
    target_include_directories(${t} PUBLIC ${t}/Inc)
    target_link_libraries(${t} PUBLIC stm32cubemx)
    target_compile_options(${t} PRIVATE
        -Wall -Wextra -Wshadow -Wdouble-promotion -Wundef -Wfloat-equal
        -fno-common -ffunction-sections -fdata-sections)
endforeach()

target_link_libraries(app PUBLIC fusion sensors bsp)

# --- executable -------------------------------------------------------
add_executable(${PROJECT_NAME}.elf Core/Src/main.c)
target_link_libraries(${PROJECT_NAME}.elf PRIVATE app)
target_link_options(${PROJECT_NAME}.elf PRIVATE
    -Wl,--gc-sections -Wl,--print-memory-usage -Wl,-Map=$<TARGET_FILE_DIR:${PROJECT_NAME}.elf>/010_9dof_bmp280_cmake.map)

# --- artifacts + budget ----------------------------------------------
add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex   $<TARGET_FILE:${PROJECT_NAME}.elf> $<TARGET_FILE_DIR:${PROJECT_NAME}.elf>/010_9dof_bmp280_cmake.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${PROJECT_NAME}.elf> $<TARGET_FILE_DIR:${PROJECT_NAME}.elf>/010_9dof_bmp280_cmake.bin
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_SOURCE_DIR}/tools/size_check.py
            --elf $<TARGET_FILE:${PROJECT_NAME}.elf> --flash-max 65536 --ram-max 12288)

# --- flash target -----------------------------------------------------
add_custom_target(flash
    COMMAND ${CMAKE_COMMAND} -E echo "Flashing ${PROJECT_NAME}.elf"
    COMMAND openocd -f interface/stlink.cfg -f target/stm32f3x.cfg
            -c "program $<TARGET_FILE:${PROJECT_NAME}.elf> verify reset exit"
    DEPENDS ${PROJECT_NAME}.elf)
```

> Target name in the generated `cmake/stm32cubemx/CMakeLists.txt` is `stm32cubemx` for single-core projects at the time of writing. If a CubeMX upgrade renames it, adjust **only** the root file and note it in `docs/toolchain.md`.

### 9.3 `CMakePresets.json`

```json
{
  "version": 6,
  "configurePresets": [
    {
      "name": "base", "hidden": true,
      "generator": "Ninja",
      "binaryDir": "${sourceDir}/build/${presetName}",
      "toolchainFile": "${sourceDir}/cmake/gcc-arm-none-eabi.cmake",
      "cacheVariables": { "CMAKE_EXPORT_COMPILE_COMMANDS": "ON" }
    },
    { "name": "Debug",   "inherits": "base",
      "cacheVariables": { "CMAKE_BUILD_TYPE": "Debug" } },
    { "name": "Release", "inherits": "base",
      "cacheVariables": { "CMAKE_BUILD_TYPE": "Release" } },
    { "name": "ci",      "inherits": "Release",
      "cacheVariables": { "CMAKE_BUILD_TYPE": "MinSizeRel",
                          "OUR_WERROR": "ON" } }
  ],
  "buildPresets": [
    { "name": "Debug",   "configurePreset": "Debug" },
    { "name": "Release", "configurePreset": "Release" },
    { "name": "ci",      "configurePreset": "ci" }
  ]
}
```

### 9.4 Flags

| Purpose | Flags |
|---|---|
| CPU | `-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard` (from generated toolchain file) |
| Language | `-std=c11` |
| Warnings (ours) | `-Wall -Wextra -Wshadow -Wundef -Wdouble-promotion -Wfloat-equal` |
| Optimization | Debug `-Og -g3`; Release `-O2`; ci `-Os` |
| Sections | `-ffunction-sections -fdata-sections`; link `-Wl,--gc-sections` |
| Libc | `--specs=nano.specs`; `_write` retargeted to USART1 in `syscalls.c` |
| Diagnostics | `-Wl,--print-memory-usage -Wl,-Map=…` |

`-Werror` is **not** applied to `Drivers/` (HAL) but **is** applied to `App/ Sensors/ Fusion/ Bsp/` in CI.

### 9.5 Size budget enforcement

`tools/size_check.py` parses `arm-none-eabi-size -A`, computes:

- flash = `.isr_vector + .text + .rodata + .init_array + .fini_array +
  .preinit_array + .ARM.extab + .ARM + .data` (`.data` counts its flash load copy)
- ram = `.data + .bss + ._user_heap_stack` (excluding `.ccmram`)

and exits non-zero if either exceeds the budget. This turns NFR-12/NFR-13 from a review note into a build failure.

---

## 10. Build workflow

```bash
# configure once per preset
cmake --preset Debug

# build
cmake --build --preset Debug

# artifacts
ls build/Debug/010_9dof_bmp280_cmake.{elf,hex,bin,map}

# flash (ST-LINK connected)
cmake --build --preset Debug --target flash
```

Release:

```bash
cmake --preset Release && cmake --build --preset Release
```

| ID | Requirement |
|---|---|
| BR-01 | `cmake --preset Release && cmake --build --preset Release` **shall** produce `.elf`, `.hex`, `.bin`, `.map`. |
| BR-02 | A clean build **shall** complete in < 60 s on a 4-core machine. |
| BR-03 | Rebuild after touching one `.c` file **shall** recompile only affected units. |
| BR-04 | `cmake --build --preset Debug --target flash` **shall** program and verify the target. |
| BR-05 | Build **shall** fail if flash or RAM budget is exceeded. |
| BR-06 | The build tree **shall** be relocatable (no absolute paths baked in). |

---

## 11. Debug workflow

### 11.1 VS Code (recommended)

| Component | Choice |
|---|---|
| Extension | `ms-vscode.cpptools`, `marus25.cortex-debug`, `ms-vscode.cmake-tools` |
| GDB server | OpenOCD (`interface/stlink.cfg`, `target/stm32f3x.cfg`) **or** ST-LINK GDB server from CubeCLT |
| SVD | `STM32F303.svd` for peripheral register view |
| SWO | ITM channel 0, 2 MHz, `PB3` (SB10 closed) |

`.vscode/launch.json` and `.vscode/tasks.json` are committed so a fresh clone debugs without setup. They must not contain machine-specific absolute paths; use `${workspaceFolder}` and a `cortex-debug.armToolchainPath` setting exported from `docs/toolchain.md`.

### 11.2 Shell

```bash
openocd -f interface/stlink.cfg -f target/stm32f3x.cfg \
        -c "program build/Debug/010_9dof_bmp280_cmake.elf verify reset exit"
```

```bash
arm-none-eabi-gdb build/Debug/010_9dof_bmp280_cmake.elf \
  -ex "target extended-remote :3333" -ex "monitor reset halt" -ex "load"
```

### 11.3 Constraints

- Debug builds only add `-g3 -Og`; they must not change behaviour.
- `printf` to VCP is permitted in debug builds only; the 50 Hz telemetry path uses raw binary frames.
- `_printf_float` is enabled in Debug only (link-time cost ~10 KB).

---

## 12. Host-side unit testing

This is the main reason to prefer CMake. Pure logic is compiled natively and tested at full speed, no target, no flashing.

### 12.1 What is testable on the host

| Module | Test |
|---|---|
| BMP280 compensation | datasheet worked example / captured sample vectors, tolerance ±1 Pa |
| Axis mapping | all 24 orientations, sign and permutation correctness |
| Altitude formula | known pressure/altitude pairs, sea level and 500 m |
| Madgwick | quaternion norm = 1 after 10⁶ updates; gyro-only integration matches analytic rotation |
| Baro filter | step response and steady-state |
| CRC16 | published test vector |
| Frame pack/unpack | round-trip, endianness, length |

### 12.2 Structure

`tests/` is a **separate CMake tree** configured with the host compiler (no toolchain file):

```bash
cmake -S tests -B build/tests -DCMAKE_BUILD_TYPE=Debug
cmake --build build/tests
ctest --test-dir build/tests --output-on-failure
```

`tests/CMakeLists.txt` compiles the portable sources directly:

```cmake
add_executable(unit_tests
    test_main.c test_bmp280.c test_axis_map.c test_madgwick.c test_baro_filter.c
    ../Sensors/Src/bmp280.c
    ../Fusion/Src/madgwick.c
    ../Fusion/Src/baro_filter.c)
target_include_directories(unit_tests PRIVATE ../Sensors/Inc ../Fusion/Inc ../App/Inc)
target_compile_definitions(unit_tests PRIVATE HOST_TEST=1)
add_test(NAME unit COMMAND unit_tests)
```

`HOST_TEST` guards the small amount of hardware plumbing (`bsp_i2c_*`) behind stubs in `tests/stubs/`.

### 12.3 Rules

| ID | Requirement |
|---|---|
| BR-07 | Portable modules **shall not** include `stm32f3xx_hal.h` outside `#ifndef HOST_TEST`. |
| BR-08 | `ctest` **shall** run in < 5 s. |
| BR-09 | Every numeric algorithm **shall** have at least one test vector from a datasheet or a reference implementation. |
| BR-10 | Tests **shall** pass on the host without hardware attached. |

---

## 13. Static analysis and formatting

| Tool | Invocation | Gate |
|---|---|---|
| `clang-format` | `clang-format --dry-run --Werror App Sensors Fusion Bsp` | CI |
| `cppcheck` | `cppcheck --enable=warning,style,performance --error-exitcode=1 App Sensors Fusion Bsp` | CI |
| `clang-tidy` | via `compile_commands.json`, `--extra-arg=--target=arm-none-eabi` | advisory |
| `-fanalyzer` | GCC static analyzer on our TUs | advisory |
| `arm-none-eabi-size` | post-build | hard budget |

`CMAKE_EXPORT_COMPILE_COMMANDS=ON` is set in the `base` preset so editors and `clang-tidy` work without extra config.

---

## 14. CI

Reference pipeline (GitHub Actions, Linux):

```yaml
jobs:
  build:
    runs-on: ubuntu-24.04
    container: ghcr.io/<org>/stm32-cubeclt:<pinned>   # or install toolchain explicitly
    steps:
      - uses: actions/checkout@v4
      - run: cmake --preset ci
      - run: cmake --build --preset ci
      - run: cmake -S tests -B build/tests && cmake --build build/tests
      - run: ctest --test-dir build/tests --output-on-failure
      - run: python3 tools/size_check.py --elf build/ci/010_9dof_bmp280_cmake.elf --flash-max 65536 --ram-max 12288
      - uses: actions/upload-artifact@v4
        with: { name: firmware, path: build/ci/010_9dof_bmp280_cmake.{elf,hex,bin,map} }
```

### 14.1 CI requirements

| ID | Requirement |
|---|---|
| TR-09 | CI **shall** build with the same pinned CubeCLT image as developers. |
| TR-10 | CI **shall** fail on warnings in our sources and on size-budget overflow. |
| TR-11 | CI **shall** run the host test suite. |
| TR-12 | CI **shall** publish `.elf/.hex/.bin/.map` artifacts per commit. |
| TR-13 | CI **shall** verify that CubeMX regeneration produces no diff (§7.4), when a CubeMX license/CLI is available on the runner; otherwise this is a documented manual step at release. |
| TR-14 | CI **shall** complete in < 5 min. |

---

## 15. Reproducibility

1. **Pin** CubeMX, CubeCLT and CMake versions in `docs/toolchain.md`; CI uses the same image.
2. **Commit** `.ioc`, `CMakeLists.txt`, `CMakePresets.json`, toolchain file, linker script and startup file.
3. **Vendor** nothing that CubeMX can regenerate, but pin the **firmware package** version (`FW_F3 V1.11.x`) in the `.ioc`.
4. **Record** `git describe --always --dirty` into a build define (`-DBUILD_GIT_DESC="..."`) exposed in the 1 Hz debug frame.
5. **Archive** the `.map` file with each release; it is the record of what was linked.
6. **Verify** byte-identical rebuild: build twice from clean trees, compare `.elf` checksums (GCC output is deterministic given identical paths and flags; use `-ffile-prefix-map` if paths leak).

---

## 16. Acceptance criteria for the build system

| ID | Criterion | Pass |
|---|---|---|
| AC-01 | Clean clone → `cmake --preset Release` → build | succeeds, no GUI |
| AC-02 | Artifacts present | `.elf .hex .bin .map` |
| AC-03 | Host tests | all pass, < 5 s |
| AC-04 | Warnings in our sources | zero with `-Werror` |
| AC-05 | Size budget | flash < 64 KB, RAM < 12 KB (build-enforced) |
| AC-06 | Flash via CMake target | verify passes |
| AC-07 | Debug via VS Code | breakpoint, step, SVD peripheral view, SWO `printf` |
| AC-08 | CubeMX regeneration | no diff outside `USER CODE` blocks |
| AC-09 | CI | green in < 5 min, artifacts published |
| AC-10 | Second machine (different path, no IDE) | identical artifacts |

AC-01…AC-05 must hold before M1; AC-06…AC-10 before M5.

---

## 17. Migration from the CubeIDE baseline

| Step | Action |
|---|---|
| 1 | Generate a **new** CubeMX project with Toolchain = CMake, same `.ioc` settings; do not convert the CubeIDE tree. |
| 2 | Copy `App/ Sensors/ Fusion/ Bsp/` and `tools/` unchanged. |
| 3 | Add the library targets from §9.2 to the root `CMakeLists.txt`. |
| 4 | Move `_write`, `_sbrk` and assert hooks into `Core/Src/syscalls.c`; keep `USER CODE` discipline. |
| 5 | Rebuild; compare `.map` sizes against the CubeIDE build (expect ±5 %). |
| 6 | Add `tests/` tree and the first three test files (BMP280 math, axis map, CRC16). |
| 7 | Add `.vscode/` launch/tasks, then delete all Eclipse metadata. |
| 8 | Tag `v1.0-cmake`; keep the CubeIDE branch for one release, then archive. |

Do not attempt to keep both build systems in one tree. Two build descriptions drift, and the `.ioc` becomes ambiguous.

---

## 18. Risks

| ID | Risk | Likelihood | Impact | Mitigation |
|---|---|---|---|---|
| R-11 | CubeMX CMake generator changes layout between minor versions | Medium | High | pin version; `README` states supported range; regeneration diff check |
| R-12 | Generated target name changes, breaking the root `CMakeLists.txt` | Low | Medium | one-line fix, documented in `docs/toolchain.md` |
| R-13 | Tool-owned file edited by mistake, silently reverted on regeneration | Medium | Medium | §7.1 ownership table; code review; regenerated-diff CI |
| R-14 | Host tests drift from target behaviour (different float semantics) | Low | Medium | same `-std=c11`, avoid `long double`, test on target for final acceptance |
| R-15 | Toolchain version skew between developer and CI | Medium | High | container image pinned by digest; `docs/toolchain.md` |
| R-16 | `printf` float pulls in ~10 KB and blows the budget | Medium | Low | Debug-only, guarded; size check catches it |
| R-17 | Debug server (OpenOCD) conflicts with ST-LINK GDB server | Low | Low | document one, keep the other optional |
| R-18 | CubeMX CLI requires a license/activation on CI | Medium | Medium | regeneration check is a release-time manual step if unavailable |
| R-19 | No IDE means slower onboarding | High | Low | committed `.vscode/`, `README` quickstart, 4-command build |

---

## 19. Milestones

| ID | Deliverable | Exit criteria |
|---|---|---|
| M0 | Build skeleton | AC-01…AC-03; LED blink + VCP banner |
| M1 | Raw sensors | accel/gyro/mag/pressure at 25 Hz over UART; AC-04/AC-05 |
| M2 | Axis + identity | T-02, T-03; `docs/hardware.md` records board rev |
| M3 | Calibration | T-04; coefficients stored and applied |
| M4 | Fusion | T-05…T-07 pass at 100 Hz |
| M5 | Hardening | T-08…T-15; AC-06…AC-10; CI green |

---

## 20. Open questions

| # | Item | Default if unresolved |
|---|---|---|
| Q-1 | Host OS target: Linux only, or Linux + Windows + macOS? | Linux reference, others best-effort |
| Q-2 | OpenOCD or ST-LINK GDB server as the documented default? | OpenOCD (no license, scriptable) |
| Q-3 | Is a CubeMX CLI on CI worth the licensing friction? | Manual regeneration check at release |
| Q-4 | Ship a Docker/Dev Container for the toolchain? | Yes, in M5 |
| Q-5 | `-Os` or `-O2` for the release artifact? | `-O2`; revisit if the budget is tight |
| Q-6 | Body-frame axis orientation relative to the PCB | Determined empirically in T-03, recorded in `docs/hardware.md` |

---

# Part B — Product specification

## 21. Software architecture

### 21.1 Layers

```
App      scheduler · calibration · telemetry · command parser
Fusion   axis_map · madgwick · baro_filter · altitude
Sensors  i3g4250d · lsm303agr · bmp280          (portable, no HAL)
Bsp      bsp_i2c · bsp_spi · bsp_time · bsp_uart · bsp_flash   (HAL, CubeMX handles)
Core     CubeMX-generated init, ISRs → USER CODE hooks only
```

Dependencies point downward only. `Sensors/` and `Fusion/` compile on the host with `HOST_TEST` (BR-07).

### 21.2 Sensor configuration

| Sensor | Mode | ODR | Range / resolution | Read at |
|---|---|---|---|---|
| I3G4250D | normal, BDU on, LSB at lower address | 100 Hz, BW 25 Hz | ±500 dps, 17.5 mdps/LSB | 100 Hz |
| LSM303AGR accel | high-resolution 12-bit, BDU on | 100 Hz | ±4 g, 2 mg/LSB | 100 Hz |
| LSM303AGR mag | continuous, offset cancellation on, BDU on | 100 Hz | 1.5 mG/LSB | 100 Hz |
| BMP280 | normal, osrs_p ×16, osrs_t ×2, IIR 16, t_sb 0.5 ms | ≈ 26 Hz | 0.16 Pa resolution | 25 Hz |

Note: the LSM303AGR accelerometer output registers are 16-bit left-justified, with the 12-bit high-resolution value in bits 15:4 (verified on MB1035-F303C-E02 during T-02); the driver shifts right by 4.

### 21.3 Scheduling

TIM6 interrupts at 1 kHz; the ISR only increments a `volatile uint32_t` tick and sets due flags. The main loop is a run-to-completion cooperative scheduler:

| Task | Period | Work |
|---|---|---|
| `imu_task` | 10 ms | read gyro + accel + mag, apply calibration and axis map, Madgwick update |
| `baro_task` | 40 ms | read BMP280, compensate, altitude, baro filter |
| `telemetry_task` | 20 ms | pack frame (§23), start USART1 TX DMA; skip and count if previous DMA still busy |
| `debug_task` | 1000 ms | pack debug frame (§23.4) |
| `cmd_task` | idle | consume bytes from the RX ring buffer (§22.4) |

Rules: no `HAL_Delay()` after init; a task overrunning its slot sets `STATUS_OVERRUN` and increments `overruns`; CPU load is measured with DWT `CYCCNT` over busy time per 1 s window.

### 21.4 Module APIs

All functions return `int32_t` status (`0` = OK, negative = error) unless stated. Units in SI unless stated.

```c
/* Bsp */
int32_t  bsp_i2c_read(uint8_t bus, uint8_t addr7, uint8_t reg, uint8_t *buf, uint16_t len);
int32_t  bsp_i2c_write(uint8_t bus, uint8_t addr7, uint8_t reg, const uint8_t *buf, uint16_t len);
int32_t  bsp_i2c_recover(uint8_t bus);                                /* DeInit/Init the handle */
uint32_t bsp_i2c_errors(uint8_t bus);                                 /* failed transfers */
int32_t  bsp_spi_read(uint8_t reg, uint8_t *buf, uint16_t len);      /* I3G4250D, CS PE3 */
int32_t  bsp_spi_write(uint8_t reg, const uint8_t *buf, uint16_t len);
uint32_t bsp_spi_errors(void);
uint32_t bsp_time_ms(void);
uint32_t bsp_time_us(void);                                          /* SysTick based */
void     bsp_delay_ms(uint32_t ms);                                   /* init only, never in a task */
int32_t  bsp_uart_send_dma(const uint8_t *buf, uint16_t len);        /* -EBUSY if in flight */
uint8_t  bsp_uart_tx_ready(void);                                    /* DMA idle */
int32_t  bsp_flash_write_page(uint32_t addr, const void *data, uint16_t len);

/* Sensors */
typedef struct { float x, y, z; } vec3_t;
int32_t i3g4250d_init(void);          int32_t i3g4250d_read(vec3_t *rad_s);
                                      uint8_t i3g4250d_id(void);          /* WHO_AM_I, T-02 */
int32_t lsm303agr_init(void);         int32_t lsm303agr_read_accel(vec3_t *m_s2);
                                      int32_t lsm303agr_read_mag(vec3_t *uT);
                                      uint8_t lsm303agr_accel_id(void);
                                      uint8_t lsm303agr_mag_id(void);
int32_t bmp280_init(void);            int32_t bmp280_read(float *pa, float *deg_c);
                                      uint8_t bmp280_chip_id(void);

/* Fusion */
void  axis_map_apply(const int8_t m[3][3], const vec3_t *in, vec3_t *out);
void  madgwick_init(float beta, float sample_hz);
void  madgwick_update(const vec3_t *g, const vec3_t *a, const vec3_t *m);  /* m NULL → IMU-only */
void  madgwick_get_quat(float q[4]);                                       /* w, x, y, z */
float altitude_from_pressure(float pa, float p0_pa);                       /* 44330·(1−(p/p0)^(1/5.255)) */
void  baro_filter_init(float alpha, float beta, float dt);
float baro_filter_update(float alt_m);

/* App */
void app_init(void);
void app_run(void);                   /* never returns */
```

Frames: body FRD (X forward, Y right, Z down), earth NED. Quaternion rotates body → earth. Sensor → body mapping is a signed permutation per sensor (`axis_map`), filled in from T-03.

### 21.5 Fault handling

A failed bus transaction marks that sensor not-OK in `status`, increments its error counter, and the task keeps running with the remaining sensors (Madgwick falls back to IMU-only without mag; altitude holds the last value). A not-OK sensor is re-initialised once per second; on success its flag is restored (NFR-15). `HAL_I2C_DeInit`/`Init` on the CubeMX handle is used to clear a stuck bus.

---

## 22. Calibration

### 22.1 Parameters

| Sensor | Model | Parameters |
|---|---|---|
| Gyro | `w = raw − b_g` | bias `b_g` (3) |
| Accel | `a = S_a·(raw − b_a)` | offset `b_a` (3), diagonal scale `S_a` (3) |
| Mag | `m = S_m·(raw − b_m)` | hard-iron `b_m` (3), diagonal soft-iron `S_m` (3) |
| Baro | `h = altitude(p, p0)` | reference pressure `p0` |

### 22.2 Procedures

| Step | Command | Procedure | Acceptance |
|---|---|---|---|
| Gyro bias | `G` | board still, average 200 samples (2 s) | per-axis std < 0.05 °/s, else rejected |
| Gyro bias (auto) | — | at boot, same as above if still; otherwise keep stored bias | — |
| Accel 6-position | `A` ×6 | each face up/down, 100 samples each, any order; firmware detects the dominant axis | each orientation within 10° of an axis, all six seen |
| Mag | `M` start / `M` stop | rotate through all orientations ≥ 20 s; min/max per axis | span per axis > 30 µT; span ratio max/min < 1.5 |
| Baro zero | `B` | average 50 samples (2 s) → `p0` | std < 3 Pa |

Results apply immediately; `S` persists them.

### 22.3 Storage

Last flash page (`0x0803F800`, 2 KB). Layout: `magic u32 = 0x31434C41`, `version u16`, `length u16`, parameters (`float`, little-endian), `crc16` over everything before it. Invalid magic, version or CRC → defaults (zero bias, unit scale) and `STATUS_CAL_*` bits cleared. The linker script is CubeMX-owned and not modified; NFR-12 (< 64 KB) guarantees the image never reaches this page.

### 22.4 Commands (USART1 RX, single ASCII byte)

| Byte | Action |
|---|---|
| `G` | gyro bias calibration |
| `A` | capture one accel orientation (six needed) |
| `M` | toggle mag calibration capture |
| `B` | set baro reference `p0` |
| `S` | save calibration to flash |
| `D` | restore defaults (RAM only; `S` to persist) |
| `R` | software reset |

Unknown bytes are ignored. Calibration progress is reported in `calib_state` (§23).

---

## 23. Telemetry

### 23.1 Link

USART1 460800 8N1 → ST-LINK/V2-1 VCP. Binary frames only; `printf` text is allowed in Debug builds before the first frame and never interleaved with frames.

### 23.2 Framing

| Field | Value |
|---|---|
| Length | 84 bytes, fixed, all frame types |
| Byte order | little-endian |
| Sync | `0xAA 0x55` |
| CRC | CRC16-CCITT-FALSE (poly `0x1021`, init `0xFFFF`, no reflection, no xorout) over bytes 2…81; test vector `"123456789"` → `0x29B1` |

### 23.3 Data frame (type `0x01`, 50 Hz)

| Offset | Size | Type | Field | Unit |
|---|---|---|---|---|
| 0 | 2 | u8[2] | sync `AA 55` | — |
| 2 | 1 | u8 | type = `0x01` | — |
| 3 | 1 | u8 | payload length = 78 | bytes |
| 4 | 2 | u16 | sequence (wraps) | — |
| 6 | 4 | u32 | timestamp of IMU sample | ms |
| 10 | 16 | f32[4] | quaternion w, x, y, z | — |
| 26 | 12 | f32[3] | accel (calibrated, body) | m/s² |
| 38 | 12 | f32[3] | gyro (calibrated, body) | rad/s |
| 50 | 12 | f32[3] | mag (calibrated, body) | µT |
| 62 | 4 | f32 | pressure | Pa |
| 66 | 4 | f32 | relative altitude (filtered) | m |
| 70 | 2 | i16 | temperature | 0.01 °C |
| 72 | 2 | u16 | status flags (§23.5) | — |
| 74 | 1 | u8 | CPU load | % |
| 75 | 1 | u8 | calib_state | — |
| 76 | 2 | u16 | task overruns (cumulative) | — |
| 78 | 2 | u16 | telemetry frames skipped (cumulative) | — |
| 80 | 2 | u16 | reserved = 0 | — |
| 82 | 2 | u16 | CRC16 | — |

### 23.4 Debug frame (type `0x02`, 1 Hz)

| Offset | Size | Type | Field |
|---|---|---|---|
| 0–9 | 10 | — | header as §23.3, type `0x02`, timestamp = uptime ms |
| 10 | 32 | char[32] | `BUILD_GIT_DESC`, NUL-padded |
| 42 | 2 | u16 | I2C1 error count |
| 44 | 2 | u16 | I2C2 error count |
| 46 | 2 | u16 | SPI1 error count |
| 48 | 2 | u16 | max loop time (µs, reset each frame) |
| 50 | 2 | u16 | CPU load ‰ |
| 52 | 2 | u16 | free stack (high-water, bytes) |
| 54 | 28 | — | reserved = 0 |
| 82 | 2 | u16 | CRC16 |


### 23.5 Status flags

| Bit | Meaning |
|---|---|
| 0 | gyro OK |
| 1 | accel OK |
| 2 | mag OK |
| 3 | baro OK |
| 4 | gyro calibrated |
| 5 | accel calibrated |
| 6 | mag calibrated |
| 7 | baro reference set |
| 8 | overrun in the last 1 s |
| 9 | calibration in progress |
| 10 | fusion IMU-only (mag excluded) |
| 15 | Debug build |

`tools/log.py` validates sync, length, CRC and sequence continuity and writes CSV; `tools/plot.py` plots it.

---

## 24. Test cases

| ID | Title | Procedure | Pass |
|---|---|---|---|
| T-01 | Boot + link | power on, run `log.py` | first valid frame < 2 s (NFR-14); debug frame shows git describe |
| T-02 | Sensor identity | read WHO_AM_I / chip ID at init | `0xD3`, `0x33`, `0x40`, `0x58`; result in debug log |
| T-03 | Axis mapping | board flat, then each face down; rotate +90° about each body axis | gravity on correct axis and sign; gyro sign matches right-hand rule; mag follows; mapping recorded in `docs/hardware.md` |
| T-04 | Calibration | run §22.2; repeat static checks | gyro residual < 0.1 °/s; \|a\| = 9.81 ± 0.05 m/s² in 6 orientations; \|m\| std < 5 % over full rotation |
| T-05 | Static attitude noise | 60 s flat and still | roll/pitch/yaw std < 0.3° (NFR-02) |
| T-06 | Dynamic roll/pitch | tilt to ±30°, ±60° against a reference fixture | RMS error < 2° (NFR-03) |
| T-07 | Heading | 8 compass points vs. reference compass, away from ferrous objects | max error < 3° (NFR-04) |
| T-08 | Gyro-only yaw drift | mag disabled, 5 min still | < 10°/min (NFR-05) |
| T-09 | Altitude noise | 60 s still, after `B` | std < 0.1 m (NFR-06) |
| T-10 | Altitude drift | 15 min still, indoor, doors closed | \|drift\| < 0.5 m (NFR-07) |
| T-11 | Latency | toggle a spare GPIO at IMU read; logic analyser on GPIO + USART1 TX | read → last frame byte < 20 ms (NFR-09) |
| T-12 | Endurance | 30 min at 100 Hz fusion / 50 Hz telemetry | 0 CRC errors, 0 sequence gaps, frame rate 50 Hz ± 0.1 % (NFR-08, NFR-10) |
| T-13 | Resources | 60 s run; `size_check.py` | CPU load < 30 % (NFR-11); flash < 64 KB, RAM < 12 KB (NFR-12/13) |
| T-14 | Fault recovery | disconnect BMP280 SDA for 5 s, reconnect | baro-OK cleared, other data uninterrupted, flag restored < 1 s after reconnect (NFR-15) |
| T-15 | Calibration persistence | calibrate, `S`, power-cycle | parameters identical after reboot; `D` + power-cycle without `S` keeps stored values |

Host tests (§12) cover the numeric parts of T-04, T-09 and the CRC/frame layout of T-12 before hardware runs.

---

## 25. Change log

| Rev | Date | Change |
|---|---|---|
| 1.1 | 2026-11-12 | Board MB1035-F303C-E02 (I3G4250D, LSM303AGR) confirmed on hardware; Part B (§21–§25) drafted to replace the unavailable baseline `SPEC.md`; NFR-05/08/14/15 defined; USART1 TX DMA added |
