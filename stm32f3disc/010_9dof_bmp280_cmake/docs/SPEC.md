# SPEC — 10-DOF IMU Node on STM32F3DISCOVERY + BMP280
## Build variant: STM32CubeMX + CMake + arm-none-eabi-gcc

| Field | Value |
|---|---|
| Document | `SPEC-CMAKE.md` |
| Revision | 1.0 (draft) |
| Status | Proposed |
| Toolchain | STM32CubeMX → CMake → Ninja → arm-none-eabi-gcc |
| Target | STM32F3DISCOVERY (STM32F303VCT6) + external BMP280 breakout |
| Relationship | Build-system variant of `SPEC.md` (CubeIDE). See §2 |

---

## 1. Purpose

Produce the same 10-DOF inertial measurement node as `SPEC.md`, but with a fully IDE-independent, command-line-first build:

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

## 2. Relationship to `SPEC.md`

| Area | Status |
|---|---|
| §3 Hardware, wiring, pinout | **Inherited unchanged** (restated in §4 for convenience) |
| §5 Software architecture, APIs | **Inherited unchanged** |
| §6 Calibration | **Inherited unchanged** |
| §7 Performance requirements (NFR-01…15) | **Inherited unchanged** (summarised in §5) |
| §8 Telemetry frame format | **Inherited unchanged** |
| §11 Test cases T-01…T-15 | **Inherited unchanged**, plus build tests in §16 |
| §4 CubeMX/CubeIDE configuration | **Superseded** by §6–§9 of this document |
| §10 Build and configuration | **Superseded** by §6–§15 |
| §12 Risks | **Extended** in §18 |
| §14 Future revisions | Unchanged |

If this variant is adopted, `SPEC.md` remains authoritative for everything except build, debug and CI. Do not maintain two divergent copies of the requirement tables.

---

## 3. System overview

```
                        ┌────────────────────────────────────────┐
                        │        STM32F303VCT6 @ 72 MHz           │
  LSM303DLHC ──I2C1─────►  accel + mag                             │
  L3GD20     ──SPI1─────►  gyro                                    │
  BMP280     ──I2C2─────►  pressure / temperature                  │
                        │  calibration → fusion → telemetry        │
                        └───────────┬────────────────────────────┘
                                    │ USART1 (PC4/PC5)
                                    ▼
                        ST-LINK VCP → host tools/log.py
```

Repository is built by `cmake --preset Release`; output is `build/Release/10dof.elf` plus `.bin`, `.hex`, `.map`.

---

## 4. Hardware (inherited)

| BMP280 pin | F3DISCOVERY | Note |
|---|---|---|
| VCC | **3V** | 3.3 V only (or 5V if using a 5 V clone board) |
| GND | GND | — |
| SCL | PA9 | I2C2_SCL (AF4), 400 kHz, external 4.7 kΩ pull-up on breakout |
| SDA | PA10 | I2C2_SDA (AF4), 400 kHz, external 4.7 kΩ pull-up on breakout |
| SDO | GND | address `0x76` |
| CSB | **3V** | must be high |

| Device | Bus | Address |
|---|---|---|
| LSM303DLHC accel | I2C1 | `0x18` / `0x19` |
| LSM303DLHC mag | I2C1 | `0x1E` |
| BMP280 | I2C2 | `0x76` / `0x77` |
| L3GD20 / I3G4250D | SPI1, CS=PE3 | — |

Pinout, clock tree and peripheral parameters: identical to `SPEC.md` §3–§4, with the CubeMX settings expressed as code in §7.3 below.

---

## 5. Product requirements (inherited, summary)

| ID | Requirement | Target |
|---|---|---|
| NFR-01 | Attitude update rate | 100 Hz |
| NFR-02 | Static attitude noise | < 0.3° RMS |
| NFR-03 | Dynamic roll/pitch error | < 2° RMS |
| NFR-04 | Calibrated heading error | < 3° |
| NFR-06 | Relative altitude noise | < 0.1 m RMS |
| NFR-07 | Relative altitude drift (15 min) | < 0.5 m |
| NFR-09 | Sensor-to-UART latency | < 20 ms |
| NFR-10 | 30 min run, 100 Hz, dropped frames | 0 |
| NFR-11 | CPU load | < 30 % |
| NFR-12 | Flash footprint | < 64 KB |
| NFR-13 | Static RAM footprint | < 12 KB |

Functional behaviour, calibration procedure, telemetry frame (84 bytes, 50 Hz, CRC16-CCITT) and test cases T-01…T-15 are unchanged.

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
| Project name | `10dof` (no spaces, ASCII) |
| Toolchain / IDE | **CMake** |
| Compiler | **GCC** |
| Application structure | Advanced |
| Generate under root | Yes |
| Keep user code | Yes |
| Delete previously generated files | **No** (protects user files) |

### 7.3 Peripheral configuration (equivalent to SPEC.md §4)

| Peripheral | Setting |
|---|---|
| RCC | HSE bypass 8 MHz (ST-LINK MCO on PF0), PREDIV /1, PLL ×9 → 72 MHz; AHB /1; APB1 /2 (36 MHz); APB2 /1; USB /1.5 (48 MHz); I2C1/I2C2 ← HSI; USART1 ← PCLK2 |
| Flash | 2 wait states, prefetch on, I-cache on |
| I2C1 | PB6/PB7, fast mode 400 kHz, analog filter on, digital filter 0 (LSM303DLHC, on-board) |
| I2C2 | PA9 SCL / PA10 SDA (AF4, open-drain, no internal pull), fast mode 400 kHz, analog filter on, digital filter 0 (BMP280) |
| SPI1 | PA5/PA6/PA7, master, 8-bit, MSB first, mode 3, prescaler /16 (4.5 MHz), SW NSS |
| GPIO | PE3 output push-pull, initial high (L3GD20 CS) |
| USART1 | PC4 TX, PC5 RX, 460800 8N1 |
| TIM6 | 1 kHz update interrupt |
| SYS | SWD, no stop debug |
| NVIC | TIM6 prio 3; I2C1/I2C2/SPI1 event prio 2; nothing at prio 0 |

### 7.4 Headless regeneration

CubeMX can regenerate from a script, which makes the `.ioc` reviewable in CI:

```text
# tools/regenerate.txt
config load 10dof.ioc
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
├── SPEC.md                  # product + CubeIDE baseline
├── SPEC-CMAKE.md            # this document
├── README.md
├── 10dof.ioc
├── CMakeLists.txt           # user-owned, generated once
├── CMakePresets.json        # user-owned
├── STM32F303VCTx_FLASH.ld
├── cmake/
│   ├── gcc-arm-none-eabi.cmake
│   └── stm32cubemx/CMakeLists.txt     # tool-owned
├── Core/                    # tool-owned (USER CODE blocks only)
├── Drivers/                 # HAL + CMSIS (tool-owned)
├── App/                     # our code
├── Sensors/
├── Fusion/
├── Bsp/
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
| `10dof` | EXECUTABLE | links `app`, `fusion`, `sensors`, `bsp`, `stm32cubemx` |
| `10dof.hex` / `10dof.bin` | custom | post-build `objcopy` |

Rationale: static libraries keep include directories honest, make host-testing trivial (link the same `fusion`/`sensors` sources natively), and give `--gc-sections` something to do.

### 9.2 Root `CMakeLists.txt` (user-owned, illustrative)

```cmake
cmake_minimum_required(VERSION 3.25)
project(10dof C ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

# Tool-owned: defines the stm32cubemx interface target, HAL sources, defines
include(cmake/stm32cubemx/CMakeLists.txt)

# --- our code ---------------------------------------------------------
add_library(bsp     STATIC Bsp/Src/bsp_i2c.c Bsp/Src/bsp_spi.c Bsp/Src/bsp_time.c)
add_library(sensors STATIC Sensors/Src/bmp280.c Sensors/Src/lsm303dlhc.c Sensors/Src/l3gd20.c)
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
    -Wl,--gc-sections -Wl,--print-memory-usage -Wl,-Map=$<TARGET_FILE_DIR:${PROJECT_NAME}.elf>/10dof.map)

# --- artifacts + budget ----------------------------------------------
add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex   $<TARGET_FILE:${PROJECT_NAME}.elf> $<TARGET_FILE_DIR:${PROJECT_NAME}.elf>/10dof.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${PROJECT_NAME}.elf> $<TARGET_FILE_DIR:${PROJECT_NAME}.elf>/10dof.bin
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_SOURCE_DIR}/tools/size_check.py
            --elf $<TARGET_FILE:${PROJECT_NAME}.elf> --flash-max 65536 --ram-max 12288)

# --- flash target -----------------------------------------------------
add_custom_target(flash
    COMMAND ${CMAKE_COMMAND} -E echo "Flashing ${PROJECT_NAME}.elf"
    COMMAND STM32_Programmer_CLI -c port=SWD -w $<TARGET_FILE:${PROJECT_NAME}.elf> -v -rst
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
      "cacheVariables": { "CMAKE_BUILD_TYPE": "MinSizeRel" } }
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

- flash = `.isr_vector + .text + .rodata + .data`
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
ls build/Debug/10dof.{elf,hex,bin,map}

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
        -c "program build/Debug/10dof.elf verify reset exit"
```

```bash
arm-none-eabi-gdb build/Debug/10dof.elf \
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
      - run: python3 tools/size_check.py --elf build/ci/10dof.elf --flash-max 65536 --ram-max 12288
      - uses: actions/upload-artifact@v4
        with: { name: firmware, path: build/ci/10dof.{elf,hex,bin,map} }
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
