# Toolchain

Reference build environment for `010_9dof_bmp280_cmake` (STM32F303VCT6).

## Versions used on the bench (2026-11-12)

| Tool | Version | Notes |
|---|---|---|
| arm-none-eabi-gcc | 15.2.0 | `arm-none-eabi-{gcc,objcopy,size,gdb}` on `PATH` |
| CMake | 4.3.0 | presets require ≥ 3.22 |
| Ninja | 1.13.2 | generator for all presets |
| OpenOCD | 0.12.0+dev | `interface/stlink.cfg`, `target/stm32f3x.cfg` |
| Python | 3.14.10 | `tools/size_check.py` (post-build) |
| STM32CubeMX | 6.15+ | regeneration only; not needed to build |

`arm-none-eabi-` must be on `PATH`; the toolchain file sets every other flag.

## Build

```bash
cmake --preset Debug   && cmake --build --preset Debug
cmake --preset Release && cmake --build --preset Release
cmake --preset ci      && cmake --build --preset ci      # -Os + -Werror
```

| Preset | Optimization | Warnings |
|---|---|---|
| `Debug` | `-Og -g3` | `-Wall -Wextra -Wshadow -Wundef -Wdouble-promotion -Wfloat-equal` |
| `Release` | `-O2` | same |
| `ci` | `-Os` (MinSizeRel) | same, plus `-Werror` (`OUR_WERROR=ON`) |

`-Werror` covers `App/`, `Bsp/`, `Sensors/`, `Fusion/`; `Drivers/` (HAL) is not
affected. Every link produces, in `build/<preset>/`:

- `010_9dof_bmp280_cmake.elf`, `.hex`, `.bin`, `.map`
- a size report from `tools/size_check.py`; the build fails above 64 KB flash or
  12 KB RAM (override with `-DFLASH_BUDGET=` / `-DRAM_BUDGET=`)

The banner and (later) the debug frame embed `git describe --always --dirty` as
`BUILD_GIT_DESC`, refreshed on every build into `build/<preset>/gen/build_info.h`
(the `build_info` target; the file is rewritten only when the description changes).

## Host tests

```bash
cmake -S tests -B build/tests -DCMAKE_BUILD_TYPE=Debug
cmake --build build/tests
ctest --test-dir build/tests --output-on-failure
```

## Flash and debug

```bash
cmake --build --preset Debug --target flash        # openocd program ... verify reset exit
```

Manual OpenOCD / GDB:

```bash
openocd -f interface/stlink.cfg -f target/stm32f3x.cfg \
        -c "program build/Debug/010_9dof_bmp280_cmake.elf verify reset exit"

openocd -f interface/stlink.cfg -f target/stm32f3x.cfg &
arm-none-eabi-gdb build/Debug/010_9dof_bmp280_cmake.elf \
  -ex "target extended-remote :3333" -ex "monitor reset halt" -ex "load"
```

The `flash` target exists only if `openocd` is found at configure time.

## Serial console

ST-LINK/V2-1 VCP → `/dev/ttyACM0` (CDC, baud setting is ignored):

```bash
python3 -m serial.tools.miniterm /dev/ttyACM0 460800
```

M0 sends `\r\n010_9dof_bmp280_cmake <git-desc>\r\n` once at boot; the message is
easy to miss because it is emitted before the host opens the port.

## Regeneration

```bash
STM32CubeMX -q tools/regenerate.txt
git diff --exit-code -- Core Drivers cmake/stm32cubemx CMakeLists.txt
```

If regeneration removes the user blocks in `Core/`, the `.ioc` was changed
out-of-band — investigate instead of re-applying them by hand.

## Ownership reminders

| Path | Owner |
|---|---|
| `Core/`, `Drivers/`, `cmake/stm32cubemx/`, `startup_*.s`, `*_FLASH.ld` | CubeMX |
| `CMakeLists.txt`, `CMakePresets.json`, `cmake/gcc-arm-none-eabi.cmake` | user |
| `App/`, `Sensors/`, `Fusion/`, `Bsp/`, `tests/`, `tools/`, `docs/` | user |

If a CubeMX upgrade renames the generated `stm32cubemx` interface target, fix
the root `CMakeLists.txt` only.
