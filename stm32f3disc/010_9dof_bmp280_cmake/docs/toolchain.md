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
| clang-format | 23.1.5 | pinned: `pipx install clang-format==23.1.5`; other majors format differently |
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

## Static analysis

Targets (all optional at configure time, gated in CI):

```bash
cmake --build --preset ci --target format-check   # clang-format --dry-run --Werror
cmake --build --preset ci --target cppcheck       # warning,style,performance
cmake --build --preset ci --target analyze        # -fanalyzer, advisory, build/analyze
```

`format-check` and `cppcheck` cover `App/`, `Bsp/`, `Fusion/` and `Sensors/{Src,Inc}`;
`Sensors/vendor/` (LibDriver) is excluded. `analyze` configures a separate
`build/analyze` tree with `-DOUR_FANALYZER=ON` so the normal build stays fast.

## Continuous integration

`.github/workflows/010_9dof_bmp280_cmake.yml` (monorepo root) runs on changes
under `stm32f3disc/010_9dof_bmp280_cmake/`: `ci` build with `-Werror`, host
tests, `format-check`, `cppcheck`, artifact upload and the two-path
reproducibility check. The size budget is enforced by the post-link step of the
build itself. The runner installs Arm GNU Toolchain **15.2.rel1** (the bench
compiler; tarball SHA-256 pinned, cached between runs) — an explicitly
documented equivalent of the CubeCLT bundle per TR-01 — clang-format 23.1.5 via
pipx, and uses CMake/Ninja from Ubuntu 24.04.

## Reproducibility

```bash
tools/repro_check.sh
```

Checks out `HEAD` (or the commit given as the first argument) into two git worktrees at different absolute paths, builds
both with the `ci` preset and compares the `.elf`, `.hex` and `.bin` SHA-256
sums (AC-10). The worktrees keep `git describe` working, so the compared
artifacts embed the same `BUILD_GIT_DESC` as a normal build of that commit; the
script fails if it comes out `unknown`. Uncommitted changes are not built
(warning printed). The toolchain file passes `-ffile-prefix-map` for the source
and build roots, so no absolute paths reach the debug info.

## Dev Container

`.devcontainer/` provides Ubuntu 24.04 with the same pinned Arm GNU toolchain,
CMake, Ninja, OpenOCD, clang-format, cppcheck and pyserial. The container gets
`/dev/bus/usb` so OpenOCD can reach the ST-LINK; start it from VS Code with
"Reopen in Container".

## Latency probe (T-11)

The probe is compiled only when `LAT_PROBE_Pin` exists in `Core/Inc/main.h`.
Add it in CubeMX (do not edit the `.ioc` by hand):

1. Open `010_9dof_bmp280_cmake.ioc` in CubeMX.
2. Pinout & Configuration → **System Core → GPIO**.
3. Click **PC0** (P2 pin 6 on the F3DISCOVERY) and select **GPIO_Output**.
4. In the GPIO configuration pane: output level **Low**, mode **Output Push Pull**,
   pull **No pull-up/pull-down**, speed **Low**, user label **LAT_PROBE**.
5. Generate code (Keep User Code = Yes). CubeMX emits `LAT_PROBE_Pin` /
   `LAT_PROBE_GPIO_Port`; `App/Src/app.c` then drives the pin high while the IMU
   is read.
6. Rebuild and flash. Trigger on the PC0 rising edge, capture USART1 PC4 TX on
   a logic analyser, and measure to the last byte of the next data frame
   (NFR-09: < 20 ms).

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

## Release checklist

1. `git status` clean; version tag set.
2. `cmake --preset ci && cmake --build --preset ci` (warnings are errors).
3. `ctest --test-dir build/tests --output-on-failure`.
4. `cmake --build --preset ci --target format-check` and `--target cppcheck`.
5. CubeMX regeneration check (AC-08): run the two commands above and require
   `git diff --exit-code`; if CubeMX CLI is unavailable on the runner, do it
   manually here and record it in `docs/test-log.md` (TR-13, Q-3).
6. `tools/repro_check.sh` (AC-10).
7. Archive `.elf`, `.hex`, `.bin`, `.map` from `build/ci/` with the release.

## Ownership reminders

| Path | Owner |
|---|---|
| `Core/`, `Drivers/`, `cmake/stm32cubemx/`, `startup_*.s`, `*_FLASH.ld` | CubeMX |
| `CMakeLists.txt`, `CMakePresets.json`, `cmake/gcc-arm-none-eabi.cmake` | user |
| `App/`, `Sensors/`, `Fusion/`, `Bsp/`, `tests/`, `tools/`, `docs/` | user |

If a CubeMX upgrade renames the generated `stm32cubemx` interface target, fix
the root `CMakeLists.txt` only.
