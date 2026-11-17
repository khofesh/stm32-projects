# agents

## STM32CubeMX

- if you need to change the configuration in stm32cubemx, don't make changes in `*.ioc` file. Show step-by-step guide instead

## Response Style

- Be extremely concise. No pleasantries, no filler.
- When asked to write code, return code only unless explanation is explicitly requested.
- No sycophantic preambles ("Sure!", "Great question!", "Absolutely!").
- No "Here's a function that..." preambles.
- Don't restate the question before answering.
- No "Note:", "Tip:", or "Remember:" appendices unless asked.
- No usage examples unless asked.
- No unsolicited suggestions or improvements beyond what was asked.
- Use short variable names where meaning is clear from context.

## Project Structure

T-16 SPI master for the `../010_9dof_bmp280_cmake` SPI2 slave. Spec: `../010_9dof_bmp280_cmake/docs/SPEC.md` §23 (frame), §23.6 (SPI2 transport), §24 T-16. Target: NUCLEO-L432KC (STM32L432KCU6), 32 MHz.

| Path | Owner | Edit |
|---|---|---|
| `010_stm32l4_spi_master.ioc` | CubeMX | Never by hand — give step-by-step CubeMX guide |
| `CMakeLists.txt` (root) | User | Yes — attach all our targets here |
| `CMakePresets.json` | User | Yes |
| `cmake/gcc-arm-none-eabi.cmake` | User | Rarely |
| `cmake/starm-clang.cmake` | CubeMX | No (unused) |
| `cmake/stm32cubemx/CMakeLists.txt` | CubeMX | No |
| `STM32L432xx_FLASH.ld`, `startup_stm32l432xx.s` | CubeMX | No |
| `Core/`, `Drivers/` | CubeMX | Only inside `USER CODE` blocks |
| `App/{Inc,Src}` | User | `frame_check` (portable: CRC, frame/sequence check, stats frame), `master` (HAL: SPI1 DMA, DRDY, USART2) |
| `tests/` | User | Host-only CMake tree (CTest) |
| `tools/` | User | `t16_compare.py` (SPI vs UART frame comparison, T-16 verdict) |

Pins: SPI1 PA1 SCK / PA11 MISO / PA12 MOSI (4 MHz, mode 0), PA4 `SPI_NSS` (GPIO), PB0 `SPI_DRDY` (EXTI both edges), PB1 `PROBE`, USART2 VCP 460800 (PA2/PA15), LD3 on PB3.

| F3DISCOVERY | L432KC | Header |
|---|---|---|
| PB13 SCK | PA1 | A1 |
| PB14 MISO | PA11 | D10 |
| PB15 MOSI | PA12 | D2 |
| PB12 NSS | PA4 | A3 |
| PB11 DRDY | PB0 | D3 |
| GND | GND | GND |

CMake targets: `stm32cubemx` (INTERFACE, generated) ← `app` (STATIC) → executable `010_stm32l4_spi_master`.

## Build

```bash
cmake --preset Debug && cmake --build --preset Debug      # or Release
cmake -S tests -B build/tests && cmake --build build/tests && ctest --test-dir build/tests --output-on-failure
```

Output in `build/<Preset>/`. Do not add `.cproject`/Eclipse metadata.

Flash (ST-LINK SN `0670FF574884654867012339`, VCP `/dev/ttyACM1`; F3DISCOVERY is SN `066BFF494877514867130941`, `/dev/ttyACM0`):

```bash
P=~/STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin/STM32_Programmer_CLI
$P -c port=SWD sn=0670FF574884654867012339 mode=UR -w build/Release/010_stm32l4_spi_master.elf -v
# FLASH_SR.PEMPTY (bit 17) is often left set after programming -> ROM bootloader on reset. Clear it (write 1), then reset.
$P -c port=SWD sn=0670FF574884654867012339 mode=Hotplug -w32 0x40022010 0x00020000   # only if bit 17 reads 1; reports an error but works
$P -c port=SWD sn=0670FF574884654867012339 mode=Hotplug -rst
```

Do not pipe the programmer into `grep -q`: SIGPIPE kills it before `-rst` and leaves the core halted.

T-16: `tools/t16_compare.py --f3 /dev/ttyACM0 --l4 /dev/ttyACM1 --duration 600 --pairs-at 300`

## Code Style

- C (C11), CMake + arm-none-eabi-gcc toolchain, HAL drivers
- Never write outside the `/* USER CODE BEGIN X */` ... `/* USER CODE END X */` markers in CubeMX-generated files — code outside is lost on regeneration
- Application code goes in `App/` (`Inc/` + `Src/`), not `Core/`; register new sources in the root `CMakeLists.txt`
- Portable modules (`frame_check`) must not include `stm32l4xx_hal.h`; HAL access stays in `master.c`
- Every numeric algorithm gets a host test vector in `tests/`
- Warnings for our code: `-Wall -Wextra -Wshadow -Wundef -Wdouble-promotion -Wfloat-equal`; budget flash < 64 KB, RAM < 12 KB
- Use fixed-width types from `<stdint.h>` (`uint8_t`, `int32_t`), not `int`/`long`
- `static` for file-local functions and variables; no globals unless shared across translation units
- Peripheral handles (`hspi1`, `huart2`, ...) come from CubeMX — declare with `extern` in headers, never redefine
- Check and handle `HAL_StatusTypeDef` return values; don't ignore them
- No dynamic allocation (`malloc`/`free`); use static buffers with explicit sizes
- Keep ISRs short — set flags, defer work to the main loop; shared ISR variables must be `volatile`
- No blocking `HAL_Delay()` in ISRs or long-running paths; prefer non-blocking / DMA / interrupt-driven APIs
- Header guards via `#ifndef __FILE_H` / `#define __FILE_H`, matching CubeMX style
- Minimal inline comments — only for genuinely complex logic or hardware quirks (register timing, errata)

## Context Retrieval Policy

Always retrieve the smallest amount of information necessary.
Escalate only when necessary. Stop escalating as soon as sufficient information has been obtained.

Preferred order:

1. Need code structure? `graph.json`.
2. Need symbols? `ast-grep`.
3. Need repository metadata? `rtk`.
4. Need implementation? Source files.
5. Use repository-wide search as last resort

Avoid reading entire directories or the whole repository unless explicitly requested.

## RTK (Rust Token Killer) - Token-Optimized CLI

`rtk` is a CLI proxy that filters and compresses command outputs, saving 60-90% tokens.

**Always** prefix commands with `rtk`. If RTK has a dedicated filter, it uses it. If not, it passes through unchanged. This means RTK is always safe to use.

Instead of:

```
ls -la .
git status
git log -10
docker ps
```

Use:

```
rtk ls -la .
rtk git status
rtk git log -10
rtk docker ps
```

### RTK commands example

```bash
rtk ls <path>
rtk read <file>
rtk find <pattern>

rtk err <cmd>           # Filter errors only from any command
rtk log <file>          # Deduplicated logs with counts
rtk json <file>         # JSON structure without values

rtk curl <url>          # Compact HTTP responses

rtk docker ps           # Compact container list
rtk docker images       # Compact image list
rtk docker logs <c>     # Deduplicated logs
```

## Graphify - Codebase context & knowledge graph protocol

A pre-computed AST knowledge graph (`graph.json`) is available at: `graphify-out/graph.json`
Use `graph.json` before searching or reading multiple source files.

Workflow:

1. Read `graph.json`.
2. Identify the relevant symbols, files, and dependency paths.
3. Read only the source files required for the task.
4. Avoid scanning unrelated files.

Use the graph for:

- dependency tracing
- call-path discovery
- identifying high-centrality modules
- impact analysis before refactoring
- locating symbol definitions

Never read an entire directory simply to locate a symbol. Use `graph.json` to locate the symbol first, then read only the relevant files.

The graph is generated from AST extraction and represents extracted structural relationships only. Treat graph edges as authoritative for structural relationships.
Do not infer dependencies that are absent from the graph.

## ast-grep

Prefer `ast-grep` over `grep` whenever searching source code.

Use `ast-grep` for:

- locating function definitions
- locating class definitions
- locating method implementations
- finding imports
- matching AST patterns
- performing structural search

Use `grep` only for:

- Markdown
- JSON
- YAML
- log files
- generated text
- plain-text configuration

Avoid using `grep` to search programming language source code when `ast-grep` can answer the query.
