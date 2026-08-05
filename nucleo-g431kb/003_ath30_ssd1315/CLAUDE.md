<!--
You can leave out this comment section in your actual AGENTS.md file.

Prerequisites:
- tree (https://formulae.brew.sh/formula/tree)
- rtk (https://github.com/rtk-ai/rtk)
- Graphify (https://github.com/safishamsi/graphify)
- ast-grep (https://github.com/ast-grep/ast-grep)
-->

# claude

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

## Code Style

- C (C11), STM32CubeIDE / GCC ARM toolchain, HAL drivers
- Never write outside the `/* USER CODE BEGIN X */` ... `/* USER CODE END X */` markers in CubeMX-generated files — code outside is lost on regeneration
- Application code goes in `Core/Src` and `Core/Inc`; reusable drivers in their own `.c`/`.h` pair
- Use fixed-width types from `<stdint.h>` (`uint8_t`, `int32_t`), not `int`/`long`
- `static` for file-local functions and variables; no globals unless shared across translation units
- Peripheral handles (`huart1`, `hi2c1`, ...) come from CubeMX — declare with `extern` in headers, never redefine
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

<!-- rtk-instructions v2 -->

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

<!-- /rtk-instructions -->

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
