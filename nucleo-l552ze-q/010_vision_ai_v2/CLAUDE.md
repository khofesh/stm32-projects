# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

STM32L5 firmware project for interfacing with the Grove Vision AI Module V2. Built for the NUCLEO-L552ZE-Q board using STM32CubeIDE and the ARM GCC toolchain.

## Build Commands

Build from the Debug directory using `make`:

```bash
cd Debug && make all
```

Clean build:
```bash
cd Debug && make clean
```

The project is configured for STM32CubeIDE - the preferred method is to import and build within the IDE.

## Hardware Configuration

- **MCU**: STM32L552ZETxQ (Cortex-M33, 110MHz)
- **Board**: NUCLEO-L552ZE-Q
- **Peripherals configured**:
  - I2C1: Grove Vision AI communication
  - UART4 with DMA: Grove Vision AI communication (alternative)
  - SPI1: Grove Vision AI communication (alternative)
  - LPUART1 (COM1): Debug output at 115200 baud

## Architecture

### SSCMA Driver (`Drivers/sscma/`)

The custom driver communicates with Seeed SenseCraft Model Assistant (SSCMA) devices:

- `sscma_stm32l5.h/c`: Main driver supporting I2C, UART, and SPI interfaces
- `sscma_parser.h/c`: JSMN-based JSON parser for SSCMA responses

Key driver features:
- Multiple interface support via `sscma_begin_i2c()`, `sscma_begin_uart()`, `sscma_begin_spi()`
- Inference results: bounding boxes, classifications, points, keypoints (pose estimation)
- Non-blocking fetch with `sscma_fetch()` for periodic polling
- AT command protocol with JSON responses

### External Dependencies

- **jsmn**: Lightweight JSON parser (linked from `../../libraries/jsmn`)

### STM32 HAL Layer

Standard STM32CubeMX-generated structure:
- `Core/Src/main.c`: Application entry point and peripheral initialization
- `Core/Inc/stm32l5xx_hal_conf.h`: HAL module configuration
- `Drivers/STM32L5xx_HAL_Driver/`: HAL driver sources
- `Drivers/BSP/STM32L5xx_Nucleo/`: Board support package

## Code Conventions

- User code sections marked with `/* USER CODE BEGIN */` and `/* USER CODE END */` are preserved during STM32CubeMX regeneration
- Peripheral initialization follows STM32CubeMX naming: `MX_<PERIPHERAL>_Init()`
- Project uses compile-time configuration via preprocessor defines in `.cproject`

## Preprocessor Defines

- `STM32L552xx`: Target MCU
- `USE_HAL_DRIVER`: Enable HAL drivers
- `USE_NUCLEO_144`: Board configuration
- `DEBUG`: Debug build (Debug configuration only)
