# Graph Report - 010_9dof_bmp280_cmake  (2026-11-10)

## Corpus Check
- 91 files · ~466,190 words
- Verdict: corpus is large enough that graph structure adds value.
- Unclassified: 5 file(s) not represented in the graph (top: (none) 2, .ioc 1, .ld 1)

## Summary
- 2012 nodes · 4772 edges · 45 communities (35 shown, 10 thin omitted)
- Extraction: 96% EXTRACTED · 4% INFERRED · 0% AMBIGUOUS · INFERRED: 205 edges (avg confidence: 0.85)
- Token cost: 60,275 input · 0 output

## Community Hubs (Navigation)
- LL I2C Driver
- CMSIS GCC Intrinsics
- LL SPI/I2S Driver
- HAL DMA Driver
- CMSIS ARMClang Intrinsics
- LL DMA Driver
- HAL USB PCD
- HAL RCC Clocking
- Cortex-M0+ Core
- CMSIS Device & Compiler
- LL System/DBGMCU
- CMSIS IAR Intrinsics
- App Init & Peripherals
- HAL Core & Cortex
- LL RCC (A)
- LL RCC (B)
- HAL EXTI & main.h
- ARMv8-M Mainline Core
- Cortex-M33 Core
- HAL Flash Driver
- ARMv8-M Baseline Core
- Cortex-M23 Core
- LL EXTI Driver
- Build System & Spec
- LL Cortex Utilities
- CMSIS ARMCC Intrinsics
- LL GPIO Driver
- LL PWR Driver
- Newlib Syscalls
- Cortex-M3 Core
- SC300 Core
- Cortex-M1 Core
- LL Bus Clocks
- LL Utils
- Packed Access Helpers (A)
- Packed Access Helpers (B)
- STARM stdio Hooks
- Syscall Timing
- Packed uint32 Type

## God Nodes (most connected - your core abstractions)
1. `__DSB()` - 44 edges
2. `HAL_GetTick()` - 35 edges
3. `__ISB()` - 32 edges
4. `I2C_Enable_IRQ()` - 29 edges
5. `I2C_TransferConfig()` - 25 edges
6. `HAL_DMA_Start_IT()` - 19 edges
7. `I2C_ITError()` - 19 edges
8. `I2C_Disable_IRQ()` - 17 edges
9. `I2C_WaitOnFlagUntilTimeout()` - 15 edges
10. `SPI_WaitFlagStateUntilTimeout()` - 14 edges

## Surprising Connections (you probably didn't know these)
- `main()` --calls--> `HAL_Init()`  [INFERRED]
  Core/Src/main.c → Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal.c
- `SystemClock_Config()` --calls--> `HAL_RCCEx_PeriphCLKConfig()`  [INFERRED]
  Core/Src/main.c → Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_rcc_ex.c
- `SystemClock_Config()` --calls--> `HAL_RCC_ClockConfig()`  [INFERRED]
  Core/Src/main.c → Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_rcc.c
- `SystemClock_Config()` --calls--> `HAL_RCC_OscConfig()`  [INFERRED]
  Core/Src/main.c → Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_rcc.c
- `MX_I2C1_Init()` --calls--> `HAL_I2C_Init()`  [INFERRED]
  Core/Src/main.c → Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_i2c.c

## Import Cycles
- None detected.

## Hyperedges (group relationships)
- **Sensors sharing I2C1 bus** — spec_i2c1_bus, spec_bmp280, spec_lsm303dlhc [EXTRACTED 1.00]
- **CubeMX-generated CMake build chain** — cmakelists, cmakelists_010_9dof_bmp280_cmake, cmake_stm32cubemx_cmakelists, cmake_stm32cubemx_cmakelists_stm32cubemx, cmake_stm32cubemx_cmakelists_stm32_drivers [EXTRACTED 1.00]
- **Vendor driver license set** — drivers_cmsis_license, drivers_cmsis_device_st_stm32f3xx_license, drivers_stm32f3xx_hal_driver_license [INFERRED 0.85]

## Communities (45 total, 10 thin omitted)

### Community 0 - "LL I2C Driver"
Cohesion: 0.04
Nodes (134): LL_I2C_AcknowledgeNextData(), LL_I2C_ClearFlag_ADDR(), LL_I2C_ClearFlag_ARLO(), LL_I2C_ClearFlag_BERR(), LL_I2C_ClearFlag_NACK(), LL_I2C_ClearFlag_OVR(), LL_I2C_ClearFlag_STOP(), LL_I2C_ClearFlag_TXE() (+126 more)

### Community 1 - "CMSIS GCC Intrinsics"
Cohesion: 0.03
Nodes (132): __CLREX(), __disable_fault_irq(), __disable_irq(), __DMB(), __enable_fault_irq(), __enable_irq(), __get_APSR(), __get_BASEPRI() (+124 more)

### Community 2 - "LL SPI/I2S Driver"
Cohesion: 0.05
Nodes (118): LL_I2S_ClearFlag_FRE(), LL_I2S_ClearFlag_OVR(), LL_I2S_ClearFlag_UDR(), LL_I2S_Disable(), LL_I2S_DisableDMAReq_RX(), LL_I2S_DisableDMAReq_TX(), LL_I2S_DisableIT_ERR(), LL_I2S_DisableIT_RXNE() (+110 more)

### Community 3 - "HAL DMA Driver"
Cohesion: 0.07
Nodes (99): DMA_CalcBaseAndBitshift(), DMA_SetConfig(), HAL_DMA_Abort(), HAL_DMA_Abort_IT(), HAL_DMA_DeInit(), HAL_DMA_GetError(), HAL_DMA_GetState(), HAL_DMA_Init() (+91 more)

### Community 4 - "CMSIS ARMClang Intrinsics"
Cohesion: 0.04
Nodes (108): __get_APSR(), __get_BASEPRI(), __get_CONTROL(), __get_FAULTMASK(), __get_IPSR(), __get_MSP(), __get_MSPLIM(), __get_PRIMASK() (+100 more)

### Community 5 - "LL DMA Driver"
Cohesion: 0.06
Nodes (94): LL_DMA_ClearFlag_GI1(), LL_DMA_ClearFlag_GI2(), LL_DMA_ClearFlag_GI3(), LL_DMA_ClearFlag_GI4(), LL_DMA_ClearFlag_GI5(), LL_DMA_ClearFlag_GI6(), LL_DMA_ClearFlag_GI7(), LL_DMA_ClearFlag_HT1() (+86 more)

### Community 6 - "HAL USB PCD"
Cohesion: 0.06
Nodes (73): HAL_PCDEx_BCD_Callback(), HAL_PCDEx_LPM_Callback(), HAL_PCDEx_PMAConfig(), HAL_PCDEx_SetConnectionState(), HAL_PCD_ActivateRemoteWakeup(), HAL_PCD_ConnectCallback(), HAL_PCD_DataInStageCallback(), HAL_PCD_DataOutStageCallback() (+65 more)

### Community 7 - "HAL RCC Clocking"
Cohesion: 0.06
Nodes (66): HAL_GetTick(), HAL_RCCEx_GetPeriphCLKConfig(), HAL_RCCEx_GetPeriphCLKFreq(), HAL_RCCEx_PeriphCLKConfig(), RCC_GetPLLCLKFreq(), HAL_RCC_ClockConfig(), HAL_RCC_CSSCallback(), HAL_RCC_DeInit() (+58 more)

### Community 8 - "Cortex-M0+ Core"
Cohesion: 0.06
Nodes (73): __DSB(), __ISB(), __NVIC_ClearPendingIRQ(), NVIC_DecodePriority(), __NVIC_DisableIRQ(), __NVIC_EnableIRQ(), NVIC_EncodePriority(), __NVIC_GetEnableIRQ() (+65 more)

### Community 9 - "CMSIS Device & Compiler"
Cohesion: 0.06
Nodes (54): T_UINT16_READ(), T_UINT16_WRITE(), T_UINT32(), T_UINT32_READ(), T_UINT32_WRITE(), __NVIC_ClearPendingIRQ(), NVIC_DecodePriority(), __NVIC_DisableIRQ() (+46 more)

### Community 10 - "LL System/DBGMCU"
Cohesion: 0.06
Nodes (64): LL_DBGMCU_APB1_GRP1_FreezePeriph(), LL_DBGMCU_APB1_GRP1_UnFreezePeriph(), LL_DBGMCU_APB2_GRP1_FreezePeriph(), LL_DBGMCU_APB2_GRP1_UnFreezePeriph(), LL_DBGMCU_DisableDBGSleepMode(), LL_DBGMCU_DisableDBGStandbyMode(), LL_DBGMCU_DisableDBGStopMode(), LL_DBGMCU_EnableDBGSleepMode() (+56 more)

### Community 11 - "CMSIS IAR Intrinsics"
Cohesion: 0.06
Nodes (52): __CLZ(), __get_APSR(), __get_MSPLIM(), __get_PSPLIM(), __iar_u32(), __iar_uint16_read(), __iar_uint16_write(), __iar_uint32_read() (+44 more)

### Community 12 - "App Init & Peripherals"
Cohesion: 0.06
Nodes (27): Error_Handler(), main(), MX_GPIO_Init(), MX_I2C1_Init(), MX_I2C2_Init(), MX_SPI1_Init(), MX_USB_PCD_Init(), SystemClock_Config() (+19 more)

### Community 13 - "HAL Core & Cortex"
Cohesion: 0.05
Nodes (26): HAL_MspInit(), SysTick_Handler(), HAL_MPU_ConfigRegion(), HAL_NVIC_ClearPendingIRQ(), HAL_NVIC_DisableIRQ(), HAL_NVIC_EnableIRQ(), HAL_NVIC_GetActive(), HAL_NVIC_GetPendingIRQ() (+18 more)

### Community 14 - "LL RCC (A)"
Cohesion: 0.04
Nodes (52): LL_RCC_ClearFlag_HSECSS(), LL_RCC_ClearFlag_LSERDY(), LL_RCC_ClearFlag_PLLRDY(), LL_RCC_ClearResetFlags(), LL_RCC_DisableIT_HSIRDY(), LL_RCC_DisableIT_LSIRDY(), LL_RCC_DisableIT_PLLRDY(), LL_RCC_DisableRTC() (+44 more)

### Community 15 - "LL RCC (B)"
Cohesion: 0.04
Nodes (52): LL_RCC_ClearFlag_HSERDY(), LL_RCC_ClearFlag_HSIRDY(), LL_RCC_ClearFlag_LSIRDY(), LL_RCC_ConfigMCO(), LL_RCC_DisableIT_HSERDY(), LL_RCC_DisableIT_LSERDY(), LL_RCC_EnableIT_LSIRDY(), LL_RCC_EnableIT_PLLRDY() (+44 more)

### Community 16 - "HAL EXTI & main.h"
Cohesion: 0.06
Nodes (13): HAL_EXTI_ClearConfigLine(), HAL_EXTI_ClearPending(), HAL_EXTI_GenerateSWI(), HAL_EXTI_GetConfigLine(), HAL_EXTI_GetHandle(), HAL_EXTI_GetPending(), HAL_EXTI_IRQHandler(), HAL_EXTI_RegisterCallback() (+5 more)

### Community 17 - "ARMv8-M Mainline Core"
Cohesion: 0.12
Nodes (38): ITM_CheckChar(), ITM_ReceiveChar(), ITM_SendChar(), __NVIC_ClearPendingIRQ(), NVIC_ClearTargetState(), NVIC_DecodePriority(), __NVIC_DisableIRQ(), __NVIC_EnableIRQ() (+30 more)

### Community 18 - "Cortex-M33 Core"
Cohesion: 0.12
Nodes (38): ITM_CheckChar(), ITM_ReceiveChar(), ITM_SendChar(), __NVIC_ClearPendingIRQ(), NVIC_ClearTargetState(), NVIC_DecodePriority(), __NVIC_DisableIRQ(), __NVIC_EnableIRQ() (+30 more)

### Community 20 - "HAL Flash Driver"
Cohesion: 0.13
Nodes (28): FLASH_MassErase(), FLASH_OB_DisableWRP(), FLASH_OB_EnableWRP(), FLASH_OB_GetRDP(), FLASH_OB_GetUser(), FLASH_OB_GetWRP(), FLASH_OB_ProgramData(), FLASH_OB_RDP_LevelConfig() (+20 more)

### Community 21 - "ARMv8-M Baseline Core"
Cohesion: 0.15
Nodes (31): __NVIC_ClearPendingIRQ(), NVIC_ClearTargetState(), NVIC_DecodePriority(), __NVIC_DisableIRQ(), __NVIC_EnableIRQ(), NVIC_EncodePriority(), __NVIC_GetActive(), __NVIC_GetEnableIRQ() (+23 more)

### Community 22 - "Cortex-M23 Core"
Cohesion: 0.15
Nodes (31): __NVIC_ClearPendingIRQ(), NVIC_ClearTargetState(), NVIC_DecodePriority(), __NVIC_DisableIRQ(), __NVIC_EnableIRQ(), NVIC_EncodePriority(), __NVIC_GetActive(), __NVIC_GetEnableIRQ() (+23 more)

### Community 23 - "LL EXTI Driver"
Cohesion: 0.11
Nodes (32): LL_EXTI_ClearFlag_0_31(), LL_EXTI_ClearFlag_32_63(), LL_EXTI_DisableEvent_0_31(), LL_EXTI_DisableEvent_32_63(), LL_EXTI_DisableFallingTrig_0_31(), LL_EXTI_DisableFallingTrig_32_63(), LL_EXTI_DisableIT_0_31(), LL_EXTI_DisableIT_32_63() (+24 more)

### Community 24 - "Build System & Spec"
Cohesion: 0.08
Nodes (22): STM32_Drivers OBJECT library, stm32cubemx INTERFACE library, Root CMakeLists.txt, 010_9dof_bmp280_cmake executable target, CMSIS Device STM32F3xx License, CMSIS Apache License 2.0, STM32F3xx HAL Driver License, SPEC: 10-DOF IMU Node (CMake variant) (+14 more)

### Community 25 - "LL Cortex Utilities"
Cohesion: 0.14
Nodes (25): LL_CPUID_GetConstant(), LL_CPUID_GetImplementer(), LL_CPUID_GetParNo(), LL_CPUID_GetRevision(), LL_CPUID_GetVariant(), LL_HANDLER_DisableFault(), LL_HANDLER_EnableFault(), LL_LPM_DisableEventOnPend() (+17 more)

### Community 26 - "CMSIS ARMCC Intrinsics"
Cohesion: 0.14
Nodes (21): __get_APSR(), __get_BASEPRI(), __get_CONTROL(), __get_FAULTMASK(), __get_FPSCR(), __get_IPSR(), __get_MSP(), __get_PRIMASK() (+13 more)

### Community 27 - "LL GPIO Driver"
Cohesion: 0.21
Nodes (23): LL_GPIO_GetAFPin_0_7(), LL_GPIO_GetAFPin_8_15(), LL_GPIO_GetPinMode(), LL_GPIO_GetPinOutputType(), LL_GPIO_GetPinPull(), LL_GPIO_GetPinSpeed(), LL_GPIO_IsAnyPinLocked(), LL_GPIO_IsInputPinSet() (+15 more)

### Community 28 - "LL PWR Driver"
Cohesion: 0.15
Nodes (24): LL_PWR_ClearFlag_SB(), LL_PWR_ClearFlag_WU(), LL_PWR_DisableBkUpAccess(), LL_PWR_DisablePVD(), LL_PWR_DisableSDADC(), LL_PWR_DisableWakeUpPin(), LL_PWR_EnableBkUpAccess(), LL_PWR_EnablePVD() (+16 more)

### Community 30 - "Cortex-M3 Core"
Cohesion: 0.19
Nodes (21): ITM_CheckChar(), ITM_ReceiveChar(), ITM_SendChar(), __NVIC_ClearPendingIRQ(), NVIC_DecodePriority(), __NVIC_DisableIRQ(), __NVIC_EnableIRQ(), NVIC_EncodePriority() (+13 more)

### Community 31 - "SC300 Core"
Cohesion: 0.19
Nodes (21): ITM_CheckChar(), ITM_ReceiveChar(), ITM_SendChar(), __NVIC_ClearPendingIRQ(), NVIC_DecodePriority(), __NVIC_DisableIRQ(), __NVIC_EnableIRQ(), NVIC_EncodePriority() (+13 more)

### Community 32 - "Cortex-M1 Core"
Cohesion: 0.25
Nodes (15): __NVIC_ClearPendingIRQ(), NVIC_DecodePriority(), __NVIC_DisableIRQ(), __NVIC_EnableIRQ(), NVIC_EncodePriority(), __NVIC_GetEnableIRQ(), __NVIC_GetPendingIRQ(), __NVIC_GetPriority() (+7 more)

### Community 35 - "LL Bus Clocks"
Cohesion: 0.22
Nodes (15): LL_AHB1_GRP1_DisableClock(), LL_AHB1_GRP1_EnableClock(), LL_AHB1_GRP1_ForceReset(), LL_AHB1_GRP1_IsEnabledClock(), LL_AHB1_GRP1_ReleaseReset(), LL_APB1_GRP1_DisableClock(), LL_APB1_GRP1_EnableClock(), LL_APB1_GRP1_ForceReset() (+7 more)

### Community 36 - "LL Utils"
Cohesion: 0.48
Nodes (5): LL_GetFlashSize(), LL_GetUID_Word0(), LL_GetUID_Word1(), LL_GetUID_Word2(), LL_InitTick()

### Community 37 - "Packed Access Helpers (A)"
Cohesion: 0.40
Nodes (4): T_UINT16_READ(), T_UINT16_WRITE(), T_UINT32_READ(), T_UINT32_WRITE()

### Community 38 - "Packed Access Helpers (B)"
Cohesion: 0.40
Nodes (4): T_UINT16_READ(), T_UINT16_WRITE(), T_UINT32_READ(), T_UINT32_WRITE()

## Knowledge Gaps
- **10 isolated node(s):** `v`, `CMSIS Device STM32F3xx License`, `CMSIS Apache License 2.0`, `LSM303DLHC accel + mag (I2C1)`, `L3GD20 / I3G4250D gyro (SPI1, CS=PE3)` (+5 more)
  These have ≤1 connection - possible missing edges or undocumented components. (Counts symbols only; 196 node(s) total have ≤1 connection when file, concept and rationale nodes are included.)
- **10 thin communities (<3 nodes) omitted from report** — run `graphify query` to explore isolated nodes.

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **Why does `LL_MPU_Enable()` connect `LL Cortex Utilities` to `Cortex-M0+ Core`?**
  _High betweenness centrality (0.049) - this node is a cross-community bridge._
- **What connects `v`, `CMSIS Device STM32F3xx License`, `CMSIS Apache License 2.0` to the rest of the system?**
  _10 weakly-connected nodes found - possible documentation gaps or missing edges._
- **Should `LL I2C Driver` be split into smaller, more focused modules?**
  _Cohesion score 0.043151567196221555 - nodes in this community are weakly interconnected._
- **Why does `__DSB()` connect `Cortex-M0+ Core` to `Cortex-M1 Core`, `CMSIS GCC Intrinsics`, `CMSIS Device & Compiler`, `ARMv8-M Mainline Core`, `Cortex-M33 Core`, `ARMv8-M Baseline Core`, `Cortex-M23 Core`, `LL Cortex Utilities`, `Cortex-M3 Core`, `SC300 Core`?**
  _High betweenness centrality (0.037) - this node is a cross-community bridge._
- **Should `CMSIS GCC Intrinsics` be split into smaller, more focused modules?**
  _Cohesion score 0.02962630456738862 - nodes in this community are weakly interconnected._
- **Why does `__disable_irq()` connect `CMSIS GCC Intrinsics` to `App Init & Peripherals`?**
  _High betweenness centrality (0.034) - this node is a cross-community bridge._
- **Should `LL SPI/I2S Driver` be split into smaller, more focused modules?**
  _Cohesion score 0.05206611570247934 - nodes in this community are weakly interconnected._