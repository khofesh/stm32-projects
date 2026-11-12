# Graph Report - 010_9dof_bmp280_cmake  (2026-11-12)

## Corpus Check
- 59 files · ~502,341 words
- Verdict: corpus is large enough that graph structure adds value.

## Summary
- 2228 nodes · 5175 edges · 70 communities (50 shown, 20 thin omitted)
- Extraction: 95% EXTRACTED · 5% INFERRED · 0% AMBIGUOUS · INFERRED: 272 edges (avg confidence: 0.85)
- Token cost: 81,118 input · 0 output

## Community Hubs (Navigation)
- LL I2C
- CMSIS GCC Intrinsics
- LL SPI/I2S
- HAL DMA
- CMSIS ARMClang
- LL DMA
- HAL USB PCD
- Core CM0+
- HAL SPI
- LL System/DBGMCU
- CMSIS IAR
- LL RCC Flags
- LL RCC Config
- HAL Core & Cortex
- Main Init
- BMP280 Driver
- System Init & LL Bus
- Core ARMv8-M Mainline
- Core CM33
- HAL Flash
- Core ARMv8-M Baseline
- Core CM23
- LL EXTI
- LSM303AGR Driver
- Spec Fusion & Calibration
- Core CM4 & Device
- CMSIS ARMCC
- LL Cortex
- LL GPIO
- LL PWR
- Core CM3
- Core SC300
- Syscalls
- ST Vendor Repo Docs
- I3G4250D Driver
- Core CM0
- Core CM1
- HAL RCC
- F3Disco BSP Core
- Core SC000
- HAL GPIO
- CMake Target Graph
- Plan Decisions
- HAL EXTI
- CMSIS Compiler
- HAL PWR Ex
- BSP Gyroscope
- HAL I2C Ex
- HAL RCC Ex
- HAL SPI Ex
- Spec Architecture & Telemetry
- Packed Helpers A
- Packed Helpers B
- STARM stdio
- Times Syscall
- ST Licenses
- T_UINT32
- CMSIS License

## God Nodes (most connected - your core abstractions)
1. `__DSB()` - 44 edges
2. `HAL_GetTick()` - 35 edges
3. `__ISB()` - 32 edges
4. `SPEC — 10-DOF IMU Node (CubeMX+CMake variant)` - 31 edges
5. `I2C_Enable_IRQ()` - 29 edges
6. `I2C_TransferConfig()` - 25 edges
7. `a_bmp280_iic_spi_read()` - 22 edges
8. `STMicroelectronics BSP Component Driver Repository` - 21 edges
9. `HAL_DMA_Start_IT()` - 19 edges
10. `I2C_ITError()` - 19 edges

## Surprising Connections (you probably didn't know these)
- `D-3 Keep executable name 010_9dof_bmp280_cmake` --references--> `Executable 010_9dof_bmp280_cmake`  [INFERRED]
  docs/PLAN.md → CMakeLists.txt
- `Current state (baseline d94912b)` --references--> `Root CMakeLists.txt`  [INFERRED]
  docs/PLAN.md → CMakeLists.txt
- `I3G4250D_LowPower()` --calls--> `GYRO_IO_Write()`  [INFERRED]
  Sensors/i3g4250d/i3g4250d.c → Bsp/stm32f3_discovery.c
- `I3G4250D_GetDataStatus()` --calls--> `GYRO_IO_Read()`  [INFERRED]
  Sensors/i3g4250d/i3g4250d.c → Bsp/stm32f3_discovery.c
- `I3G4250D_ReadXYZAngRate()` --calls--> `GYRO_IO_Read()`  [INFERRED]
  Sensors/i3g4250d/i3g4250d.c → Bsp/stm32f3_discovery.c

## Import Cycles
- None detected.

## Hyperedges (group relationships)
- **Vendor driver license set** — drivers_cmsis_license, drivers_cmsis_device_st_stm32f3xx_license, drivers_stm32f3xx_hal_driver_license [INFERRED 0.85]
- **CMake target dependency graph** — cmake_stm32cubemx_cmakelists_stm32cubemx, cmakelists_bsp, cmakelists_sensors, cmakelists_010_9dof_bmp280_cmake, cmake_stm32cubemx_cmakelists_stm32_drivers [EXTRACTED 1.00]
- **Vendored ST drivers slated for removal (D-1, task 1.9)** — docs_plan_d1_own_drivers, docs_plan_remove_st_bsp, sensors_i3g4250d_release_notes_i3g4250d_component_driver, sensors_lsm303agr_release_notes_lsm303agr_component_driver, sensors_common_release_notes_bsp_common_components [INFERRED 0.85]
- **Portable host-tested numeric modules** — docs_spec_host_unit_testing, docs_spec_madgwick, docs_spec_baro_filter, docs_spec_axis_map [EXTRACTED 1.00]

## Communities (70 total, 20 thin omitted)

### Community 0 - "LL I2C"
Cohesion: 0.04
Nodes (134): LL_I2C_AcknowledgeNextData(), LL_I2C_ClearFlag_ADDR(), LL_I2C_ClearFlag_ARLO(), LL_I2C_ClearFlag_BERR(), LL_I2C_ClearFlag_NACK(), LL_I2C_ClearFlag_OVR(), LL_I2C_ClearFlag_STOP(), LL_I2C_ClearFlag_TXE() (+126 more)

### Community 1 - "CMSIS GCC Intrinsics"
Cohesion: 0.03
Nodes (132): __CLREX(), __disable_fault_irq(), __disable_irq(), __DMB(), __enable_fault_irq(), __enable_irq(), __get_APSR(), __get_BASEPRI() (+124 more)

### Community 2 - "LL SPI/I2S"
Cohesion: 0.05
Nodes (118): LL_I2S_ClearFlag_FRE(), LL_I2S_ClearFlag_OVR(), LL_I2S_ClearFlag_UDR(), LL_I2S_Disable(), LL_I2S_DisableDMAReq_RX(), LL_I2S_DisableDMAReq_TX(), LL_I2S_DisableIT_ERR(), LL_I2S_DisableIT_RXNE() (+110 more)

### Community 3 - "HAL DMA"
Cohesion: 0.07
Nodes (99): DMA_CalcBaseAndBitshift(), DMA_SetConfig(), HAL_DMA_Abort(), HAL_DMA_Abort_IT(), HAL_DMA_DeInit(), HAL_DMA_GetError(), HAL_DMA_GetState(), HAL_DMA_Init() (+91 more)

### Community 4 - "CMSIS ARMClang"
Cohesion: 0.04
Nodes (108): __get_APSR(), __get_BASEPRI(), __get_CONTROL(), __get_FAULTMASK(), __get_IPSR(), __get_MSP(), __get_MSPLIM(), __get_PRIMASK() (+100 more)

### Community 5 - "LL DMA"
Cohesion: 0.06
Nodes (94): LL_DMA_ClearFlag_GI1(), LL_DMA_ClearFlag_GI2(), LL_DMA_ClearFlag_GI3(), LL_DMA_ClearFlag_GI4(), LL_DMA_ClearFlag_GI5(), LL_DMA_ClearFlag_GI6(), LL_DMA_ClearFlag_GI7(), LL_DMA_ClearFlag_HT1() (+86 more)

### Community 6 - "HAL USB PCD"
Cohesion: 0.06
Nodes (73): HAL_PCDEx_BCD_Callback(), HAL_PCDEx_LPM_Callback(), HAL_PCDEx_PMAConfig(), HAL_PCDEx_SetConnectionState(), HAL_PCD_ActivateRemoteWakeup(), HAL_PCD_ConnectCallback(), HAL_PCD_DataInStageCallback(), HAL_PCD_DataOutStageCallback() (+65 more)

### Community 7 - "Core CM0+"
Cohesion: 0.06
Nodes (74): __DSB(), __ISB(), __NVIC_ClearPendingIRQ(), NVIC_DecodePriority(), __NVIC_DisableIRQ(), __NVIC_EnableIRQ(), NVIC_EncodePriority(), __NVIC_GetEnableIRQ() (+66 more)

### Community 8 - "HAL SPI"
Cohesion: 0.09
Nodes (51): HAL_GetTick(), HAL_SPI_Abort(), HAL_SPI_Abort_IT(), HAL_SPI_AbortCpltCallback(), HAL_SPI_DeInit(), HAL_SPI_DMAPause(), HAL_SPI_DMAResume(), HAL_SPI_DMAStop() (+43 more)

### Community 9 - "LL System/DBGMCU"
Cohesion: 0.06
Nodes (64): LL_DBGMCU_APB1_GRP1_FreezePeriph(), LL_DBGMCU_APB1_GRP1_UnFreezePeriph(), LL_DBGMCU_APB2_GRP1_FreezePeriph(), LL_DBGMCU_APB2_GRP1_UnFreezePeriph(), LL_DBGMCU_DisableDBGSleepMode(), LL_DBGMCU_DisableDBGStandbyMode(), LL_DBGMCU_DisableDBGStopMode(), LL_DBGMCU_EnableDBGSleepMode() (+56 more)

### Community 10 - "CMSIS IAR"
Cohesion: 0.06
Nodes (52): __CLZ(), __get_APSR(), __get_MSPLIM(), __get_PSPLIM(), __iar_u32(), __iar_uint16_read(), __iar_uint16_write(), __iar_uint32_read() (+44 more)

### Community 11 - "LL RCC Flags"
Cohesion: 0.04
Nodes (52): LL_RCC_ClearFlag_HSECSS(), LL_RCC_ClearFlag_LSERDY(), LL_RCC_ClearFlag_PLLRDY(), LL_RCC_ClearResetFlags(), LL_RCC_DisableIT_HSIRDY(), LL_RCC_DisableIT_LSIRDY(), LL_RCC_DisableIT_PLLRDY(), LL_RCC_DisableRTC() (+44 more)

### Community 12 - "LL RCC Config"
Cohesion: 0.04
Nodes (52): LL_RCC_ClearFlag_HSERDY(), LL_RCC_ClearFlag_HSIRDY(), LL_RCC_ClearFlag_LSIRDY(), LL_RCC_ConfigMCO(), LL_RCC_DisableIT_HSERDY(), LL_RCC_DisableIT_LSERDY(), LL_RCC_EnableIT_LSIRDY(), LL_RCC_EnableIT_PLLRDY() (+44 more)

### Community 13 - "HAL Core & Cortex"
Cohesion: 0.06
Nodes (24): HAL_MPU_ConfigRegion(), HAL_NVIC_ClearPendingIRQ(), HAL_NVIC_DisableIRQ(), HAL_NVIC_EnableIRQ(), HAL_NVIC_GetActive(), HAL_NVIC_GetPendingIRQ(), HAL_NVIC_GetPriority(), HAL_NVIC_SetPendingIRQ() (+16 more)

### Community 14 - "Main Init"
Cohesion: 0.06
Nodes (21): Error_Handler(), main(), MX_DMA_Init(), MX_GPIO_Init(), MX_I2C1_Init(), MX_I2C2_Init(), MX_SPI1_Init(), MX_TIM6_Init() (+13 more)

### Community 15 - "BMP280 Driver"
Cohesion: 0.14
Nodes (31): a_bmp280_compensate_pressure(), a_bmp280_compensate_temperature(), a_bmp280_get_nvm_calibration(), a_bmp280_iic_spi_read(), a_bmp280_iic_spi_write(), bmp280_deinit(), bmp280_get_addr_pin(), bmp280_get_filter() (+23 more)

### Community 16 - "System Init & LL Bus"
Cohesion: 0.07
Nodes (20): LL_AHB1_GRP1_DisableClock(), LL_AHB1_GRP1_EnableClock(), LL_AHB1_GRP1_ForceReset(), LL_AHB1_GRP1_IsEnabledClock(), LL_AHB1_GRP1_ReleaseReset(), LL_APB1_GRP1_DisableClock(), LL_APB1_GRP1_EnableClock(), LL_APB1_GRP1_ForceReset() (+12 more)

### Community 17 - "Core ARMv8-M Mainline"
Cohesion: 0.12
Nodes (38): ITM_CheckChar(), ITM_ReceiveChar(), ITM_SendChar(), __NVIC_ClearPendingIRQ(), NVIC_ClearTargetState(), NVIC_DecodePriority(), __NVIC_DisableIRQ(), __NVIC_EnableIRQ() (+30 more)

### Community 18 - "Core CM33"
Cohesion: 0.12
Nodes (38): ITM_CheckChar(), ITM_ReceiveChar(), ITM_SendChar(), __NVIC_ClearPendingIRQ(), NVIC_ClearTargetState(), NVIC_DecodePriority(), __NVIC_DisableIRQ(), __NVIC_EnableIRQ() (+30 more)

### Community 20 - "HAL Flash"
Cohesion: 0.13
Nodes (28): FLASH_MassErase(), FLASH_OB_DisableWRP(), FLASH_OB_EnableWRP(), FLASH_OB_GetRDP(), FLASH_OB_GetUser(), FLASH_OB_GetWRP(), FLASH_OB_ProgramData(), FLASH_OB_RDP_LevelConfig() (+20 more)

### Community 21 - "Core ARMv8-M Baseline"
Cohesion: 0.15
Nodes (31): __NVIC_ClearPendingIRQ(), NVIC_ClearTargetState(), NVIC_DecodePriority(), __NVIC_DisableIRQ(), __NVIC_EnableIRQ(), NVIC_EncodePriority(), __NVIC_GetActive(), __NVIC_GetEnableIRQ() (+23 more)

### Community 22 - "Core CM23"
Cohesion: 0.15
Nodes (31): __NVIC_ClearPendingIRQ(), NVIC_ClearTargetState(), NVIC_DecodePriority(), __NVIC_DisableIRQ(), __NVIC_EnableIRQ(), NVIC_EncodePriority(), __NVIC_GetActive(), __NVIC_GetEnableIRQ() (+23 more)

### Community 23 - "LL EXTI"
Cohesion: 0.11
Nodes (32): LL_EXTI_ClearFlag_0_31(), LL_EXTI_ClearFlag_32_63(), LL_EXTI_DisableEvent_0_31(), LL_EXTI_DisableEvent_32_63(), LL_EXTI_DisableFallingTrig_0_31(), LL_EXTI_DisableFallingTrig_32_63(), LL_EXTI_DisableIT_0_31(), LL_EXTI_DisableIT_32_63() (+24 more)

### Community 24 - "LSM303AGR Driver"
Cohesion: 0.16
Nodes (24): COMPASSACCELERO_IO_Init(), COMPASSACCELERO_IO_ITConfig(), COMPASSACCELERO_IO_Read(), COMPASSACCELERO_IO_Write(), LSM303AGR_AccClickITDisable(), LSM303AGR_AccClickITEnable(), LSM303AGR_AccFilterClickCmd(), LSM303AGR_AccFilterCmd() (+16 more)

### Community 25 - "Spec Fusion & Calibration"
Cohesion: 0.11
Nodes (24): M0 Build skeleton, M3 Calibration, M4 Fusion, M5 Hardening, SPEC — 10-DOF IMU Node (CubeMX+CMake variant), Build acceptance criteria AC-01..AC-10, Axis map (signed permutation, FRD body frame), Alpha-beta baro filter + altitude formula (+16 more)

### Community 26 - "Core CM4 & Device"
Cohesion: 0.17
Nodes (21): ITM_CheckChar(), ITM_ReceiveChar(), ITM_SendChar(), __NVIC_ClearPendingIRQ(), NVIC_DecodePriority(), __NVIC_DisableIRQ(), __NVIC_EnableIRQ(), NVIC_EncodePriority() (+13 more)

### Community 27 - "CMSIS ARMCC"
Cohesion: 0.14
Nodes (21): __get_APSR(), __get_BASEPRI(), __get_CONTROL(), __get_FAULTMASK(), __get_FPSCR(), __get_IPSR(), __get_MSP(), __get_PRIMASK() (+13 more)

### Community 28 - "LL Cortex"
Cohesion: 0.15
Nodes (24): LL_CPUID_GetConstant(), LL_CPUID_GetImplementer(), LL_CPUID_GetParNo(), LL_CPUID_GetRevision(), LL_CPUID_GetVariant(), LL_HANDLER_DisableFault(), LL_HANDLER_EnableFault(), LL_LPM_DisableEventOnPend() (+16 more)

### Community 29 - "LL GPIO"
Cohesion: 0.21
Nodes (23): LL_GPIO_GetAFPin_0_7(), LL_GPIO_GetAFPin_8_15(), LL_GPIO_GetPinMode(), LL_GPIO_GetPinOutputType(), LL_GPIO_GetPinPull(), LL_GPIO_GetPinSpeed(), LL_GPIO_IsAnyPinLocked(), LL_GPIO_IsInputPinSet() (+15 more)

### Community 30 - "LL PWR"
Cohesion: 0.15
Nodes (24): LL_PWR_ClearFlag_SB(), LL_PWR_ClearFlag_WU(), LL_PWR_DisableBkUpAccess(), LL_PWR_DisablePVD(), LL_PWR_DisableSDADC(), LL_PWR_DisableWakeUpPin(), LL_PWR_EnableBkUpAccess(), LL_PWR_EnablePVD() (+16 more)

### Community 31 - "Core CM3"
Cohesion: 0.18
Nodes (21): ITM_CheckChar(), ITM_ReceiveChar(), ITM_SendChar(), __NVIC_ClearPendingIRQ(), NVIC_DecodePriority(), __NVIC_DisableIRQ(), __NVIC_EnableIRQ(), NVIC_EncodePriority() (+13 more)

### Community 32 - "Core SC300"
Cohesion: 0.19
Nodes (21): ITM_CheckChar(), ITM_ReceiveChar(), ITM_SendChar(), __NVIC_ClearPendingIRQ(), NVIC_DecodePriority(), __NVIC_DisableIRQ(), __NVIC_EnableIRQ(), NVIC_EncodePriority() (+13 more)

### Community 34 - "ST Vendor Repo Docs"
Cohesion: 0.13
Nodes (22): BSD-3-Clause License, Common Issue Template, Common Pull Request Template, Common Code of Conduct, Common Contributing Guide, Common License (BSD-3-Clause), Common Release Notes, Common Security Policy (ST PSIRT) (+14 more)

### Community 35 - "I3G4250D Driver"
Cohesion: 0.22
Nodes (14): GYRO_IO_Init(), GYRO_IO_Read(), GYRO_IO_Write(), I3G4250D_DisableIT(), I3G4250D_EnableIT(), I3G4250D_FilterCmd(), I3G4250D_FilterConfig(), I3G4250D_GetDataStatus() (+6 more)

### Community 36 - "Core CM0"
Cohesion: 0.25
Nodes (15): __NVIC_ClearPendingIRQ(), NVIC_DecodePriority(), __NVIC_DisableIRQ(), __NVIC_EnableIRQ(), NVIC_EncodePriority(), __NVIC_GetEnableIRQ(), __NVIC_GetPendingIRQ(), __NVIC_GetPriority() (+7 more)

### Community 37 - "Core CM1"
Cohesion: 0.25
Nodes (15): __NVIC_ClearPendingIRQ(), NVIC_DecodePriority(), __NVIC_DisableIRQ(), __NVIC_EnableIRQ(), NVIC_EncodePriority(), __NVIC_GetEnableIRQ(), __NVIC_GetPendingIRQ(), __NVIC_GetPriority() (+7 more)

### Community 39 - "HAL RCC"
Cohesion: 0.16
Nodes (11): HAL_RCC_ClockConfig(), HAL_RCC_CSSCallback(), HAL_RCC_DeInit(), HAL_RCC_GetClockConfig(), HAL_RCC_GetHCLKFreq(), HAL_RCC_GetOscConfig(), HAL_RCC_GetPCLK1Freq(), HAL_RCC_GetPCLK2Freq() (+3 more)

### Community 41 - "F3Disco BSP Core"
Cohesion: 0.18
Nodes (11): BSP_LED_Init(), BSP_LED_Off(), BSP_LED_On(), BSP_LED_Toggle(), BSP_PB_GetState(), BSP_PB_Init(), I2Cx_Error(), I2Cx_ReadData() (+3 more)

### Community 42 - "Core SC000"
Cohesion: 0.29
Nodes (13): __NVIC_ClearPendingIRQ(), __NVIC_DisableIRQ(), __NVIC_EnableIRQ(), __NVIC_GetEnableIRQ(), __NVIC_GetPendingIRQ(), __NVIC_GetPriority(), __NVIC_GetVector(), __NVIC_SetPendingIRQ() (+5 more)

### Community 43 - "HAL GPIO"
Cohesion: 0.20
Nodes (9): HAL_GPIO_DeInit(), HAL_GPIO_EXTI_Callback(), HAL_GPIO_EXTI_IRQHandler(), HAL_GPIO_Init(), HAL_GPIO_LockPin(), HAL_GPIO_ReadPin(), HAL_GPIO_TogglePin(), HAL_GPIO_WritePin() (+1 more)

### Community 44 - "CMake Target Graph"
Cohesion: 0.27
Nodes (13): CubeMX-generated CMakeLists.txt, MX_Application_Src (main.c, ISRs, startup), STM32_Drivers OBJECT library, stm32cubemx INTERFACE library, Root CMakeLists.txt, Executable 010_9dof_bmp280_cmake, bsp STATIC library, sensors STATIC library (+5 more)

### Community 45 - "Plan Decisions"
Cohesion: 0.14
Nodes (8): OUR_WARNINGS flag set, PLAN — implementing SPEC.md, Current state (baseline d94912b), M2 Axis + identity, Plan-specific risks, Spec updates to make along the way, Open questions Q-1..Q-6, Risks R-11..R-19

### Community 46 - "HAL EXTI"
Cohesion: 0.29
Nodes (9): HAL_EXTI_ClearConfigLine(), HAL_EXTI_ClearPending(), HAL_EXTI_GenerateSWI(), HAL_EXTI_GetConfigLine(), HAL_EXTI_GetHandle(), HAL_EXTI_GetPending(), HAL_EXTI_IRQHandler(), HAL_EXTI_RegisterCallback() (+1 more)

### Community 48 - "CMSIS Compiler"
Cohesion: 0.27
Nodes (5): T_UINT16_READ(), T_UINT16_WRITE(), T_UINT32(), T_UINT32_READ(), T_UINT32_WRITE()

### Community 49 - "HAL PWR Ex"
Cohesion: 0.22
Nodes (3): HAL_PWR_ConfigPVD(), HAL_PWR_PVD_IRQHandler(), HAL_PWR_PVDCallback()

### Community 52 - "HAL I2C Ex"
Cohesion: 0.39
Nodes (4): HAL_I2CEx_ConfigAnalogFilter(), HAL_I2CEx_ConfigDigitalFilter(), HAL_I2CEx_DisableWakeUp(), HAL_I2CEx_EnableWakeUp()

### Community 53 - "HAL RCC Ex"
Cohesion: 0.38
Nodes (4): HAL_RCCEx_GetPeriphCLKConfig(), HAL_RCCEx_GetPeriphCLKFreq(), HAL_RCCEx_PeriphCLKConfig(), RCC_GetPLLCLKFreq()

### Community 55 - "Spec Architecture & Telemetry"
Cohesion: 0.40
Nodes (4): M1 Raw sensors, Part B — Product specification, Telemetry status flags, Telemetry frames (84 B, CRC16-CCITT-FALSE, 460800 baud)

### Community 56 - "Packed Helpers A"
Cohesion: 0.40
Nodes (4): T_UINT16_READ(), T_UINT16_WRITE(), T_UINT32_READ(), T_UINT32_WRITE()

### Community 57 - "Packed Helpers B"
Cohesion: 0.40
Nodes (4): T_UINT16_READ(), T_UINT16_WRITE(), T_UINT32_READ(), T_UINT32_WRITE()

## Ambiguous Edges - Review These
- `sensors STATIC library` → `bsp STATIC library`  [AMBIGUOUS]
  CMakeLists.txt · relation: conceptually_related_to

## Knowledge Gaps
- **17 isolated node(s):** `v`, `CMSIS Device STM32F3xx License`, `CMSIS Apache License 2.0`, `STM32F3xx HAL Driver License`, `Common Issue Template` (+12 more)
  These have ≤1 connection - possible missing edges or undocumented components. (Counts symbols only; 249 node(s) total have ≤1 connection when file, concept and rationale nodes are included.)
- **20 thin communities (<3 nodes) omitted from report** — run `graphify query` to explore isolated nodes.

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **Why does `LL_MPU_Enable()` connect `Core CM0+` to `LL Cortex`?**
  _High betweenness centrality (0.055) - this node is a cross-community bridge._
- **What connects `v`, `CMSIS Device STM32F3xx License`, `CMSIS Apache License 2.0` to the rest of the system?**
  _17 weakly-connected nodes found - possible documentation gaps or missing edges._
- **Should `LL I2C` be split into smaller, more focused modules?**
  _Cohesion score 0.043151567196221555 - nodes in this community are weakly interconnected._
- **Why does `__DSB()` connect `Core CM0+` to `Core SC300`, `CMSIS GCC Intrinsics`, `Core CM0`, `Core CM1`, `Core SC000`, `Core ARMv8-M Mainline`, `Core CM33`, `Core ARMv8-M Baseline`, `Core CM23`, `Core CM4 & Device`, `Core CM3`?**
  _High betweenness centrality (0.029) - this node is a cross-community bridge._
- **Should `CMSIS GCC Intrinsics` be split into smaller, more focused modules?**
  _Cohesion score 0.02962630456738862 - nodes in this community are weakly interconnected._
- **Why does `__ISB()` connect `Core CM0+` to `Core SC300`, `CMSIS GCC Intrinsics`, `Core CM0`, `Core CM1`, `Core SC000`, `Core ARMv8-M Mainline`, `Core CM33`, `Core ARMv8-M Baseline`, `Core CM23`, `Core CM4 & Device`, `Core CM3`?**
  _High betweenness centrality (0.026) - this node is a cross-community bridge._

### Low-confidence Hints
_AMBIGUOUS edges — the extractor was unsure. Verify before acting on these._

- **What is the exact relationship between `sensors STATIC library` and `bsp STATIC library`?**
  _Edge tagged AMBIGUOUS (relation: conceptually_related_to) - confidence is low._