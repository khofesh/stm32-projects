# Graph Report - 010_9dof_bmp280_cmake  (2026-11-13)

## Corpus Check
- 64 files · ~500,599 words
- Verdict: corpus is large enough that graph structure adds value.

## Summary
- 2464 nodes · 5834 edges · 78 communities (63 shown, 15 thin omitted)
- Extraction: 93% EXTRACTED · 7% INFERRED · 0% AMBIGUOUS · INFERRED: 403 edges (avg confidence: 0.85)
- Token cost: 0 input · 0 output

## Community Hubs (Navigation)
- LL I2C Driver
- CMSIS GCC Intrinsics
- LL SPI/I2S Driver
- HAL DMA
- CMSIS ARMClang Intrinsics
- LL DMA Driver
- HAL USB PCD
- CMSIS Cortex-M0+ Core
- HAL SPI
- LL System/DBGMCU
- Axis Map and Debug
- BMP280 Vendor Driver
- CMSIS IAR Intrinsics
- IMU Task Calibration Apply
- LL RCC Flags
- LL RCC Clock Config
- CMSIS ARMv8-M Mainline
- CMSIS Cortex-M33 Core
- HAL Flash
- CMSIS ARMv8-M Baseline
- CMSIS Cortex-M23 Core
- LL EXTI Driver
- Calibration Storage
- HAL Core
- I2C Bus and BMP280
- UART and Timer ISR
- Newlib Syscalls
- LL Cortex Driver
- App Scheduler Loop
- CMSIS ARMCC Intrinsics
- LL GPIO Driver
- LL PWR Driver
- CMSIS SC300 Core
- CMSIS Cortex-M3 Core
- CMSIS Cortex-M4 Core
- Host Tools (log, size)
- Calibration Procedure Docs
- HAL Cortex/MPU
- SPI Bus and Gyro
- Calibration State Machine
- CMake Build Targets
- CMSIS Cortex-M0 Core
- CMSIS Cortex-M1 Core
- HAL RCC
- Sensor Init and Identity
- LL Bus Clocks
- Main Entry and Init
- CMSIS SC000 Core
- Spec Build Requirements
- HAL GPIO
- HAL EXTI
- HAL RCC Extended
- Hardware and Axis Maps
- CMSIS Compiler Abstraction
- HAL PWR Extended
- CubeMX CMake Integration
- HAL I2C Extended
- Plan Decisions and Flash Save
- LL Utils
- Size Budget and Requirements
- CMSIS Packed Access A
- CMSIS Packed Access B
- CubeMX Regeneration
- ST Licenses
- CMSIS Unaligned Types
- CMSIS License

## God Nodes (most connected - your core abstractions)
1. `__DSB()` - 44 edges
2. `SPEC — 10-DOF IMU Node` - 37 edges
3. `HAL_GetTick()` - 35 edges
4. `__ISB()` - 32 edges
5. `I2C_Enable_IRQ()` - 29 edges
6. `test_state_machine()` - 26 edges
7. `I2C_TransferConfig()` - 25 edges
8. `a_bmp280_iic_spi_read()` - 22 edges
9. `HAL_DMA_Start_IT()` - 19 edges
10. `I2C_ITError()` - 19 edges

## Surprising Connections (you probably didn't know these)
- `unit_tests executable` --semantically_similar_to--> `OUR_WARNINGS / OUR_WERROR`  [INFERRED] [semantically similar]
  tests/CMakeLists.txt → CMakeLists.txt
- `D-6 In-repo assert test harness` --rationale_for--> `unit_tests executable`  [INFERRED]
  docs/PLAN.md → tests/CMakeLists.txt
- `sensors_log()` --calls--> `bmp280_chip_id()`  [INFERRED]
  App/Src/app.c → Sensors/Src/bmp280.c
- `sensors_log()` --calls--> `i3g4250d_id()`  [INFERRED]
  App/Src/app.c → Sensors/Src/i3g4250d.c
- `sensors_log()` --calls--> `lsm303agr_accel_id()`  [INFERRED]
  App/Src/app.c → Sensors/Src/lsm303agr.c

## Import Cycles
- None detected.

## Hyperedges (group relationships)
- **Vendor driver license set** — drivers_cmsis_license, drivers_cmsis_device_st_stm32f3xx_license, drivers_stm32f3xx_hal_driver_license [INFERRED 0.85]
- **Layered static library chain bsp->sensors->fusion->app** — cmakelists_bsp, cmakelists_sensors, cmakelists_fusion, cmakelists_app, cmakelists_010_9dof_bmp280_cmake [EXTRACTED 1.00]
- **Calibration capture, storage and reporting flow** — docs_calibration_gyro_bias_procedure, docs_calibration_accel_six_position, docs_calibration_mag_minmax_procedure, docs_calibration_baro_reference, docs_calibration_save_verify, docs_calibration_storage_format_v2, docs_calibration_calib_state_encoding [EXTRACTED 1.00]
- **CubeMX regeneration discipline** — tools_regenerate, docs_spec_headless_regeneration, docs_toolchain_regeneration_check, docs_toolchain_ownership_table, docs_plan_d3_keep_exe_name [INFERRED 0.85]

## Communities (78 total, 15 thin omitted)

### Community 0 - "LL I2C Driver"
Cohesion: 0.04
Nodes (134): LL_I2C_AcknowledgeNextData(), LL_I2C_ClearFlag_ADDR(), LL_I2C_ClearFlag_ARLO(), LL_I2C_ClearFlag_BERR(), LL_I2C_ClearFlag_NACK(), LL_I2C_ClearFlag_OVR(), LL_I2C_ClearFlag_STOP(), LL_I2C_ClearFlag_TXE() (+126 more)

### Community 1 - "CMSIS GCC Intrinsics"
Cohesion: 0.03
Nodes (132): __CLREX(), __disable_fault_irq(), __disable_irq(), __DMB(), __enable_fault_irq(), __enable_irq(), __get_APSR(), __get_BASEPRI() (+124 more)

### Community 2 - "LL SPI/I2S Driver"
Cohesion: 0.05
Nodes (118): LL_I2S_ClearFlag_FRE(), LL_I2S_ClearFlag_OVR(), LL_I2S_ClearFlag_UDR(), LL_I2S_Disable(), LL_I2S_DisableDMAReq_RX(), LL_I2S_DisableDMAReq_TX(), LL_I2S_DisableIT_ERR(), LL_I2S_DisableIT_RXNE() (+110 more)

### Community 3 - "HAL DMA"
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

### Community 7 - "CMSIS Cortex-M0+ Core"
Cohesion: 0.06
Nodes (73): __DSB(), __ISB(), __NVIC_ClearPendingIRQ(), NVIC_DecodePriority(), __NVIC_DisableIRQ(), __NVIC_EnableIRQ(), NVIC_EncodePriority(), __NVIC_GetEnableIRQ() (+65 more)

### Community 8 - "HAL SPI"
Cohesion: 0.09
Nodes (51): HAL_GetTick(), HAL_SPI_Abort(), HAL_SPI_Abort_IT(), HAL_SPI_AbortCpltCallback(), HAL_SPI_DeInit(), HAL_SPI_DMAPause(), HAL_SPI_DMAResume(), HAL_SPI_DMAStop() (+43 more)

### Community 9 - "LL System/DBGMCU"
Cohesion: 0.06
Nodes (64): LL_DBGMCU_APB1_GRP1_FreezePeriph(), LL_DBGMCU_APB1_GRP1_UnFreezePeriph(), LL_DBGMCU_APB2_GRP1_FreezePeriph(), LL_DBGMCU_APB2_GRP1_UnFreezePeriph(), LL_DBGMCU_DisableDBGSleepMode(), LL_DBGMCU_DisableDBGStandbyMode(), LL_DBGMCU_DisableDBGStopMode(), LL_DBGMCU_EnableDBGSleepMode() (+56 more)

### Community 10 - "Axis Map and Debug"
Cohesion: 0.08
Nodes (41): debug_flush(), board_axes_apply_accel(), board_axes_apply_gyro(), board_axes_apply_mag(), crc16_ccitt(), check(), finish(), frame_data_pack() (+33 more)

### Community 11 - "BMP280 Vendor Driver"
Cohesion: 0.09
Nodes (37): bmp280_init(), config(), a_bmp280_compensate_pressure(), a_bmp280_compensate_temperature(), a_bmp280_get_nvm_calibration(), a_bmp280_iic_spi_read(), a_bmp280_iic_spi_write(), bmp280_deinit() (+29 more)

### Community 12 - "CMSIS IAR Intrinsics"
Cohesion: 0.06
Nodes (52): __CLZ(), __get_APSR(), __get_MSPLIM(), __get_PSPLIM(), __iar_u32(), __iar_uint16_read(), __iar_uint16_write(), __iar_uint32_read() (+44 more)

### Community 13 - "IMU Task Calibration Apply"
Cohesion: 0.10
Nodes (43): imu_task(), accel_take_pose(), calibration_apply_accel(), calibration_apply_gyro(), calibration_apply_mag(), calibration_feed_accel(), calibration_feed_gyro(), calibration_feed_mag() (+35 more)

### Community 14 - "LL RCC Flags"
Cohesion: 0.04
Nodes (52): LL_RCC_ClearFlag_HSECSS(), LL_RCC_ClearFlag_LSERDY(), LL_RCC_ClearFlag_PLLRDY(), LL_RCC_ClearResetFlags(), LL_RCC_DisableIT_HSIRDY(), LL_RCC_DisableIT_LSIRDY(), LL_RCC_DisableIT_PLLRDY(), LL_RCC_DisableRTC() (+44 more)

### Community 15 - "LL RCC Clock Config"
Cohesion: 0.04
Nodes (52): LL_RCC_ClearFlag_HSERDY(), LL_RCC_ClearFlag_HSIRDY(), LL_RCC_ClearFlag_LSIRDY(), LL_RCC_ConfigMCO(), LL_RCC_DisableIT_HSERDY(), LL_RCC_DisableIT_LSERDY(), LL_RCC_EnableIT_LSIRDY(), LL_RCC_EnableIT_PLLRDY() (+44 more)

### Community 16 - "CMSIS ARMv8-M Mainline"
Cohesion: 0.12
Nodes (38): ITM_CheckChar(), ITM_ReceiveChar(), ITM_SendChar(), __NVIC_ClearPendingIRQ(), NVIC_ClearTargetState(), NVIC_DecodePriority(), __NVIC_DisableIRQ(), __NVIC_EnableIRQ() (+30 more)

### Community 17 - "CMSIS Cortex-M33 Core"
Cohesion: 0.12
Nodes (38): ITM_CheckChar(), ITM_ReceiveChar(), ITM_SendChar(), __NVIC_ClearPendingIRQ(), NVIC_ClearTargetState(), NVIC_DecodePriority(), __NVIC_DisableIRQ(), __NVIC_EnableIRQ() (+30 more)

### Community 19 - "HAL Flash"
Cohesion: 0.13
Nodes (28): FLASH_MassErase(), FLASH_OB_DisableWRP(), FLASH_OB_EnableWRP(), FLASH_OB_GetRDP(), FLASH_OB_GetUser(), FLASH_OB_GetWRP(), FLASH_OB_ProgramData(), FLASH_OB_RDP_LevelConfig() (+20 more)

### Community 20 - "CMSIS ARMv8-M Baseline"
Cohesion: 0.15
Nodes (31): __NVIC_ClearPendingIRQ(), NVIC_ClearTargetState(), NVIC_DecodePriority(), __NVIC_DisableIRQ(), __NVIC_EnableIRQ(), NVIC_EncodePriority(), __NVIC_GetActive(), __NVIC_GetEnableIRQ() (+23 more)

### Community 21 - "CMSIS Cortex-M23 Core"
Cohesion: 0.15
Nodes (31): __NVIC_ClearPendingIRQ(), NVIC_ClearTargetState(), NVIC_DecodePriority(), __NVIC_DisableIRQ(), __NVIC_EnableIRQ(), NVIC_EncodePriority(), __NVIC_GetActive(), __NVIC_GetEnableIRQ() (+23 more)

### Community 22 - "LL EXTI Driver"
Cohesion: 0.11
Nodes (32): LL_EXTI_ClearFlag_0_31(), LL_EXTI_ClearFlag_32_63(), LL_EXTI_DisableEvent_0_31(), LL_EXTI_DisableEvent_32_63(), LL_EXTI_DisableFallingTrig_0_31(), LL_EXTI_DisableFallingTrig_32_63(), LL_EXTI_DisableIT_0_31(), LL_EXTI_DisableIT_32_63() (+24 more)

### Community 23 - "Calibration Storage"
Cohesion: 0.12
Nodes (20): cal_store_decode(), cal_store_encode(), cal_store_load(), cal_store_save(), get_f32(), get_u16(), get_u32(), put_f32() (+12 more)

### Community 24 - "HAL Core"
Cohesion: 0.09
Nodes (14): HAL_NVIC_SetPriority(), HAL_NVIC_SetPriorityGrouping(), HAL_SYSTICK_Config(), HAL_DeInit(), HAL_Delay(), HAL_GetTickFreq(), HAL_IncTick(), HAL_Init() (+6 more)

### Community 25 - "I2C Bus and BMP280"
Cohesion: 0.11
Nodes (14): bsp_i2c_errors(), bsp_i2c_read(), bsp_i2c_recover(), bsp_i2c_write(), bus_clear(), bus_fail(), bus_get(), recover() (+6 more)

### Community 26 - "UART and Timer ISR"
Cohesion: 0.10
Nodes (15): HAL_TIM_PeriodElapsedCallback(), bsp_uart_init(), HAL_UART_ErrorCallback(), HAL_UART_RxCpltCallback(), rx_arm(), HAL_I2C_MspDeInit(), HAL_I2C_MspInit(), HAL_PCD_MspDeInit() (+7 more)

### Community 27 - "Newlib Syscalls"
Cohesion: 0.08
Nodes (5): _exit(), _kill(), starm_getc(), starm_putc(), _times()

### Community 28 - "LL Cortex Driver"
Cohesion: 0.14
Nodes (25): LL_CPUID_GetConstant(), LL_CPUID_GetImplementer(), LL_CPUID_GetParNo(), LL_CPUID_GetRevision(), LL_CPUID_GetVariant(), LL_HANDLER_DisableFault(), LL_HANDLER_EnableFault(), LL_LPM_DisableEventOnPend() (+17 more)

### Community 29 - "App Scheduler Loop"
Cohesion: 0.12
Nodes (16): app_run(), baro_task(), debug_task(), overrun_note(), status_set(), calibration_feed_baro(), calibration_save(), calibration_take_save_request() (+8 more)

### Community 30 - "CMSIS ARMCC Intrinsics"
Cohesion: 0.14
Nodes (21): __get_APSR(), __get_BASEPRI(), __get_CONTROL(), __get_FAULTMASK(), __get_FPSCR(), __get_IPSR(), __get_MSP(), __get_PRIMASK() (+13 more)

### Community 31 - "LL GPIO Driver"
Cohesion: 0.21
Nodes (23): LL_GPIO_GetAFPin_0_7(), LL_GPIO_GetAFPin_8_15(), LL_GPIO_GetPinMode(), LL_GPIO_GetPinOutputType(), LL_GPIO_GetPinPull(), LL_GPIO_GetPinSpeed(), LL_GPIO_IsAnyPinLocked(), LL_GPIO_IsInputPinSet() (+15 more)

### Community 32 - "LL PWR Driver"
Cohesion: 0.15
Nodes (24): LL_PWR_ClearFlag_SB(), LL_PWR_ClearFlag_WU(), LL_PWR_DisableBkUpAccess(), LL_PWR_DisablePVD(), LL_PWR_DisableSDADC(), LL_PWR_DisableWakeUpPin(), LL_PWR_EnableBkUpAccess(), LL_PWR_EnablePVD() (+16 more)

### Community 33 - "CMSIS SC300 Core"
Cohesion: 0.18
Nodes (21): ITM_CheckChar(), ITM_ReceiveChar(), ITM_SendChar(), __NVIC_ClearPendingIRQ(), NVIC_DecodePriority(), __NVIC_DisableIRQ(), __NVIC_EnableIRQ(), NVIC_EncodePriority() (+13 more)

### Community 34 - "CMSIS Cortex-M3 Core"
Cohesion: 0.19
Nodes (21): ITM_CheckChar(), ITM_ReceiveChar(), ITM_SendChar(), __NVIC_ClearPendingIRQ(), NVIC_DecodePriority(), __NVIC_DisableIRQ(), __NVIC_EnableIRQ(), NVIC_EncodePriority() (+13 more)

### Community 35 - "CMSIS Cortex-M4 Core"
Cohesion: 0.19
Nodes (21): ITM_CheckChar(), ITM_ReceiveChar(), ITM_SendChar(), __NVIC_ClearPendingIRQ(), NVIC_DecodePriority(), __NVIC_DisableIRQ(), __NVIC_EnableIRQ(), NVIC_EncodePriority() (+13 more)

### Community 37 - "Host Tools (log, size)"
Cohesion: 0.16
Nodes (9): crc16_ccitt(), frames(), main(), open_stream(), Stats, main(), parse_size_output(), report() (+1 more)

### Community 38 - "Calibration Procedure Docs"
Cohesion: 0.12
Nodes (22): Calibration procedure (T-04/T-15), Accelerometer six-position calibration, Calibration acceptance checklist, Barometer reference p0 (B), calib_state byte encoding, Gyro bias procedure (G, SEM < 0.05 °/s), Magnetometer min/max hard/soft-iron calibration, Calibration storage format v2 (magic ALC1) (+14 more)

### Community 40 - "HAL Cortex/MPU"
Cohesion: 0.13
Nodes (10): HAL_MPU_ConfigRegion(), HAL_NVIC_ClearPendingIRQ(), HAL_NVIC_DisableIRQ(), HAL_NVIC_EnableIRQ(), HAL_NVIC_GetActive(), HAL_NVIC_GetPendingIRQ(), HAL_NVIC_GetPriority(), HAL_NVIC_SetPendingIRQ() (+2 more)

### Community 41 - "SPI Bus and Gyro"
Cohesion: 0.15
Nodes (7): bsp_spi_read(), bsp_spi_write(), spi_fail(), i3g4250d_init(), i3g4250d_read(), le16(), write_reg()

### Community 42 - "Calibration State Machine"
Cohesion: 0.22
Nodes (16): telemetry_task(), calibration_active(), calibration_init(), calibration_p0_pa(), calibration_request_save(), calibration_reset_defaults(), calibration_start_accel(), calibration_start_baro() (+8 more)

### Community 43 - "CMake Build Targets"
Cohesion: 0.24
Nodes (17): CMakeLists.txt (root), add_our_library(), app static library, bmp280_init=ld_bmp280_init define, bsp static library, build_info custom target, fusion static library, OUR_WARNINGS / OUR_WERROR (+9 more)

### Community 44 - "CMSIS Cortex-M0 Core"
Cohesion: 0.25
Nodes (15): __NVIC_ClearPendingIRQ(), NVIC_DecodePriority(), __NVIC_DisableIRQ(), __NVIC_EnableIRQ(), NVIC_EncodePriority(), __NVIC_GetEnableIRQ(), __NVIC_GetPendingIRQ(), __NVIC_GetPriority() (+7 more)

### Community 45 - "CMSIS Cortex-M1 Core"
Cohesion: 0.25
Nodes (15): __NVIC_ClearPendingIRQ(), NVIC_DecodePriority(), __NVIC_DisableIRQ(), __NVIC_EnableIRQ(), NVIC_EncodePriority(), __NVIC_GetEnableIRQ(), __NVIC_GetPendingIRQ(), __NVIC_GetPriority() (+7 more)

### Community 46 - "HAL RCC"
Cohesion: 0.16
Nodes (11): HAL_RCC_ClockConfig(), HAL_RCC_CSSCallback(), HAL_RCC_DeInit(), HAL_RCC_GetClockConfig(), HAL_RCC_GetHCLKFreq(), HAL_RCC_GetOscConfig(), HAL_RCC_GetPCLK1Freq(), HAL_RCC_GetPCLK2Freq() (+3 more)

### Community 47 - "Sensor Init and Identity"
Cohesion: 0.21
Nodes (14): sensors_init(), sensors_log(), bmp280_chip_id(), i3g4250d_id(), le16(), lsm303agr_accel_id(), lsm303agr_accel_init(), lsm303agr_init() (+6 more)

### Community 50 - "LL Bus Clocks"
Cohesion: 0.22
Nodes (15): LL_AHB1_GRP1_DisableClock(), LL_AHB1_GRP1_EnableClock(), LL_AHB1_GRP1_ForceReset(), LL_AHB1_GRP1_IsEnabledClock(), LL_AHB1_GRP1_ReleaseReset(), LL_APB1_GRP1_DisableClock(), LL_APB1_GRP1_EnableClock(), LL_APB1_GRP1_ForceReset() (+7 more)

### Community 51 - "Main Entry and Init"
Cohesion: 0.27
Nodes (14): app_init(), sched_start(), bsp_time_init(), Error_Handler(), main(), MX_DMA_Init(), MX_GPIO_Init(), MX_I2C1_Init() (+6 more)

### Community 52 - "CMSIS SC000 Core"
Cohesion: 0.29
Nodes (13): __NVIC_ClearPendingIRQ(), __NVIC_DisableIRQ(), __NVIC_EnableIRQ(), __NVIC_GetEnableIRQ(), __NVIC_GetPendingIRQ(), __NVIC_GetPriority(), __NVIC_GetVector(), __NVIC_SetPendingIRQ() (+5 more)

### Community 53 - "Spec Build Requirements"
Cohesion: 0.23
Nodes (13): SPEC — 10-DOF IMU Node, Build acceptance criteria AC-01…AC-10, CI pipeline (§14, TR-09…TR-14), CMake architecture (§9), Migration from CubeIDE baseline (§17), STM32CubeMX project generation (§7), Host-side unit testing (§12), Milestones M0…M5 (+5 more)

### Community 54 - "HAL GPIO"
Cohesion: 0.20
Nodes (9): HAL_GPIO_DeInit(), HAL_GPIO_EXTI_Callback(), HAL_GPIO_EXTI_IRQHandler(), HAL_GPIO_Init(), HAL_GPIO_LockPin(), HAL_GPIO_ReadPin(), HAL_GPIO_TogglePin(), HAL_GPIO_WritePin() (+1 more)

### Community 55 - "HAL EXTI"
Cohesion: 0.29
Nodes (9): HAL_EXTI_ClearConfigLine(), HAL_EXTI_ClearPending(), HAL_EXTI_GenerateSWI(), HAL_EXTI_GetConfigLine(), HAL_EXTI_GetHandle(), HAL_EXTI_GetPending(), HAL_EXTI_IRQHandler(), HAL_EXTI_RegisterCallback() (+1 more)

### Community 56 - "HAL RCC Extended"
Cohesion: 0.18
Nodes (5): HAL_RCCEx_GetPeriphCLKConfig(), HAL_RCCEx_GetPeriphCLKFreq(), HAL_RCCEx_PeriphCLKConfig(), RCC_GetPLLCLKFreq(), HAL_SPIEx_FlushRxFifo()

### Community 58 - "Hardware and Axis Maps"
Cohesion: 0.21
Nodes (11): Hardware (board, wiring, axis maps), Measured accel/gyro/mag matrices, STM32F3DISCOVERY MB1035-F303C-E02, FRD body frame + operator grip, T-03 result (2026-11-13), M4 Fusion milestone, Signed-permutation axis map, Alpha-beta baro filter + altitude formula (+3 more)

### Community 59 - "CMSIS Compiler Abstraction"
Cohesion: 0.27
Nodes (5): T_UINT16_READ(), T_UINT16_WRITE(), T_UINT32(), T_UINT32_READ(), T_UINT32_WRITE()

### Community 60 - "HAL PWR Extended"
Cohesion: 0.22
Nodes (3): HAL_PWR_ConfigPVD(), HAL_PWR_PVD_IRQHandler(), HAL_PWR_PVDCallback()

### Community 61 - "CubeMX CMake Integration"
Cohesion: 0.28
Nodes (7): CubeMX-generated CMakeLists.txt, MX_Application_Src (main.c, ISRs, startup), STM32_Drivers OBJECT library, stm32cubemx INTERFACE library, 010_9dof_bmp280_cmake executable, flash custom target (OpenOCD), Open questions Q-1…Q-6

### Community 62 - "HAL I2C Extended"
Cohesion: 0.39
Nodes (4): HAL_I2CEx_ConfigAnalogFilter(), HAL_I2CEx_ConfigDigitalFilter(), HAL_I2CEx_DisableWakeUp(), HAL_I2CEx_EnableWakeUp()

### Community 63 - "Plan Decisions and Flash Save"
Cohesion: 0.29
Nodes (5): Save and verify (S, flash-stall compensation), PLAN — implementing SPEC, Flash erase stalls scheduler (~40 ms), M5 Hardening milestone, Fault handling (§21.5)

### Community 64 - "LL Utils"
Cohesion: 0.48
Nodes (5): LL_GetFlashSize(), LL_GetUID_Word0(), LL_GetUID_Word1(), LL_GetUID_Word2(), LL_InitTick()

### Community 65 - "Size Budget and Requirements"
Cohesion: 0.40
Nodes (5): post-build .hex/.bin + size check, FLASH_BUDGET / RAM_BUDGET, Adafruit BMP280 wiring on I2C2 @0x77, Product requirements NFR-01…NFR-15, Sensor configuration (§21.2)

### Community 66 - "CMSIS Packed Access A"
Cohesion: 0.40
Nodes (4): T_UINT16_READ(), T_UINT16_WRITE(), T_UINT32_READ(), T_UINT32_WRITE()

### Community 67 - "CMSIS Packed Access B"
Cohesion: 0.40
Nodes (4): T_UINT16_READ(), T_UINT16_WRITE(), T_UINT32_READ(), T_UINT32_WRITE()

### Community 68 - "CubeMX Regeneration"
Cohesion: 1.00
Nodes (3): Headless CubeMX regeneration (§7.4), Regeneration diff check, tools/regenerate.txt (headless CubeMX script)

## Knowledge Gaps
- **10 isolated node(s):** `v`, `CMSIS Device STM32F3xx License`, `STM32F3xx HAL Driver License`, `CMSIS Apache License 2.0`, `bmp280_init=ld_bmp280_init define` (+5 more)
  These have ≤1 connection - possible missing edges or undocumented components. (Counts symbols only; 255 node(s) total have ≤1 connection when file, concept and rationale nodes are included.)
- **15 thin communities (<3 nodes) omitted from report** — run `graphify query` to explore isolated nodes.

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **Why does `LL_MPU_Enable()` connect `LL Cortex Driver` to `CMSIS Cortex-M0+ Core`?**
  _High betweenness centrality (0.042) - this node is a cross-community bridge._
- **What connects `v`, `CMSIS Device STM32F3xx License`, `STM32F3xx HAL Driver License` to the rest of the system?**
  _10 weakly-connected nodes found - possible documentation gaps or missing edges._
- **Should `LL I2C Driver` be split into smaller, more focused modules?**
  _Cohesion score 0.043151567196221555 - nodes in this community are weakly interconnected._
- **Why does `__DSB()` connect `CMSIS Cortex-M0+ Core` to `CMSIS GCC Intrinsics`, `CMSIS Cortex-M3 Core`, `CMSIS Cortex-M4 Core`, `CMSIS SC300 Core`, `CMSIS Cortex-M0 Core`, `CMSIS Cortex-M1 Core`, `CMSIS ARMv8-M Mainline`, `CMSIS Cortex-M33 Core`, `CMSIS ARMv8-M Baseline`, `CMSIS Cortex-M23 Core`, `CMSIS SC000 Core`, `LL Cortex Driver`?**
  _High betweenness centrality (0.025) - this node is a cross-community bridge._
- **Should `CMSIS GCC Intrinsics` be split into smaller, more focused modules?**
  _Cohesion score 0.02962630456738862 - nodes in this community are weakly interconnected._
- **Why does `__ISB()` connect `CMSIS Cortex-M0+ Core` to `CMSIS GCC Intrinsics`, `CMSIS Cortex-M3 Core`, `CMSIS Cortex-M4 Core`, `CMSIS SC300 Core`, `CMSIS Cortex-M0 Core`, `CMSIS Cortex-M1 Core`, `CMSIS ARMv8-M Mainline`, `CMSIS Cortex-M33 Core`, `CMSIS ARMv8-M Baseline`, `CMSIS Cortex-M23 Core`, `CMSIS SC000 Core`, `LL Cortex Driver`?**
  _High betweenness centrality (0.023) - this node is a cross-community bridge._
- **Should `LL SPI/I2S Driver` be split into smaller, more focused modules?**
  _Cohesion score 0.05206611570247934 - nodes in this community are weakly interconnected._