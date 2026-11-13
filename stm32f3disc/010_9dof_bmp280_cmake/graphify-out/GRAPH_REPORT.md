# Graph Report - 010_9dof_bmp280_cmake  (2026-11-13)

## Corpus Check
- Large corpus: 181 files · ~625,750 words. Semantic extraction will be expensive (many Claude tokens). Consider running on a subfolder.

## Summary
- 3264 nodes · 8266 edges · 97 communities (80 shown, 17 thin omitted)
- Extraction: 93% EXTRACTED · 7% INFERRED · 0% AMBIGUOUS · INFERRED: 603 edges (avg confidence: 0.85)
- Token cost: 65,667 input · 0 output

## Community Hubs (Navigation)
- LL USART Driver
- LL I2C Driver
- CMSIS GCC Intrinsics
- LL Timer Driver
- LL SPI/I2S Driver
- CMSIS ARMClang Intrinsics
- LL DMA Driver
- HAL USB PCD
- HAL I2C Driver
- Sensor Drivers + SPI Bus
- App Headers
- HAL Timer Core
- LL Timer Channels
- LL System/DBGMCU
- LibDriver BMP280
- CMSIS IAR Intrinsics
- Madgwick Fusion + Tools
- HAL Timer Extended
- LL RCC Flags
- LL RCC Clock Ready
- HAL EXTI Driver
- Core Barriers + Reset
- Calibration Math + Tests
- BMP280 Wrapper + I2C Bus
- HAL UART Driver
- Core ARMv8-M Mainline
- Core Cortex-M33
- App Scheduler Loop
- Telemetry Framing + CRC
- LL EXTI Driver
- Calibration State Machine
- HAL Tick + Delay
- Core ARMv8-M Baseline
- Flash Storage Path
- SPEC + Calibration Docs
- Newlib Syscalls
- LL Cortex Driver
- CMSIS ARMCC Intrinsics
- LL GPIO Driver
- LL PWR Driver
- Device Headers
- HAL RCC Driver
- HAL MSP Init
- Main + Error Handler
- Core Cortex-M3
- HAL Timer Callbacks
- ARMv8 MPU
- Build + Analysis Targets
- HAL DMA Driver
- Altitude + Baro Filter
- Board Axis Mapping
- HAL PWR Driver
- HAL Cortex/NVIC
- HAL UART Callbacks
- HAL Timer Channel Config
- Core Cortex-M0
- Core Cortex-M0+
- Core Cortex-M1
- LL Bus Clocks
- Interrupt Handlers
- HAL Timer Init
- HAL UART Extended
- Core SC000
- Calibration Store Codec
- Firmware Library Stack
- HAL Timer Sync Config
- HAL GPIO Driver
- Spec Build Rules + Budget
- CMSIS Compiler Abstraction
- Calibration Procedure Docs
- ARMv7 MPU
- HAL PWR Extended
- HAL UART Modes Init
- CubeMX CMake Targets
- UART DMA IRQ Path
- CubeMX Regeneration
- Sensor Frames + Axes
- LL Utils
- Plan + Fusion Tuning
- ARMClang Packed Access
- GCC Packed Access
- Repro Check Script
- BMP280 Wiring + M1
- Flash Stall Risk
- Vendor Licenses
- GCC Packed Types
- SVD Fetch Script
- Latency Probe
- Serial Console
- CMSIS License

## God Nodes (most connected - your core abstractions)
1. `HAL_GetTick()` - 45 edges
2. `__DSB()` - 44 edges
3. `TIM_CCxChannelCmd()` - 40 edges
4. `SPEC — 10-DOF IMU Node` - 36 edges
5. `__ISB()` - 32 edges
6. `HAL_DMA_Start_IT()` - 31 edges
7. `I2C_Enable_IRQ()` - 29 edges
8. `test_state_machine()` - 27 edges
9. `I2C_TransferConfig()` - 25 edges
10. `HAL_DMA_Abort_IT()` - 24 edges

## Surprising Connections (you probably didn't know these)
- `app_init()` --calls--> `bsp_time_init()`  [INFERRED]
  App/Src/app.c → Bsp/Src/bsp_time.c
- `debug_task()` --calls--> `bsp_spi_errors()`  [INFERRED]
  App/Src/app.c → Bsp/Src/bsp_spi.c
- `cal_store_load()` --calls--> `bsp_flash_read()`  [INFERRED]
  App/Src/cal_store.c → Bsp/Src/bsp_flash.c
- `SPI1_IRQHandler()` --calls--> `HAL_SPI_IRQHandler()`  [INFERRED]
  Core/Src/stm32f3xx_it.c → Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_spi.c
- `TIM6_DAC_IRQHandler()` --calls--> `HAL_TIM_IRQHandler()`  [INFERRED]
  Core/Src/stm32f3xx_it.c → Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_tim.c

## Import Cycles
- None detected.

## Hyperedges (group relationships)
- **Vendor driver license set** — drivers_cmsis_license, drivers_cmsis_device_st_stm32f3xx_license, drivers_stm32f3xx_hal_driver_license [INFERRED 0.85]
- **Calibration capture, storage and reporting flow** — docs_calibration_gyro_bias_procedure, docs_calibration_accel_six_position, docs_calibration_mag_minmax_procedure, docs_calibration_baro_reference, docs_calibration_save_verify, docs_calibration_storage_format_v2, docs_calibration_calib_state_encoding [EXTRACTED 1.00]
- **Sensor-to-body axis mapping (T-03 measurement to board matrices)** — docs_hardware_t03_procedure, docs_hardware_signed_permutation_matrix, docs_hardware_frd_body_frame, docs_hardware_mag_heading_difference, docs_plan_d7_board_axes [INFERRED 0.85]
- **Firmware static library layering bsp<-sensors<-fusion<-app->exe** — cmakelists_bsp, cmakelists_sensors, cmakelists_fusion, cmakelists_app, cmakelists_010_9dof_bmp280_cmake, cmakelists_stm32cubemx [EXTRACTED 1.00]
- **Static analysis gates (format-check, cppcheck, analyze)** — cmakelists_format_check, cmakelists_cppcheck, cmakelists_analyze, docs_toolchain_ci_workflow [EXTRACTED 1.00]
- **Release quality flow** — docs_toolchain_release_checklist, docs_toolchain_presets, tests_cmakelists_unit_tests, docs_toolchain_regeneration, docs_toolchain_reproducibility, cmakelists_size_budget [INFERRED 0.85]

## Communities (97 total, 17 thin omitted)

### Community 0 - "LL USART Driver"
Cohesion: 0.03
Nodes (209): LL_USART_ClearFlag_CM(), LL_USART_ClearFlag_EOB(), LL_USART_ClearFlag_FE(), LL_USART_ClearFlag_IDLE(), LL_USART_ClearFlag_LBD(), LL_USART_ClearFlag_nCTS(), LL_USART_ClearFlag_NE(), LL_USART_ClearFlag_ORE() (+201 more)

### Community 1 - "LL I2C Driver"
Cohesion: 0.04
Nodes (134): LL_I2C_AcknowledgeNextData(), LL_I2C_ClearFlag_ADDR(), LL_I2C_ClearFlag_ARLO(), LL_I2C_ClearFlag_BERR(), LL_I2C_ClearFlag_NACK(), LL_I2C_ClearFlag_OVR(), LL_I2C_ClearFlag_STOP(), LL_I2C_ClearFlag_TXE() (+126 more)

### Community 2 - "CMSIS GCC Intrinsics"
Cohesion: 0.03
Nodes (132): __CLREX(), __disable_fault_irq(), __disable_irq(), __DMB(), __enable_fault_irq(), __enable_irq(), __get_APSR(), __get_BASEPRI() (+124 more)

### Community 3 - "LL Timer Driver"
Cohesion: 0.03
Nodes (132): LL_TIM_CC_EnablePreload(), LL_TIM_CC_GetDMAReqTrigger(), LL_TIM_CC_IsEnabledPreload(), LL_TIM_CC_SetDMAReqTrigger(), LL_TIM_CC_SetLockLevel(), LL_TIM_CC_SetUpdate(), LL_TIM_ClearFlag_CC1(), LL_TIM_ClearFlag_CC1OVR() (+124 more)

### Community 4 - "LL SPI/I2S Driver"
Cohesion: 0.05
Nodes (118): LL_I2S_ClearFlag_FRE(), LL_I2S_ClearFlag_OVR(), LL_I2S_ClearFlag_UDR(), LL_I2S_Disable(), LL_I2S_DisableDMAReq_RX(), LL_I2S_DisableDMAReq_TX(), LL_I2S_DisableIT_ERR(), LL_I2S_DisableIT_RXNE() (+110 more)

### Community 5 - "CMSIS ARMClang Intrinsics"
Cohesion: 0.04
Nodes (108): __get_APSR(), __get_BASEPRI(), __get_CONTROL(), __get_FAULTMASK(), __get_IPSR(), __get_MSP(), __get_MSPLIM(), __get_PRIMASK() (+100 more)

### Community 6 - "LL DMA Driver"
Cohesion: 0.06
Nodes (94): LL_DMA_ClearFlag_GI1(), LL_DMA_ClearFlag_GI2(), LL_DMA_ClearFlag_GI3(), LL_DMA_ClearFlag_GI4(), LL_DMA_ClearFlag_GI5(), LL_DMA_ClearFlag_GI6(), LL_DMA_ClearFlag_GI7(), LL_DMA_ClearFlag_HT1() (+86 more)

### Community 7 - "HAL USB PCD"
Cohesion: 0.06
Nodes (73): HAL_PCDEx_BCD_Callback(), HAL_PCDEx_LPM_Callback(), HAL_PCDEx_PMAConfig(), HAL_PCDEx_SetConnectionState(), HAL_PCD_ActivateRemoteWakeup(), HAL_PCD_ConnectCallback(), HAL_PCD_DataInStageCallback(), HAL_PCD_DataOutStageCallback() (+65 more)

### Community 8 - "HAL I2C Driver"
Cohesion: 0.09
Nodes (85): HAL_DMA_Start_IT(), HAL_I2C_AbortCpltCallback(), HAL_I2C_AddrCallback(), HAL_I2C_DeInit(), HAL_I2C_DisableListen_IT(), HAL_I2C_EnableListen_IT(), HAL_I2C_ER_IRQHandler(), HAL_I2C_ErrorCallback() (+77 more)

### Community 9 - "Sensor Drivers + SPI Bus"
Cohesion: 0.07
Nodes (60): sensors_init(), bsp_spi_errors(), bsp_spi_read(), bsp_spi_write(), spi_fail(), HAL_GetTick(), HAL_SPI_Abort(), HAL_SPI_Abort_IT() (+52 more)

### Community 10 - "App Headers"
Cohesion: 0.05
Nodes (50): __NVIC_ClearPendingIRQ(), NVIC_ClearTargetState(), NVIC_DecodePriority(), __NVIC_DisableIRQ(), __NVIC_EnableIRQ(), NVIC_EncodePriority(), __NVIC_GetActive(), __NVIC_GetEnableIRQ() (+42 more)

### Community 11 - "HAL Timer Core"
Cohesion: 0.08
Nodes (68): HAL_DMA_Abort_IT(), HAL_TIM_Base_DeInit(), HAL_TIM_Base_GetState(), HAL_TIM_Base_MspDeInit(), HAL_TIM_Base_Start(), HAL_TIM_Base_Start_DMA(), HAL_TIM_Base_Start_IT(), HAL_TIM_Base_Stop() (+60 more)

### Community 12 - "LL Timer Channels"
Cohesion: 0.03
Nodes (66): LL_TIM_CC_DisableChannel(), LL_TIM_CC_DisablePreload(), LL_TIM_CC_EnableChannel(), LL_TIM_CC_IsEnabledChannel(), LL_TIM_ClearFlag_BRK(), LL_TIM_ClearFlag_BRK2(), LL_TIM_ClearFlag_CC2OVR(), LL_TIM_ClearFlag_CC3OVR() (+58 more)

### Community 13 - "LL System/DBGMCU"
Cohesion: 0.06
Nodes (64): LL_DBGMCU_APB1_GRP1_FreezePeriph(), LL_DBGMCU_APB1_GRP1_UnFreezePeriph(), LL_DBGMCU_APB2_GRP1_FreezePeriph(), LL_DBGMCU_APB2_GRP1_UnFreezePeriph(), LL_DBGMCU_DisableDBGSleepMode(), LL_DBGMCU_DisableDBGStandbyMode(), LL_DBGMCU_DisableDBGStopMode(), LL_DBGMCU_EnableDBGSleepMode() (+56 more)

### Community 14 - "LibDriver BMP280"
Cohesion: 0.09
Nodes (37): bmp280_init(), config(), a_bmp280_compensate_pressure(), a_bmp280_compensate_temperature(), a_bmp280_get_nvm_calibration(), a_bmp280_iic_spi_read(), a_bmp280_iic_spi_write(), bmp280_deinit() (+29 more)

### Community 15 - "CMSIS IAR Intrinsics"
Cohesion: 0.06
Nodes (52): __CLZ(), __get_APSR(), __get_MSPLIM(), __get_PSPLIM(), __iar_u32(), __iar_uint16_read(), __iar_uint16_write(), __iar_uint32_read() (+44 more)

### Community 16 - "Madgwick Fusion + Tools"
Cohesion: 0.09
Nodes (34): inv_norm(), madgwick_get_quat(), madgwick_init(), madgwick_set_beta(), madgwick_update(), madgwick_update_dt(), quat_from_euler(), quat_norm() (+26 more)

### Community 17 - "HAL Timer Extended"
Cohesion: 0.10
Nodes (44): HAL_TIMEx_Break2Callback(), HAL_TIMEx_BreakCallback(), HAL_TIMEx_CommutCallback(), HAL_TIMEx_CommutHalfCpltCallback(), HAL_TIMEx_ConfigBreakDeadTime(), HAL_TIMEx_ConfigCommutEvent(), HAL_TIMEx_ConfigCommutEvent_DMA(), HAL_TIMEx_ConfigCommutEvent_IT() (+36 more)

### Community 18 - "LL RCC Flags"
Cohesion: 0.04
Nodes (52): LL_RCC_ClearFlag_HSECSS(), LL_RCC_ClearFlag_LSERDY(), LL_RCC_ClearFlag_PLLRDY(), LL_RCC_ClearResetFlags(), LL_RCC_DisableIT_HSIRDY(), LL_RCC_DisableIT_LSIRDY(), LL_RCC_DisableIT_PLLRDY(), LL_RCC_DisableRTC() (+44 more)

### Community 19 - "LL RCC Clock Ready"
Cohesion: 0.04
Nodes (52): LL_RCC_ClearFlag_HSERDY(), LL_RCC_ClearFlag_HSIRDY(), LL_RCC_ClearFlag_LSIRDY(), LL_RCC_ConfigMCO(), LL_RCC_DisableIT_HSERDY(), LL_RCC_DisableIT_LSERDY(), LL_RCC_EnableIT_LSIRDY(), LL_RCC_EnableIT_PLLRDY() (+44 more)

### Community 20 - "HAL EXTI Driver"
Cohesion: 0.09
Nodes (37): HAL_EXTI_ClearConfigLine(), HAL_EXTI_ClearPending(), HAL_EXTI_GenerateSWI(), HAL_EXTI_GetConfigLine(), HAL_EXTI_GetHandle(), HAL_EXTI_GetPending(), HAL_EXTI_IRQHandler(), HAL_EXTI_RegisterCallback() (+29 more)

### Community 21 - "Core Barriers + Reset"
Cohesion: 0.09
Nodes (45): __DSB(), __ISB(), __NVIC_SystemReset(), __NVIC_SystemReset(), __NVIC_SystemReset(), __NVIC_SystemReset(), __NVIC_SystemReset(), __NVIC_SystemReset() (+37 more)

### Community 22 - "Calibration Math + Tests"
Cohesion: 0.11
Nodes (35): accel_take_pose(), calibration_feed_accel(), accum_add_axis(), calib_accel_orientation(), calib_accel_solve(), calib_accum_add(), calib_accum_mean(), calib_accum_reset() (+27 more)

### Community 23 - "BMP280 Wrapper + I2C Bus"
Cohesion: 0.08
Nodes (26): sensors_log(), bsp_i2c_errors(), bsp_i2c_read(), bsp_i2c_recover(), bsp_i2c_write(), bus_clear(), bus_fail(), bus_get() (+18 more)

### Community 24 - "HAL UART Driver"
Cohesion: 0.12
Nodes (35): HAL_HalfDuplex_EnableReceiver(), HAL_HalfDuplex_EnableTransmitter(), HAL_LIN_SendBreak(), HAL_MultiProcessor_DisableMuteMode(), HAL_MultiProcessor_EnableMuteMode(), HAL_MultiProcessor_EnterMuteMode(), HAL_UART_Abort_IT(), HAL_UART_AbortReceive_IT() (+27 more)

### Community 25 - "Core ARMv8-M Mainline"
Cohesion: 0.13
Nodes (37): ITM_CheckChar(), ITM_ReceiveChar(), ITM_SendChar(), __NVIC_ClearPendingIRQ(), NVIC_ClearTargetState(), NVIC_DecodePriority(), __NVIC_DisableIRQ(), __NVIC_EnableIRQ() (+29 more)

### Community 26 - "Core Cortex-M33"
Cohesion: 0.13
Nodes (37): ITM_CheckChar(), ITM_ReceiveChar(), ITM_SendChar(), __NVIC_ClearPendingIRQ(), NVIC_ClearTargetState(), NVIC_DecodePriority(), __NVIC_DisableIRQ(), __NVIC_EnableIRQ() (+29 more)

### Community 28 - "App Scheduler Loop"
Cohesion: 0.09
Nodes (22): app_init(), app_run(), baro_task(), debug_task(), dwt_init(), fault_task(), fusion_update(), imu_dt_s() (+14 more)

### Community 29 - "Telemetry Framing + CRC"
Cohesion: 0.15
Nodes (27): debug_flush(), crc16_ccitt(), check(), finish(), frame_data_pack(), frame_data_unpack(), frame_debug_pack(), frame_debug_unpack() (+19 more)

### Community 30 - "LL EXTI Driver"
Cohesion: 0.11
Nodes (32): LL_EXTI_ClearFlag_0_31(), LL_EXTI_ClearFlag_32_63(), LL_EXTI_DisableEvent_0_31(), LL_EXTI_DisableEvent_32_63(), LL_EXTI_DisableFallingTrig_0_31(), LL_EXTI_DisableFallingTrig_32_63(), LL_EXTI_DisableIT_0_31(), LL_EXTI_DisableIT_32_63() (+24 more)

### Community 31 - "Calibration State Machine"
Cohesion: 0.14
Nodes (31): imu_task(), telemetry_task(), calibration_active(), calibration_apply_accel(), calibration_apply_gyro(), calibration_apply_mag(), calibration_feed_baro(), calibration_feed_gyro() (+23 more)

### Community 32 - "HAL Tick + Delay"
Cohesion: 0.09
Nodes (16): bsp_delay_ms(), SysTick_Handler(), HAL_NVIC_SetPriorityGrouping(), HAL_SYSTICK_Config(), HAL_DeInit(), HAL_Delay(), HAL_GetTickFreq(), HAL_IncTick() (+8 more)

### Community 33 - "Core ARMv8-M Baseline"
Cohesion: 0.16
Nodes (30): __NVIC_ClearPendingIRQ(), NVIC_ClearTargetState(), NVIC_DecodePriority(), __NVIC_DisableIRQ(), __NVIC_EnableIRQ(), NVIC_EncodePriority(), __NVIC_GetActive(), __NVIC_GetEnableIRQ() (+22 more)

### Community 34 - "Flash Storage Path"
Cohesion: 0.09
Nodes (11): bsp_flash_read(), bsp_flash_write_page(), cycles_start(), cycles_to_ms(), program_bytes(), write_locked(), bsp_uart_init(), HAL_UART_ErrorCallback() (+3 more)

### Community 35 - "SPEC + Calibration Docs"
Cohesion: 0.11
Nodes (29): Calibration acceptance checklist, calib_state byte encoding, Calibration storage format v2 (magic ALC1), SPEC — 10-DOF IMU Node, Build acceptance criteria AC-01…AC-10, Signed-permutation axis map, Alpha-beta baro filter + altitude formula, Single-byte calibration commands G/A/M/B/S/D/R (+21 more)

### Community 36 - "Newlib Syscalls"
Cohesion: 0.08
Nodes (7): __io_putchar(), _exit(), _kill(), starm_getc(), starm_putc(), _times(), _write()

### Community 37 - "LL Cortex Driver"
Cohesion: 0.14
Nodes (25): LL_CPUID_GetConstant(), LL_CPUID_GetImplementer(), LL_CPUID_GetParNo(), LL_CPUID_GetRevision(), LL_CPUID_GetVariant(), LL_HANDLER_DisableFault(), LL_HANDLER_EnableFault(), LL_LPM_DisableEventOnPend() (+17 more)

### Community 38 - "CMSIS ARMCC Intrinsics"
Cohesion: 0.14
Nodes (21): __get_APSR(), __get_BASEPRI(), __get_CONTROL(), __get_FAULTMASK(), __get_FPSCR(), __get_IPSR(), __get_MSP(), __get_PRIMASK() (+13 more)

### Community 39 - "LL GPIO Driver"
Cohesion: 0.21
Nodes (23): LL_GPIO_GetAFPin_0_7(), LL_GPIO_GetAFPin_8_15(), LL_GPIO_GetPinMode(), LL_GPIO_GetPinOutputType(), LL_GPIO_GetPinPull(), LL_GPIO_GetPinSpeed(), LL_GPIO_IsAnyPinLocked(), LL_GPIO_IsInputPinSet() (+15 more)

### Community 40 - "LL PWR Driver"
Cohesion: 0.15
Nodes (24): LL_PWR_ClearFlag_SB(), LL_PWR_ClearFlag_WU(), LL_PWR_DisableBkUpAccess(), LL_PWR_DisablePVD(), LL_PWR_DisableSDADC(), LL_PWR_DisableWakeUpPin(), LL_PWR_EnableBkUpAccess(), LL_PWR_EnablePVD() (+16 more)

### Community 41 - "Device Headers"
Cohesion: 0.18
Nodes (20): ITM_CheckChar(), ITM_ReceiveChar(), ITM_SendChar(), __NVIC_ClearPendingIRQ(), NVIC_DecodePriority(), __NVIC_DisableIRQ(), __NVIC_EnableIRQ(), NVIC_EncodePriority() (+12 more)

### Community 43 - "HAL RCC Driver"
Cohesion: 0.12
Nodes (15): HAL_RCCEx_GetPeriphCLKConfig(), HAL_RCCEx_GetPeriphCLKFreq(), HAL_RCCEx_PeriphCLKConfig(), RCC_GetPLLCLKFreq(), HAL_RCC_ClockConfig(), HAL_RCC_CSSCallback(), HAL_RCC_DeInit(), HAL_RCC_GetClockConfig() (+7 more)

### Community 44 - "HAL MSP Init"
Cohesion: 0.16
Nodes (16): HAL_I2C_MspDeInit(), HAL_I2C_MspInit(), HAL_PCD_MspDeInit(), HAL_PCD_MspInit(), HAL_SPI_MspDeInit(), HAL_SPI_MspInit(), HAL_TIM_Base_MspDeInit(), HAL_TIM_Base_MspInit() (+8 more)

### Community 45 - "Main + Error Handler"
Cohesion: 0.19
Nodes (15): Error_Handler(), main(), MX_DMA_Init(), MX_GPIO_Init(), MX_I2C1_Init(), MX_I2C2_Init(), MX_SPI1_Init(), MX_TIM6_Init() (+7 more)

### Community 46 - "Core Cortex-M3"
Cohesion: 0.20
Nodes (20): ITM_CheckChar(), ITM_ReceiveChar(), ITM_SendChar(), __NVIC_ClearPendingIRQ(), NVIC_DecodePriority(), __NVIC_DisableIRQ(), __NVIC_EnableIRQ(), NVIC_EncodePriority() (+12 more)

### Community 47 - "HAL Timer Callbacks"
Cohesion: 0.14
Nodes (20): HAL_TIM_ErrorCallback(), HAL_TIM_IC_CaptureCallback(), HAL_TIM_IC_CaptureHalfCpltCallback(), HAL_TIM_IRQHandler(), HAL_TIM_OC_DelayElapsedCallback(), HAL_TIM_PeriodElapsedCallback(), HAL_TIM_PeriodElapsedHalfCpltCallback(), HAL_TIM_PWM_PulseFinishedCallback() (+12 more)

### Community 48 - "ARMv8 MPU"
Cohesion: 0.24
Nodes (17): ARM_MPU_ClrRegion(), ARM_MPU_ClrRegion_NS(), ARM_MPU_ClrRegionEx(), ARM_MPU_Disable(), ARM_MPU_Disable_NS(), ARM_MPU_Enable(), ARM_MPU_Enable_NS(), ARM_MPU_Load() (+9 more)

### Community 49 - "Build + Analysis Targets"
Cohesion: 0.13
Nodes (17): analyze target (-fanalyzer, advisory), build_info target (gen/build_info.h), cppcheck target, format-check target (clang-format 23), Fault handling §21.5 (OK flags, 1 s re-init, IMU-only fallback), M0 Build skeleton, M5 Hardening (faults, CI, reproducibility), Reproducibility (§15) (+9 more)

### Community 50 - "HAL DMA Driver"
Cohesion: 0.23
Nodes (15): DMA_CalcBaseAndBitshift(), DMA_SetConfig(), HAL_DMA_Abort(), HAL_DMA_DeInit(), HAL_DMA_GetError(), HAL_DMA_GetState(), HAL_DMA_Init(), HAL_DMA_IRQHandler() (+7 more)

### Community 51 - "Altitude + Baro Filter"
Cohesion: 0.14
Nodes (7): altitude_from_pressure(), baro_filter_init(), baro_filter_update(), test_altitude(), test_baro_filter(), test_crc16(), main()

### Community 52 - "Board Axis Mapping"
Cohesion: 0.21
Nodes (11): board_axes_apply_accel(), board_axes_apply_gyro(), board_axes_apply_mag(), axis_map_apply(), axis_map_is_signed_permutation(), det3(), test_all_orientations(), test_axis_map() (+3 more)

### Community 54 - "HAL Cortex/NVIC"
Cohesion: 0.13
Nodes (8): HAL_MPU_ConfigRegion(), HAL_NVIC_ClearPendingIRQ(), HAL_NVIC_GetActive(), HAL_NVIC_GetPendingIRQ(), HAL_NVIC_GetPriority(), HAL_NVIC_SetPendingIRQ(), HAL_SYSTICK_Callback(), HAL_SYSTICK_IRQHandler()

### Community 55 - "HAL UART Callbacks"
Cohesion: 0.16
Nodes (17): HAL_UART_AbortCpltCallback(), HAL_UART_AbortTransmitCpltCallback(), HAL_UART_RxCpltCallback(), HAL_UART_RxHalfCpltCallback(), HAL_UART_TxCpltCallback(), HAL_UART_TxHalfCpltCallback(), HAL_UARTEx_RxEventCallback(), UART_DMAReceiveCplt() (+9 more)

### Community 56 - "HAL Timer Channel Config"
Cohesion: 0.26
Nodes (14): HAL_TIM_IC_ConfigChannel(), HAL_TIM_OC_ConfigChannel(), HAL_TIM_OnePulse_ConfigChannel(), HAL_TIM_PWM_ConfigChannel(), TIM_OC1_SetConfig(), TIM_OC2_SetConfig(), TIM_OC3_SetConfig(), TIM_OC4_SetConfig() (+6 more)

### Community 58 - "Core Cortex-M0"
Cohesion: 0.28
Nodes (14): __NVIC_ClearPendingIRQ(), NVIC_DecodePriority(), __NVIC_DisableIRQ(), __NVIC_EnableIRQ(), NVIC_EncodePriority(), __NVIC_GetEnableIRQ(), __NVIC_GetPendingIRQ(), __NVIC_GetPriority() (+6 more)

### Community 59 - "Core Cortex-M0+"
Cohesion: 0.28
Nodes (14): __NVIC_ClearPendingIRQ(), NVIC_DecodePriority(), __NVIC_DisableIRQ(), __NVIC_EnableIRQ(), NVIC_EncodePriority(), __NVIC_GetEnableIRQ(), __NVIC_GetPendingIRQ(), __NVIC_GetPriority() (+6 more)

### Community 60 - "Core Cortex-M1"
Cohesion: 0.28
Nodes (14): __NVIC_ClearPendingIRQ(), NVIC_DecodePriority(), __NVIC_DisableIRQ(), __NVIC_EnableIRQ(), NVIC_EncodePriority(), __NVIC_GetEnableIRQ(), __NVIC_GetPendingIRQ(), __NVIC_GetPriority() (+6 more)

### Community 61 - "LL Bus Clocks"
Cohesion: 0.22
Nodes (15): LL_AHB1_GRP1_DisableClock(), LL_AHB1_GRP1_EnableClock(), LL_AHB1_GRP1_ForceReset(), LL_AHB1_GRP1_IsEnabledClock(), LL_AHB1_GRP1_ReleaseReset(), LL_APB1_GRP1_DisableClock(), LL_APB1_GRP1_EnableClock(), LL_APB1_GRP1_ForceReset() (+7 more)

### Community 62 - "Interrupt Handlers"
Cohesion: 0.13
Nodes (6): DMA1_Channel4_IRQHandler(), I2C1_EV_IRQHandler(), I2C2_EV_IRQHandler(), SPI1_IRQHandler(), TIM6_DAC_IRQHandler(), HAL_I2C_EV_IRQHandler()

### Community 63 - "HAL Timer Init"
Cohesion: 0.17
Nodes (14): HAL_TIM_Base_Init(), HAL_TIM_Base_MspInit(), HAL_TIM_Encoder_Init(), HAL_TIM_Encoder_MspInit(), HAL_TIM_IC_Init(), HAL_TIM_IC_MspInit(), HAL_TIM_OC_Init(), HAL_TIM_OC_MspInit() (+6 more)

### Community 64 - "HAL UART Extended"
Cohesion: 0.27
Nodes (10): HAL_MultiProcessorEx_AddressLength_Set(), HAL_UARTEx_DisableStopMode(), HAL_UARTEx_EnableStopMode(), HAL_UARTEx_GetRxEventType(), HAL_UARTEx_ReceiveToIdle(), HAL_UARTEx_ReceiveToIdle_DMA(), HAL_UARTEx_ReceiveToIdle_IT(), HAL_UARTEx_StopModeWakeUpSourceConfig() (+2 more)

### Community 65 - "Core SC000"
Cohesion: 0.32
Nodes (12): __NVIC_ClearPendingIRQ(), __NVIC_DisableIRQ(), __NVIC_EnableIRQ(), __NVIC_GetEnableIRQ(), __NVIC_GetPendingIRQ(), __NVIC_GetPriority(), __NVIC_GetVector(), __NVIC_SetPendingIRQ() (+4 more)

### Community 66 - "Calibration Store Codec"
Cohesion: 0.31
Nodes (11): cal_store_decode(), cal_store_encode(), cal_store_load(), cal_store_save(), get_f32(), get_u16(), get_u32(), put_f32() (+3 more)

### Community 67 - "Firmware Library Stack"
Cohesion: 0.29
Nodes (9): add_our_library(), app (STATIC target), bsp (STATIC target), fusion (STATIC target), OUR_WARNINGS / OUR_WERROR / OUR_FANALYZER, sensors (STATIC target), Latency probe T-11 (LAT_PROBE PC0, NFR-09), HOST_TEST compile definition (+1 more)

### Community 68 - "HAL Timer Sync Config"
Cohesion: 0.23
Nodes (9): HAL_TIM_ConfigClockSource(), HAL_TIM_ConfigOCrefClear(), HAL_TIM_SlaveConfigSynchro(), HAL_TIM_SlaveConfigSynchro_IT(), TIM_ETR_SetConfig(), TIM_ITRx_SetConfig(), TIM_SlaveTimer_SetConfig(), TIM_TI1_ConfigInputStage() (+1 more)

### Community 69 - "HAL GPIO Driver"
Cohesion: 0.27
Nodes (6): HAL_GPIO_EXTI_Callback(), HAL_GPIO_EXTI_IRQHandler(), HAL_GPIO_LockPin(), HAL_GPIO_ReadPin(), HAL_GPIO_TogglePin(), HAL_GPIO_WritePin()

### Community 70 - "Spec Build Rules + Budget"
Cohesion: 0.24
Nodes (8): CMakeLists.txt (root), Size budget post-build (FLASH_BUDGET/RAM_BUDGET), CI pipeline (§14, TR-09…TR-14), CMake architecture (§9), Host-side unit testing (§12), Part A — Build system, Size budget enforcement (§9.5), T-12/T-13 30 min endurance + resources (pass)

### Community 71 - "CMSIS Compiler Abstraction"
Cohesion: 0.27
Nodes (5): T_UINT16_READ(), T_UINT16_WRITE(), T_UINT32(), T_UINT32_READ(), T_UINT32_WRITE()

### Community 72 - "Calibration Procedure Docs"
Cohesion: 0.22
Nodes (8): Calibration procedure (T-04/T-15), Accelerometer six-position calibration, Barometer reference p0 (B), Gyro bias procedure (G, SEM < 0.05 °/s), Magnetometer min/max hard/soft-iron calibration, Save and verify (S, flash-stall compensation), T-03 axis measurement procedure, M2 Axis + identity (T-02, T-03)

### Community 73 - "ARMv7 MPU"
Cohesion: 0.36
Nodes (7): ARM_MPU_ClrRegion(), ARM_MPU_Disable(), ARM_MPU_Enable(), ARM_MPU_Load(), ARM_MPU_SetRegion(), ARM_MPU_SetRegionEx(), orderedCpy()

### Community 74 - "HAL PWR Extended"
Cohesion: 0.22
Nodes (3): HAL_PWR_ConfigPVD(), HAL_PWR_PVD_IRQHandler(), HAL_PWR_PVDCallback()

### Community 75 - "HAL UART Modes Init"
Cohesion: 0.56
Nodes (10): HAL_RS485Ex_Init(), HAL_HalfDuplex_Init(), HAL_LIN_Init(), HAL_MultiProcessor_Init(), HAL_UART_Init(), HAL_UART_MspInit(), UART_AdvFeatureConfig(), UART_CheckIdleState() (+2 more)

### Community 76 - "CubeMX CMake Targets"
Cohesion: 0.29
Nodes (6): CubeMX-generated CMakeLists.txt, MX_Application_Src (main.c, ISRs, startup), STM32_Drivers OBJECT library (HAL), stm32cubemx INTERFACE library, 010_9dof_bmp280_cmake (executable), flash target (OpenOCD)

### Community 77 - "UART DMA IRQ Path"
Cohesion: 0.32
Nodes (8): USART1_IRQHandler(), HAL_UART_DMAStop(), HAL_UART_ErrorCallback(), HAL_UART_IRQHandler(), UART_DMAAbortOnError(), UART_DMAError(), UART_EndRxTransfer(), UART_EndTxTransfer()

### Community 78 - "CubeMX Regeneration"
Cohesion: 0.29
Nodes (6): stm32cubemx (INTERFACE, generated), STM32CubeMX project generation (§7), Headless CubeMX regeneration (§7.4), CubeMX regeneration check (AC-08), Release checklist, tools/regenerate.txt (headless CubeMX script)

### Community 79 - "Sensor Frames + Axes"
Cohesion: 0.33
Nodes (6): FRD body frame / NED earth frame, I3G4250D gyro (SPI1, CS PE3), LSM303AGR accel/mag (I2C1), Per-sensor signed permutation matrix (body = M * sensor), STM32F3DISCOVERY MB1035 (STM32F303VCT6), T-05 static attitude (partial)

### Community 80 - "LL Utils"
Cohesion: 0.48
Nodes (5): LL_GetFlashSize(), LL_GetUID_Word0(), LL_GetUID_Word1(), LL_GetUID_Word2(), LL_InitTick()

### Community 81 - "Plan + Fusion Tuning"
Cohesion: 0.40
Nodes (4): PLAN.md (implementation plan), M4 Fusion (Madgwick 100 Hz, baro filter), Madgwick beta = 0.1, Madgwick beta tuning replay

### Community 82 - "ARMClang Packed Access"
Cohesion: 0.40
Nodes (4): T_UINT16_READ(), T_UINT16_WRITE(), T_UINT32_READ(), T_UINT32_WRITE()

### Community 83 - "GCC Packed Access"
Cohesion: 0.40
Nodes (4): T_UINT16_READ(), T_UINT16_WRITE(), T_UINT32_READ(), T_UINT32_WRITE()

## Ambiguous Edges - Review These
- `bmp280_init=ld_bmp280_init rename` → `unit_tests (host executable)`  [AMBIGUOUS]
  tests/CMakeLists.txt · relation: conceptually_related_to

## Knowledge Gaps
- **29 isolated node(s):** `v`, `CMSIS Device STM32F3xx License`, `CMSIS Apache License 2.0`, `STM32F3xx HAL Driver License`, `PLAN.md (implementation plan)` (+24 more)
  These have ≤1 connection - possible missing edges or undocumented components. (Counts symbols only; 303 node(s) total have ≤1 connection when file, concept and rationale nodes are included.)
- **17 thin communities (<3 nodes) omitted from report** — run `graphify query` to explore isolated nodes.

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **Why does `HAL_GetTick()` connect `Sensor Drivers + SPI Bus` to `HAL Tick + Delay`, `HAL UART Extended`, `Flash Storage Path`, `Newlib Syscalls`, `HAL I2C Driver`, `HAL RCC Driver`, `HAL UART Modes Init`, `HAL DMA Driver`, `HAL EXTI Driver`, `BMP280 Wrapper + I2C Bus`, `HAL UART Driver`, `Calibration State Machine`?**
  _High betweenness centrality (0.033) - this node is a cross-community bridge._
- **What connects `v`, `CMSIS Device STM32F3xx License`, `CMSIS Apache License 2.0` to the rest of the system?**
  _29 weakly-connected nodes found - possible documentation gaps or missing edges._
- **Should `LL USART Driver` be split into smaller, more focused modules?**
  _Cohesion score 0.02803362246266655 - nodes in this community are weakly interconnected._
- **Why does `__disable_irq()` connect `CMSIS GCC Intrinsics` to `App Scheduler Loop`, `Main + Error Handler`?**
  _High betweenness centrality (0.032) - this node is a cross-community bridge._
- **Should `LL I2C Driver` be split into smaller, more focused modules?**
  _Cohesion score 0.043151567196221555 - nodes in this community are weakly interconnected._
- **Why does `LL_MPU_Enable()` connect `LL Cortex Driver` to `Core Barriers + Reset`?**
  _High betweenness centrality (0.031) - this node is a cross-community bridge._

### Low-confidence Hints
_AMBIGUOUS edges — the extractor was unsure. Verify before acting on these._

- **What is the exact relationship between `bmp280_init=ld_bmp280_init rename` and `unit_tests (host executable)`?**
  _Edge tagged AMBIGUOUS (relation: conceptually_related_to) - confidence is low._