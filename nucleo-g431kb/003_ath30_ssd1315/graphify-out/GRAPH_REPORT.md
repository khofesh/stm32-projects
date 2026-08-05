# Graph Report - .  (2026-08-05)

## Corpus Check
- 78 files · ~305,305 words
- Verdict: corpus is large enough that graph structure adds value.

## Summary
- 1625 nodes · 3831 edges · 53 communities (49 shown, 4 thin omitted)
- Extraction: 95% EXTRACTED · 5% INFERRED · 0% AMBIGUOUS · INFERRED: 181 edges (avg confidence: 0.8)
- Token cost: 0 input · 0 output

## Community Hubs (Navigation)
- LL I2C Register Layer
- UART / DMA Transfers
- AHT30 Interface Glue
- LL DMA Channels
- LL Power Control
- LL System / DBGMCU
- SSD1315 OLED Driver
- LL DMAMUX Routing
- LL RCC Clock Flags
- LL RCC Clock Config
- HAL USART Driver
- LL CRS Clock Recovery
- LL Bus Clock Gating
- HAL Flash Programming
- LL EXTI Lines
- HAL PWR Extended
- Interrupt Handlers
- Nucleo BSP COM Port
- LL Cortex Core Access
- LL GPIO Access
- HAL RCC Clock Setup
- HAL NVIC / MPU
- GPIO and Board LEDs
- Newlib Syscall Stubs
- HAL PWR Core
- HAL RCCEx / CRS
- AHT30 Sensor Driver
- SSD1315 Interface Glue
- HAL EXTI Driver
- HAL USARTEx FIFO
- MSP Peripheral Init
- HAL DMAEx MUX
- HAL I2CEx Filters
- SysTick Timebase
- Application Main
- LL Utils / Device ID
- HAL Init Lifecycle
- PVD / PVM Callbacks
- Ring Buffer
- Flash RAM Functions
- Stop Mode Entry
- Low Power Run Exit
- Low Power Run Entry

## God Nodes (most connected - your core abstractions)
1. `HAL_GetTick()` - 36 edges
2. `I2C_Enable_IRQ()` - 29 edges
3. `a_ssd1315_multiple_write_byte()` - 26 edges
4. `I2C_TransferConfig()` - 25 edges
5. `HAL_DMA_Start_IT()` - 21 edges
6. `I2C_ITError()` - 19 edges
7. `I2C_Disable_IRQ()` - 17 edges
8. `a_ssd1315_write_byte()` - 17 edges
9. `I2C_WaitOnFlagUntilTimeout()` - 15 edges
10. `HAL_DMA_Abort_IT()` - 14 edges

## Surprising Connections (you probably didn't know these)
- `ssd1315_interface_iic_write()` --calls--> `HAL_I2C_Mem_Write()`  [INFERRED]
  Core/Src/driver_ssd1315_interface.c → Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_i2c.c
- `aht30_interface_iic_deinit()` --calls--> `HAL_I2C_DeInit()`  [INFERRED]
  Core/Src/driver_aht30_interface.c → Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_i2c.c
- `aht30_interface_delay_ms()` --calls--> `HAL_Delay()`  [INFERRED]
  Core/Src/driver_aht30_interface.c → Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal.c
- `ssd1315_interface_delay_ms()` --calls--> `HAL_Delay()`  [INFERRED]
  Core/Src/driver_ssd1315_interface.c → Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal.c
- `main()` --calls--> `BSP_COM_Init()`  [INFERRED]
  Core/Src/main.c → Drivers/BSP/STM32G4xx_Nucleo/stm32g4xx_nucleo.c

## Import Cycles
- None detected.

## Communities (53 total, 4 thin omitted)

### Community 0 - "LL I2C Register Layer"
Cohesion: 0.04
Nodes (136): __STATIC_INLINE, LL_I2C_AcknowledgeNextData(), LL_I2C_ClearFlag_ADDR(), LL_I2C_ClearFlag_ARLO(), LL_I2C_ClearFlag_BERR(), LL_I2C_ClearFlag_NACK(), LL_I2C_ClearFlag_OVR(), LL_I2C_ClearFlag_STOP() (+128 more)

### Community 1 - "UART / DMA Transfers"
Cohesion: 0.06
Nodes (109): HAL_DMA_Abort(), HAL_DMA_GetError(), DMA_HandleTypeDef, FlagStatus, HAL_StatusTypeDef, UART_HandleTypeDef, __weak, HAL_StatusTypeDef (+101 more)

### Community 2 - "AHT30 Interface Glue"
Cohesion: 0.07
Nodes (105): aht30_interface_iic_deinit(), aht30_interface_iic_read_cmd(), aht30_interface_iic_write_cmd(), I2C_HandleTypeDef, HAL_I2C_ErrorCallback(), HAL_I2C_MasterRxCpltCallback(), HAL_I2C_MasterTxCpltCallback(), HAL_DMA_Abort_IT() (+97 more)

### Community 3 - "LL DMA Channels"
Cohesion: 0.06
Nodes (106): DMA_TypeDef, __STATIC_INLINE, LL_DMA_ClearFlag_GI1(), LL_DMA_ClearFlag_GI2(), LL_DMA_ClearFlag_GI3(), LL_DMA_ClearFlag_GI4(), LL_DMA_ClearFlag_GI5(), LL_DMA_ClearFlag_GI6() (+98 more)

### Community 4 - "LL Power Control"
Cohesion: 0.05
Nodes (80): __STATIC_INLINE, LL_PWR_ClearFlag_SB(), LL_PWR_ClearFlag_WU(), LL_PWR_ClearFlag_WU1(), LL_PWR_ClearFlag_WU2(), LL_PWR_ClearFlag_WU3(), LL_PWR_ClearFlag_WU4(), LL_PWR_ClearFlag_WU5() (+72 more)

### Community 5 - "LL System / DBGMCU"
Cohesion: 0.05
Nodes (80): __STATIC_INLINE, LL_DBGMCU_APB1_GRP1_FreezePeriph(), LL_DBGMCU_APB1_GRP1_UnFreezePeriph(), LL_DBGMCU_APB1_GRP2_FreezePeriph(), LL_DBGMCU_APB1_GRP2_UnFreezePeriph(), LL_DBGMCU_APB2_GRP1_FreezePeriph(), LL_DBGMCU_APB2_GRP1_UnFreezePeriph(), LL_DBGMCU_DisableDBGSleepMode() (+72 more)

### Community 6 - "SSD1315 OLED Driver"
Cohesion: 0.06
Nodes (76): a_ssd1315_gram_draw_point(), a_ssd1315_gram_show_char(), a_ssd1315_multiple_write_byte(), a_ssd1315_write_byte(), ssd1315_activate_scroll(), ssd1315_clear(), ssd1315_deactivate_scroll(), ssd1315_deinit() (+68 more)

### Community 7 - "LL DMAMUX Routing"
Cohesion: 0.08
Nodes (71): DMAMUX_Channel_TypeDef, __STATIC_INLINE, LL_DMAMUX_ClearFlag_RGO0(), LL_DMAMUX_ClearFlag_RGO1(), LL_DMAMUX_ClearFlag_RGO2(), LL_DMAMUX_ClearFlag_RGO3(), LL_DMAMUX_ClearFlag_SO0(), LL_DMAMUX_ClearFlag_SO1() (+63 more)

### Community 8 - "LL RCC Clock Flags"
Cohesion: 0.03
Nodes (71): LL_RCC_ClearFlag_HSIRDY(), LL_RCC_ClearFlag_LSECSS(), LL_RCC_ClearFlag_LSIRDY(), LL_RCC_ClearFlag_PLLRDY(), LL_RCC_ClearResetFlags(), LL_RCC_DisableIT_HSI48RDY(), LL_RCC_DisableIT_HSIRDY(), LL_RCC_DisableIT_LSECSS() (+63 more)

### Community 9 - "LL RCC Clock Config"
Cohesion: 0.03
Nodes (72): __STATIC_INLINE, LL_RCC_ClearFlag_HSECSS(), LL_RCC_ClearFlag_HSERDY(), LL_RCC_ClearFlag_HSI48RDY(), LL_RCC_ClearFlag_LSERDY(), LL_RCC_ConfigMCO(), LL_RCC_DisableIT_HSERDY(), LL_RCC_EnableIT_HSERDY() (+64 more)

### Community 10 - "HAL USART Driver"
Cohesion: 0.09
Nodes (60): DMA_HandleTypeDef, FlagStatus, HAL_StatusTypeDef, USART_HandleTypeDef, __weak, HAL_USART_Abort(), HAL_USART_Abort_IT(), HAL_USART_AbortCpltCallback() (+52 more)

### Community 11 - "LL CRS Clock Recovery"
Cohesion: 0.08
Nodes (46): __STATIC_INLINE, LL_CRS_ClearFlag_ERR(), LL_CRS_ClearFlag_ESYNC(), LL_CRS_ClearFlag_SYNCOK(), LL_CRS_ClearFlag_SYNCWARN(), LL_CRS_ConfigSynchronization(), LL_CRS_DisableAutoTrimming(), LL_CRS_DisableFreqErrorCounter() (+38 more)

### Community 12 - "LL Bus Clock Gating"
Cohesion: 0.09
Nodes (43): __STATIC_INLINE, LL_AHB1_GRP1_DisableClock(), LL_AHB1_GRP1_DisableClockStopSleep(), LL_AHB1_GRP1_EnableClock(), LL_AHB1_GRP1_EnableClockStopSleep(), LL_AHB1_GRP1_ForceReset(), LL_AHB1_GRP1_IsEnabledClock(), LL_AHB1_GRP1_ReleaseReset() (+35 more)

### Community 13 - "HAL Flash Programming"
Cohesion: 0.10
Nodes (31): HAL_StatusTypeDef, __weak, HAL_StatusTypeDef, FLASH_FlushCaches(), FLASH_MassErase(), FLASH_OB_BootLockConfig(), FLASH_OB_PCROPConfig(), FLASH_OB_RDPConfig() (+23 more)

### Community 14 - "LL EXTI Lines"
Cohesion: 0.11
Nodes (33): __STATIC_INLINE, LL_EXTI_ClearFlag_0_31(), LL_EXTI_ClearFlag_32_63(), LL_EXTI_DisableEvent_0_31(), LL_EXTI_DisableEvent_32_63(), LL_EXTI_DisableFallingTrig_0_31(), LL_EXTI_DisableFallingTrig_32_63(), LL_EXTI_DisableIT_0_31() (+25 more)

### Community 15 - "HAL PWR Extended"
Cohesion: 0.07
Nodes (11): HAL_StatusTypeDef, HAL_PWREx_ConfigPVM(), HAL_PWREx_ControlVoltageScaling(), HAL_PWREx_DisableGPIOPullDown(), HAL_PWREx_DisableGPIOPullUp(), HAL_PWREx_DisableLowPowerRunMode(), HAL_PWREx_EnableGPIOPullDown(), HAL_PWREx_EnableGPIOPullUp() (+3 more)

### Community 16 - "Interrupt Handlers"
Cohesion: 0.12
Nodes (20): DMA1_Channel1_IRQHandler(), DMA1_Channel2_IRQHandler(), I2C1_ER_IRQHandler(), I2C1_EV_IRQHandler(), DMA_HandleTypeDef, HAL_StatusTypeDef, DMA_CalcDMAMUXChannelBaseAndMask(), DMA_CalcDMAMUXRequestGenBaseAndMask() (+12 more)

### Community 17 - "Nucleo BSP COM Port"
Cohesion: 0.11
Nodes (19): BSP_COM_Cb_t, Button_TypeDef, COM_InitTypeDef, COM_TypeDef, BSP_COM_DeInit(), BSP_COM_Init(), BSP_COM_RegisterDefaultMspCallbacks(), BSP_COM_RegisterMspCallbacks() (+11 more)

### Community 18 - "LL Cortex Core Access"
Cohesion: 0.14
Nodes (26): __STATIC_INLINE, LL_CPUID_GetArchitecture(), LL_CPUID_GetImplementer(), LL_CPUID_GetParNo(), LL_CPUID_GetRevision(), LL_CPUID_GetVariant(), LL_HANDLER_DisableFault(), LL_HANDLER_EnableFault() (+18 more)

### Community 20 - "LL GPIO Access"
Cohesion: 0.21
Nodes (25): GPIO_TypeDef, __STATIC_INLINE, LL_GPIO_GetAFPin_0_7(), LL_GPIO_GetAFPin_8_15(), LL_GPIO_GetPinMode(), LL_GPIO_GetPinOutputType(), LL_GPIO_GetPinPull(), LL_GPIO_GetPinSpeed() (+17 more)

### Community 21 - "HAL RCC Clock Setup"
Cohesion: 0.13
Nodes (20): HAL_NVIC_SetPriority(), HAL_SYSTICK_Config(), HAL_InitTick(), HAL_StatusTypeDef, __weak, HAL_RCCEx_GetPeriphCLKFreq(), HAL_RCC_ClockConfig(), HAL_RCC_CSSCallback() (+12 more)

### Community 22 - "HAL NVIC / MPU"
Cohesion: 0.12
Nodes (14): ButtonMode_TypeDef, BSP_PB_Init(), __weak, HAL_MPU_ConfigRegion(), HAL_NVIC_ClearPendingIRQ(), HAL_NVIC_EnableIRQ(), HAL_NVIC_GetActive(), HAL_NVIC_GetPendingIRQ() (+6 more)

### Community 23 - "GPIO and Board LEDs"
Cohesion: 0.15
Nodes (20): BSP_LED_DeInit(), BSP_LED_GetState(), BSP_LED_Init(), BSP_LED_Off(), BSP_LED_On(), BSP_LED_Toggle(), GPIO_TypeDef, HAL_StatusTypeDef (+12 more)

### Community 26 - "HAL PWR Core"
Cohesion: 0.12
Nodes (7): HAL_StatusTypeDef, HAL_PWR_ConfigPVD(), HAL_PWR_DisableBkUpAccess(), HAL_PWR_EnableBkUpAccess(), HAL_RCCEx_DisableLSCO(), HAL_RCCEx_EnableLSCO(), PWR_PVDTypeDef

### Community 27 - "HAL RCCEx / CRS"
Cohesion: 0.17
Nodes (13): __weak, HAL_RCCEx_CRS_ErrorCallback(), HAL_RCCEx_CRS_ExpectedSyncCallback(), HAL_RCCEx_CRS_IRQHandler(), HAL_RCCEx_CRS_SyncOkCallback(), HAL_RCCEx_CRS_SyncWarnCallback(), HAL_RCCEx_CRSConfig(), HAL_RCCEx_CRSGetSynchronizationInfo() (+5 more)

### Community 28 - "AHT30 Sensor Driver"
Cohesion: 0.31
Nodes (15): aht30_handle_t, aht30_info_t, a_aht30_calc_crc(), a_aht30_iic_read(), a_aht30_iic_write(), a_aht30_jh_reset_reg(), aht30_deinit(), aht30_get_reg() (+7 more)

### Community 30 - "HAL EXTI Driver"
Cohesion: 0.29
Nodes (13): HAL_StatusTypeDef, HAL_EXTI_ClearConfigLine(), HAL_EXTI_ClearPending(), HAL_EXTI_GenerateSWI(), HAL_EXTI_GetConfigLine(), HAL_EXTI_GetHandle(), HAL_EXTI_GetPending(), HAL_EXTI_IRQHandler() (+5 more)

### Community 31 - "HAL USARTEx FIFO"
Cohesion: 0.35
Nodes (13): HAL_StatusTypeDef, USART_HandleTypeDef, __weak, HAL_USARTEx_ConfigNSS(), HAL_USARTEx_DisableFifoMode(), HAL_USARTEx_DisableSlaveMode(), HAL_USARTEx_EnableFifoMode(), HAL_USARTEx_EnableSlaveMode() (+5 more)

### Community 32 - "MSP Peripheral Init"
Cohesion: 0.18
Nodes (12): I2C_HandleTypeDef, HAL_I2C_MspDeInit(), HAL_I2C_MspInit(), HAL_MspInit(), BSP_PB_DeInit(), HAL_NVIC_DisableIRQ(), HAL_GPIO_DeInit(), HAL_PWREx_DisableUCPDDeadBattery() (+4 more)

### Community 33 - "HAL DMAEx MUX"
Cohesion: 0.36
Nodes (9): DMA_HandleTypeDef, HAL_StatusTypeDef, HAL_DMAEx_ConfigMuxRequestGenerator(), HAL_DMAEx_ConfigMuxSync(), HAL_DMAEx_DisableMuxRequestGenerator(), HAL_DMAEx_EnableMuxRequestGenerator(), HAL_DMAEx_MUX_IRQHandler(), HAL_DMA_MuxRequestGeneratorConfigTypeDef (+1 more)

### Community 34 - "HAL I2CEx Filters"
Cohesion: 0.39
Nodes (6): HAL_StatusTypeDef, I2C_HandleTypeDef, HAL_I2CEx_ConfigAnalogFilter(), HAL_I2CEx_ConfigDigitalFilter(), HAL_I2CEx_DisableWakeUp(), HAL_I2CEx_EnableWakeUp()

### Community 35 - "SysTick Timebase"
Cohesion: 0.25
Nodes (8): aht30_interface_delay_ms(), ssd1315_interface_delay_ms(), SysTick_Handler(), __weak, HAL_Delay(), HAL_IncTick(), HAL_ResumeTick(), HAL_SuspendTick()

### Community 36 - "Application Main"
Cohesion: 0.50
Nodes (6): Error_Handler(), main(), MX_DMA_Init(), MX_GPIO_Init(), MX_I2C1_Init(), SystemClock_Config()

### Community 37 - "LL Utils / Device ID"
Cohesion: 0.43
Nodes (7): __STATIC_INLINE, LL_GetFlashSize(), LL_GetPackageType(), LL_GetUID_Word0(), LL_GetUID_Word1(), LL_GetUID_Word2(), LL_InitTick()

### Community 38 - "HAL Init Lifecycle"
Cohesion: 0.25
Nodes (8): HAL_StatusTypeDef, HAL_NVIC_SetPriorityGrouping(), HAL_DeInit(), HAL_Init(), HAL_MspDeInit(), HAL_MspInit(), HAL_SetTickFreq(), HAL_SYSCFG_EnableVREFBUF()

### Community 39 - "PVD / PVM Callbacks"
Cohesion: 0.36
Nodes (8): __weak, __weak, HAL_PWREx_PVD_PVM_IRQHandler(), HAL_PWREx_PVM1Callback(), HAL_PWREx_PVM2Callback(), HAL_PWREx_PVM3Callback(), HAL_PWREx_PVM4Callback(), HAL_PWR_PVDCallback()

### Community 40 - "Ring Buffer"
Cohesion: 0.62
Nodes (6): RingBuffer_GetDataLength(), RingBuffer_GetFreeSpace(), RingBuffer_Init(), RingBuffer_Read(), RingBuffer_Write(), RingBuffer

### Community 41 - "Flash RAM Functions"
Cohesion: 0.33
Nodes (5): HAL_FLASHEx_OB_DBankConfig(), void(), HAL_FLASHEx_DisableRunPowerDown, HAL_FLASHEx_EnableRunPowerDown, __RAM_FUNC

### Community 43 - "Stop Mode Entry"
Cohesion: 0.67
Nodes (3): HAL_PWREx_EnterSTOP0Mode(), HAL_PWREx_EnterSTOP1Mode(), HAL_PWR_EnterSTOPMode()

## Knowledge Gaps
- **4 thin communities (<3 nodes) omitted from report** — run `graphify query` to explore isolated nodes.

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **Why does `HAL_GetTick()` connect `AHT30 Interface Glue` to `MSP Peripheral Init`, `UART / DMA Transfers`, `SysTick Timebase`, `HAL Init Lifecycle`, `HAL USART Driver`, `HAL Flash Programming`, `Interrupt Handlers`, `HAL DBGMCU / Version`, `HAL RCC Clock Setup`, `HAL RCCEx / CRS`?**
  _High betweenness centrality (0.076) - this node is a cross-community bridge._
- **Why does `FLASH_WaitForLastOperation()` connect `HAL Flash Programming` to `AHT30 Interface Glue`?**
  _High betweenness centrality (0.017) - this node is a cross-community bridge._
- **Why does `SystemClock_Config()` connect `Application Main` to `HAL RCC Clock Setup`, `HAL PWR Extended`?**
  _High betweenness centrality (0.017) - this node is a cross-community bridge._
- **Should `LL I2C Register Layer` be split into smaller, more focused modules?**
  _Cohesion score 0.043151567196221555 - nodes in this community are weakly interconnected._
- **Should `UART / DMA Transfers` be split into smaller, more focused modules?**
  _Cohesion score 0.056511056511056514 - nodes in this community are weakly interconnected._
- **Should `AHT30 Interface Glue` be split into smaller, more focused modules?**
  _Cohesion score 0.07373428474345906 - nodes in this community are weakly interconnected._
- **Should `LL DMA Channels` be split into smaller, more focused modules?**
  _Cohesion score 0.05501675189560924 - nodes in this community are weakly interconnected._