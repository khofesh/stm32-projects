# Graph Report - .  (2026-08-08)

## Corpus Check
- Corpus is ~11,040 words - fits in a single context window. You may not need a graph.

## Summary
- 96 nodes · 128 edges · 12 communities (11 shown, 1 thin omitted)
- Extraction: 96% EXTRACTED · 4% INFERRED · 0% AMBIGUOUS · INFERRED: 5 edges (avg confidence: 0.8)
- Token cost: 0 input · 0 output

## Community Hubs (Navigation)
- BSP LED and Button API
- Newlib Syscall Stubs
- BSP UART Console Port
- Application Main and Peripheral Init
- HAL MSP Low-Level Init

## God Nodes (most connected - your core abstractions)
1. `main()` - 12 edges
2. `Error_Handler()` - 8 edges
3. `BSP_COM_Init()` - 7 edges
4. `MX_USART_Init()` - 5 edges
5. `BSP_PB_Init()` - 4 edges
6. `BSP_PB_Callback()` - 4 edges
7. `SystemClock_Config()` - 3 edges
8. `MX_DCACHE1_Init()` - 3 edges
9. `MX_GPDMA1_Init()` - 3 edges
10. `MX_ICACHE_Init()` - 3 edges

## Surprising Connections (you probably didn't know these)
- `main()` --calls--> `BSP_COM_Init()`  [INFERRED]
  Core/Src/main.c → Drivers/BSP/STM32H5xx_Nucleo/stm32h5xx_nucleo.c
- `main()` --calls--> `BSP_LED_Init()`  [INFERRED]
  Core/Src/main.c → Drivers/BSP/STM32H5xx_Nucleo/stm32h5xx_nucleo.c
- `main()` --calls--> `BSP_PB_Init()`  [INFERRED]
  Core/Src/main.c → Drivers/BSP/STM32H5xx_Nucleo/stm32h5xx_nucleo.c
- `EXTI13_IRQHandler()` --calls--> `BSP_PB_IRQHandler()`  [INFERRED]
  Core/Src/stm32h5xx_it.c → Drivers/BSP/STM32H5xx_Nucleo/stm32h5xx_nucleo.c
- `HAL_SD_MspInit()` --calls--> `Error_Handler()`  [INFERRED]
  Core/Src/stm32h5xx_hal_msp.c → Core/Src/main.c

## Import Cycles
- None detected.

## Communities (12 total, 1 thin omitted)

### Community 0 - "BSP LED and Button API"
Cohesion: 0.13
Nodes (17): Button_TypeDef, ButtonMode_TypeDef, EXTI13_IRQHandler(), BSP_LED_DeInit(), BSP_LED_GetState(), BSP_LED_Init(), BSP_LED_Off(), BSP_LED_On() (+9 more)

### Community 2 - "BSP UART Console Port"
Cohesion: 0.19
Nodes (13): BSP_COM_Cb_t, COM_InitTypeDef, COM_TypeDef, BSP_COM_DeInit(), BSP_COM_Init(), BSP_COM_RegisterDefaultMspCallbacks(), BSP_COM_RegisterMspCallbacks(), BSP_COM_SelectLogPort() (+5 more)

### Community 4 - "Application Main and Peripheral Init"
Cohesion: 0.42
Nodes (9): Error_Handler(), main(), MPU_Config(), MX_DCACHE1_Init(), MX_GPDMA1_Init(), MX_GPIO_Init(), MX_ICACHE_Init(), MX_SDMMC1_SD_Init() (+1 more)

### Community 5 - "HAL MSP Low-Level Init"
Cohesion: 0.32
Nodes (6): HAL_DCACHE_MspDeInit(), HAL_DCACHE_MspInit(), HAL_SD_MspDeInit(), HAL_SD_MspInit(), DCACHE_HandleTypeDef, SD_HandleTypeDef

## Knowledge Gaps
- **1 thin communities (<3 nodes) omitted from report** — run `graphify query` to explore isolated nodes.

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **Why does `main()` connect `Application Main and Peripheral Init` to `BSP LED and Button API`, `BSP UART Console Port`?**
  _High betweenness centrality (0.207) - this node is a cross-community bridge._
- **Why does `EXTI13_IRQHandler()` connect `BSP LED and Button API` to `Cortex-M and Peripheral IRQ Handlers`?**
  _High betweenness centrality (0.151) - this node is a cross-community bridge._
- **Are the 3 inferred relationships involving `main()` (e.g. with `BSP_COM_Init()` and `BSP_LED_Init()`) actually correct?**
  _`main()` has 3 INFERRED edges - model-reasoned connections that need verification._
- **Should `BSP LED and Button API` be split into smaller, more focused modules?**
  _Cohesion score 0.1341991341991342 - nodes in this community are weakly interconnected._
- **Should `Newlib Syscall Stubs` be split into smaller, more focused modules?**
  _Cohesion score 0.1111111111111111 - nodes in this community are weakly interconnected._