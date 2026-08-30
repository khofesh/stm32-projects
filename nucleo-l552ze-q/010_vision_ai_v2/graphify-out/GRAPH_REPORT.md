# Graph Report - .  (2026-08-30)

## Corpus Check
- Corpus is ~19,208 words - fits in a single context window. You may not need a graph.

## Summary
- 218 nodes · 418 edges · 14 communities
- Extraction: 94% EXTRACTED · 6% INFERRED · 0% AMBIGUOUS · INFERRED: 26 edges (avg confidence: 0.81)
- Token cost: 41,872 input · 0 output

## Community Hubs (Navigation)
- SSCMA Transport Layer
- Nucleo BSP Peripherals
- SSCMA JSON Response Parser
- Inference Result Data Types
- Newlib Syscall Stubs
- I2C Inference Debug Trace
- Application Init And Main Loop
- HAL MSP Peripheral Wiring

## God Nodes (most connected - your core abstractions)
1. `wait_response()` - 21 edges
2. `main()` - 19 edges
3. `sscma_write()` - 15 edges
4. `find_key()` - 12 edges
5. `delay_ms()` - 12 edges
6. `sscma_begin_spi()` - 11 edges
7. `sscma_begin_i2c()` - 10 edges
8. `Error_Handler()` - 9 edges
9. `sscma_begin_uart()` - 9 edges
10. `parse_invoke_results()` - 9 edges

## Surprising Connections (you probably didn't know these)
- `main()` --calls--> `BSP_COM_Init()`  [INFERRED]
  Core/Src/main.c → Drivers/BSP/STM32L5xx_Nucleo/stm32l5xx_nucleo.c
- `main()` --calls--> `BSP_LED_Init()`  [INFERRED]
  Core/Src/main.c → Drivers/BSP/STM32L5xx_Nucleo/stm32l5xx_nucleo.c
- `main()` --calls--> `BSP_PB_Init()`  [INFERRED]
  Core/Src/main.c → Drivers/BSP/STM32L5xx_Nucleo/stm32l5xx_nucleo.c
- `main()` --calls--> `sscma_begin_i2c()`  [INFERRED]
  Core/Src/main.c → Drivers/sscma/sscma_stm32l5.c
- `main()` --calls--> `sscma_err_to_str()`  [INFERRED]
  Core/Src/main.c → Drivers/sscma/sscma_stm32l5.c

## Import Cycles
- None detected.

## Hyperedges (group relationships)
- **AT+INVOKE Request/Ack/Result Flow over I2C** — log_at_invoke_command, log_invoke_ack_payload, log_i2c_available_polling, log_invoke_result_payload, log_two_phase_response_protocol [EXTRACTED 1.00]
- **Repeated Inference Timeout Failure Mode** — log_inference_timeout_failure, log_i2c_available_polling, log_response_desync, log_id_name_query_commands [INFERRED 0.85]

## Communities (14 total, 0 thin omitted)

### Community 0 - "SSCMA Transport Layer"
Cohesion: 0.12
Nodes (50): I2C_HandleTypeDef, SPI_HandleTypeDef, UART_HandleTypeDef, delay_ms(), i2c_available(), i2c_cmd(), i2c_read(), i2c_write() (+42 more)

### Community 1 - "Nucleo BSP Peripherals"
Cohesion: 0.09
Nodes (31): BSP_COM_Cb_t, Button_TypeDef, ButtonMode_TypeDef, COM_InitTypeDef, COM_TypeDef, EXTI13_IRQHandler(), BSP_COM_DeInit(), BSP_COM_Init() (+23 more)

### Community 2 - "SSCMA JSON Response Parser"
Cohesion: 0.21
Nodes (21): sscma_box_t, sscma_point_t, find_key(), skip_token(), sscma_parse_get_boxes(), sscma_parse_get_classes(), sscma_parse_get_data_string(), sscma_parse_get_keypoints() (+13 more)

### Community 3 - "Inference Result Data Types"
Cohesion: 0.09
Nodes (22): sscma_box_t, sscma_point_t, sscma_box_s, h, score, target, w, x (+14 more)

### Community 4 - "Newlib Syscall Stubs"
Cohesion: 0.11
Nodes (4): _exit(), _kill(), _write(), __io_putchar()

### Community 6 - "I2C Inference Debug Trace"
Cohesion: 0.16
Nodes (15): 240x240 Auto Sensor Mode, AT+INVOKE=1,1,0 Command, Base64 JPEG Frame in Response, Bounding Box Detection Result, Grove Vision AI V2 Device (ID a412c445), I2C Available-Byte Polling Loop, ID? and NAME? Device Query Commands, Inference Perf Metrics (preprocess/inference/postprocess) (+7 more)

### Community 7 - "Application Init And Main Loop"
Cohesion: 0.29
Nodes (12): Error_Handler(), main(), MX_DMA_Init(), MX_GPIO_Init(), MX_I2C1_Init(), MX_ICACHE_Init(), MX_SPI1_Init(), MX_UART4_Init() (+4 more)

### Community 8 - "HAL MSP Peripheral Wiring"
Cohesion: 0.24
Nodes (9): I2C_HandleTypeDef, SPI_HandleTypeDef, UART_HandleTypeDef, HAL_I2C_MspDeInit(), HAL_I2C_MspInit(), HAL_SPI_MspDeInit(), HAL_SPI_MspInit(), HAL_UART_MspDeInit() (+1 more)

## Knowledge Gaps
- **19 isolated node(s):** `x`, `y`, `w`, `h`, `score` (+14 more)
  These have ≤1 connection - possible missing edges or undocumented components.

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **Why does `main()` connect `Application Init And Main Loop` to `SSCMA Transport Layer`, `Nucleo BSP Peripherals`?**
  _High betweenness centrality (0.468) - this node is a cross-community bridge._
- **Why does `BSP_COM_Init()` connect `Nucleo BSP Peripherals` to `Application Init And Main Loop`?**
  _High betweenness centrality (0.144) - this node is a cross-community bridge._
- **Why does `__io_putchar()` connect `Newlib Syscall Stubs` to `Nucleo BSP Peripherals`?**
  _High betweenness centrality (0.142) - this node is a cross-community bridge._
- **Are the 3 inferred relationships involving `wait_response()` (e.g. with `sscma_parse_get_data_string()` and `sscma_parse_get_string()`) actually correct?**
  _`wait_response()` has 3 INFERRED edges - model-reasoned connections that need verification._
- **Are the 10 inferred relationships involving `main()` (e.g. with `BSP_COM_Init()` and `BSP_LED_Init()`) actually correct?**
  _`main()` has 10 INFERRED edges - model-reasoned connections that need verification._
- **What connects `x`, `y`, `w` to the rest of the system?**
  _19 weakly-connected nodes found - possible documentation gaps or missing edges._
- **Should `SSCMA Transport Layer` be split into smaller, more focused modules?**
  _Cohesion score 0.12470588235294118 - nodes in this community are weakly interconnected._