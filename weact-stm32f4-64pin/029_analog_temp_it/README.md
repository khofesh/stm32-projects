# analog temperature in interrupt mode

```c
  HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);

  // Enable ADC interrupt
  HAL_NVIC_SetPriority(ADC_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(ADC_IRQn);

  // Enable Timer interrupt
  HAL_NVIC_SetPriority(TIM2_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
```

UART (priority 0) > ADC (priority 2) > Timer (priority 3)
