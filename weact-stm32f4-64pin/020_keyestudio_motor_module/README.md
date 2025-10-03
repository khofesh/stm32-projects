# stm32 + motor module

pins

- V -> 5V
- G -> GND
- S -> PA6 PWM

- Period: How long one complete on/off cycle takes (like counting from 0 to Period)
- Compare Value: At what count the signal switches from ON to OFF
- Duty Cycle: The percentage of time the signal is ON = (Compare Value / Period) × 100%

If Period = 1000 and Compare = 500:
```shell
Count:  0___500___________1000  (repeat)
Signal: |‾‾‾‾‾|___________|
        ON    OFF
        
Duty Cycle = 500/1000 = 50% (half power)
```

If Period = 1000 and Compare = 800:
```shell
Count:  0___________800__1000  (repeat)
Signal: |‾‾‾‾‾‾‾‾‾‾‾|__|
        ON          OFF
        
Duty Cycle = 800/1000 = 80% (more power)
```
