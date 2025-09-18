# UART stuffs

## redirect UART output to a file

```shell
cat /dev/ttyACM0 >> stm32_esp32_bme280.log &
```

stop the logging

```shell
ps aux | grep cat
kill <PID>
```
