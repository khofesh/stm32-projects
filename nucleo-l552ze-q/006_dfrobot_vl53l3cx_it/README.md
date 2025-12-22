# VL53L3CX i2c interrupt mode

logs:
```shell
22:41:42.488 -> Count=  131, #Objs=2 status=0, D=   59mm, Signal=7.81 Mcps, Ambient=0.11 Mcps
22:41:42.488 ->                      status=0, D= 1257mm, Signal=0.04 Mcps, Ambient=0.11 Mcps
22:41:42.520 -> Count=  132, #Objs=2 status=0, D=   58mm, Signal=7.90 Mcps, Ambient=0.09 Mcps
22:41:42.520 ->                      status=0, D= 1376mm, Signal=0.11 Mcps, Ambient=0.09 Mcps
22:41:42.552 -> Count=  133, #Objs=2 status=0, D=   59mm, Signal=7.80 Mcps, Ambient=0.11 Mcps
22:41:42.552 ->                      status=0, D= 1261mm, Signal=0.04 Mcps, Ambient=0.11 Mcps
22:41:42.584 -> Count=  134, #Objs=2 status=0, D=   59mm, Signal=7.88 Mcps, Ambient=0.09 Mcps
22:41:42.584 ->                      status=0, D= 1359mm, Signal=0.11 Mcps, Ambient=0.09 Mcps
```

- `Count=131, 132, 133, 134`: Sequential measurement number (stream count)
- `#Objs=2`: Number of objects detected in this measurement (2 objects found)

```shell
status=0, D=59mm, Signal=7.81 Mcps, Ambient=0.11 Mcps
```

- status=0: Range status OK (valid measurement)
- D=59mm: Distance to object is 59 millimeters (~6cm)
- Signal=7.81 Mcps: Strong return signal (7.81 Mega counts per second)
- Ambient=0.11 Mcps: Low ambient light interference
