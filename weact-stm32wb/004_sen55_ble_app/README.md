# notify SEN55 data and read-write board LED

- seeed studio SEN55
- waveshare USB to TTL

## connection:

PB7 (TX) --> waveshare RX \
PB6 (RX) --> waveshare TX

PE4 -> board LED

PB8 (SCL) --> yellow cable (SCL) \
PB9 (SDA) --> white cable (SDA)

![the sensor](./6280451413775599031.jpg)

## reading data

see `stm32-projects/applications/sen55-reader` on how to read SEN55 data

![alt text](image.png)
