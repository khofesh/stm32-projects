# STM32 + DHT20

- DHT20 VCC -> 3.3V
- DHT20 GND -> GND
- DHT20 SDA -> PB7 (I2C1_SDA)
- DHT20 SCL -> PB6 (I2C1_SCL)
- UART TX -> PA9 (for debug output)

sensor used : https://www.tokopedia.com/khurs-iot/dht20-temperature-humidity-i2c-iic-dht-20-sensor-suhu-dan-kelembaban-original-asair-for-arduino?extParam=src%3Dshop%26whid%3D7029488&aff_unique_id=&channel=others&chain_key=

description:

```
The DHT20 temperature & humidity sensor employs I2C digital output protocol, which is very suitable for HVAC, automobiles, data loggers, weather stations, home appliances, and other related temperature and humidity detection and controlled areas.

FEATURES
Factory calibration
I2C digital output
Quick response, strong anti-interference ability
Low power, low cost
Compatible with mainboards like Arduino, micro:bit, ESP32, Raspberry Pi
Compatible with 3.3~5.5V main controller
APPLICATIONS
HVAC
Automobiles
Data loggers
Weather stations
Home appliances

SPECIFICATION
Operating Voltage: 3.3V ~ 5.5V DC
Operating Current: ＜1mA
Output Signal: I2C (0×38)
Temperature Range: -40 ~ +80°C ±0.5°C
Humidity Range: 0~100%RH ±3%RH (25°C)
Resolution: 0.01℃, 0.024%RH
Storage Condition: 10℃-50℃, 20-60%RH
Board Dimension: 22*33 mm/0.87*1.30”
Mounting Hole Size: Inner diameter 3.1mm (0.13”)/Outer diameter 6 mm (0.24”)

Packing List:
Temperature and Humidity Sensor x1
4Pin I2C Sensor Connector x1
```
