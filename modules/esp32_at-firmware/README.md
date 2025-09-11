# ESP-AT

- https://github.com/espressif/esp-at
- documentation: https://docs.espressif.com/projects/esp-at/en/latest/esp32/Compile_and_Develop/How_to_clone_project_and_compile_it.html

## how to

device used: ESP32 from VCC-GND with 16MB flash

default UART:
- 16 - RX
- 17 - TX

how to know it
```shell
$ esptool.py --port /dev/ttyACM0 flash_id
esptool.py v4.8.1
Serial port /dev/ttyACM0
Connecting....
Detecting chip type... Unsupported detection protocol, switching and trying again...
Connecting......
Detecting chip type... ESP32
Chip is ESP32-D0WD-V3 (revision v3.0)
Features: WiFi, BT, Dual Core, 240MHz, VRef calibration in efuse, Coding Scheme None
Crystal is 40MHz
MAC: c0:49:ef:27:e2:88
Uploading stub...
Running stub...
Stub running...
Manufacturer: 20
Device: 4018
Detected flash size: 16MB
Flash voltage set by a strapping pin to 3.3V
Hard resetting via RTS pin...
```

build and flash AT firmware

```shell
$ git clone --recursive https://github.com/espressif/esp-at.git
$ cd esp-at
$ . $HOME/esp/v5.4.1/esp-idf/export.sh
$  ./build.py menuconfig
Platform name:
1. PLATFORM_ESP32
2. PLATFORM_ESP32C3
3. PLATFORM_ESP32C2
4. PLATFORM_ESP32C5
5. PLATFORM_ESP32C6
6. PLATFORM_ESP32S2
choose(range[1,6]):1

Module name:
1. WROOM-32	(Firmware description: 4MB, Wi-Fi + BLE, OTA, TX:17 RX:16)
2. PICO-D4	(Firmware description: 4MB, Wi-Fi + BLE, OTA, TX:22 RX:19)
3. MINI-1	(Firmware description: 4MB, Wi-Fi + BLE, OTA, TX:22 RX:19)
4. ESP32-SDIO	(Firmware description: 4MB, Wi-Fi + BLE, OTA, communicate with MCU via SDIO)
5. ESP32-D2WD	(Firmware description: 2MB, Wi-Fi + BLE, No OTA, TX:22 RX:19)
6. SOLO-1	(Firmware description: Not recommended for new design. 4MB, Wi-Fi + BLE, OTA, TX:17 RX:16)
7. WROVER-32	(Firmware description: Not recommended for new design. 4MB, Wi-Fi + BLE, OTA, TX:22 RX:19)
choose(range[1,7]):1

Enable silence mode to remove some logs and reduce the firmware size?
0. No
1. Yes
choose(range[0,1]):1
Platform name:ESP32	Module name:WROOM-32	Silence:1

$ ./build.py build
$ idf.py -p /dev/ttyACM0 flash

```
