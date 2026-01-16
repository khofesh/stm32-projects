- [x] convert video streaming example in `PICO_SPI_CAM-master/C/Examples/Arducam_MINI_5MP_Plus_Videostreaming/Arducam_MINI_5MP_Plus_Videostreaming.cpp` to STM32 HAL

## Implementation Details

### Features Implemented

- **Command Processing**: Host can send single-byte commands via UART (LPUART1/VCP)
- **Single Capture Mode**: Command 0x10 triggers single image capture
- **Video Streaming Mode**: Command 0x20 starts continuous capture, 0x21 stops
- **Resolution Control**: Commands 0x00-0x06 set resolution (320x240 to 2592x1944)
- **Quality Control**: Commands 0xD0-0xD2 set JPEG quality (high/default/low)
- **Frame Protocol**: Images sent with markers (0xFF 0xAA ... length ... data ... 0xBB 0xCC)

### Communication

- Uses **LPUART1** (COM1) via ST-Link VCP at 115200 baud
- Interrupt-based command reception for non-blocking operation
- Compatible with existing `host_app/capture_image.py`

### Files Modified

- `Core/Src/main.c`: Added command handler, capture modes, UART communication
- `Core/Src/stm32l5xx_it.c`: Added LPUART1 interrupt handler

### Usage

1. Flash firmware to Nucleo-L552ZE-Q
2. Connect ArduCAM OV5642 (SPI: PA5/PA6/PA7/PD14, I2C: PB8/PB9)
3. Run: `python host_app/capture_image.py --port /dev/ttyACM0 --mode single`
4. For streaming: `python host_app/capture_image.py --port /dev/ttyACM0 --mode streaming`
