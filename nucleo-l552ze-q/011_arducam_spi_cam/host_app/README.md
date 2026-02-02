# ArduCAM Host Application

Python host application for capturing images from STM32 + ArduCAM via serial port.

## Requirements

- Python 3.7+
- pyserial

## Installation

```bash
pip install -r requirements.txt
```

## Usage

### Single Image Capture

```bash
python capture_image.py --port /dev/ttyACM0 --size 640x480 --save-location ./images
```

### Video Streaming

```bash
python capture_image.py --port /dev/ttyACM0 --mode streaming --save-location ./frames
```

### All Options

```bash
python capture_image.py \
    --port /dev/ttyACM0 \
    --baud-rate 115200 \
    --save-location ./images \
    --size 1600x1200 \
    --quality high \
    --camera-model OV5642 \
    --mode single \
    --timeout 10.0
```

## Command Line Arguments

| Argument          | Description                           | Default  |
| ----------------- | ------------------------------------- | -------- |
| `--port`          | Serial port (required)                | -        |
| `--baud-rate`     | Baud rate                             | 115200   |
| `--save-location` | Directory to save images              | ./images |
| `--size`          | Resolution                            | 640x480  |
| `--quality`       | JPEG quality (high/default/low)       | default  |
| `--camera-model`  | Camera model                          | OV5642   |
| `--mode`          | Capture mode (single/streaming)       | single   |
| `--max-frames`    | Max frames in streaming (0=unlimited) | 0        |
| `--timeout`       | Read timeout in seconds               | 10.0     |

## Supported Resolutions

- 320x240
- 640x480
- 1024x768
- 1280x960
- 1600x1200
- 2048x1536
- 2592x1944

## Protocol

### Commands (Host → STM32)

| Command   | Description                           |
| --------- | ------------------------------------- |
| 0x00-0x06 | Set resolution (320x240 to 2592x1944) |
| 0x10      | Single capture (JPEG)                 |
| 0x20      | Start video streaming                 |
| 0x21      | Stop video streaming                  |
| 0xD0      | Set high quality                      |
| 0xD1      | Set default quality                   |
| 0xD2      | Set low quality                       |

### Frame Format (STM32 → Host)

```
[Start Marker: 0xFF 0xAA]
[Length: 4 bytes, little-endian]
[Image Data: JPEG bytes]
[End Marker: 0xBB 0xCC]
```

## Examples

### Capture high-resolution image

```bash
python capture_image.py --port /dev/ttyACM0 --size 2592x1944 --quality high

python capture_image.py --port /dev/ttyACM0 --size 1280x960 --quality high --save-location ./images
```

### Stream 100 frames

```bash
python capture_image.py --port /dev/ttyACM0 --mode streaming --max-frames 100
```

### Windows example

```bash
python capture_image.py --port COM3 --size 640x480 --save-location C:\images
```
