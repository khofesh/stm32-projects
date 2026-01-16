```shell
19:30:12.609 -> ArduCAM driver initialized
19:30:12.609 -> Testing SPI communication...
19:30:12.609 ->   Write 0x55 to reg 0x00: OK
19:30:12.643 ->   Read reg 0x00: 0x00 (expected 0x55), status: OK
19:30:12.643 ->   Write 0xAA to reg 0x00: OK
19:30:12.643 ->   Read reg 0x00: 0xAA (expected 0xAA), status: OK
19:30:12.643 -> SPI communication OK
19:30:12.643 -> Scanning I2C bus...
19:30:12.643 ->   Found device at 0x3C
19:30:12.673 -> Trying OV5642 at address 0x3C...
19:30:12.673 -> Sensor detected, Chip ID: 0x5642
19:30:15.114 -> Camera sensor initialized
19:30:15.114 -> VSYNC level set
19:30:16.176 -> Resolution set to 640x480
19:30:17.173 -> Camera in standby mode (low power)
19:30:17.173 -> Camera ready. Waiting for commands...
19:30:45.543 -> Starting single capture...
19:30:45.863 -> Capture started, waiting for completion...
19:30:46.119 -> Capture done!
19:30:46.119 -> Raw FIFO regs: 0x42=58, 0x43=59, 0x44=00
19:30:46.119 -> FIFO length: 22872 bytes
19:30:46.215 -> First bytes: FF D8, Last bytes: 00 00
19:31:15.868 -> Starting single capture...
19:31:16.157 -> Capture started, waiting for completion...
19:31:16.446 -> Capture done!
19:31:16.446 -> Raw FIFO regs: 0x42=78, 0x43=5E, 0x44=00
19:31:16.446 -> FIFO length: 24184 bytes
19:31:16.510 -> First bytes: FF D8, Last bytes: 00 00
19:31:37.603 -> Starting single capture...
19:31:37.924 -> Capture started, waiting for completion...
19:31:38.181 -> Capture done!
19:31:38.181 -> Raw FIFO regs: 0x42=28, 0x43=4D, 0x44=00
19:31:38.181 -> FIFO length: 19752 bytes
19:31:38.245 -> First bytes: FF D8, Last bytes: 00 00
```
