# esp32c6

The SDIO slave pins are as below:

- CLK is GPIO19 -> (pcb number) 23/8
- CMD is GPIO18 ->24/7
- DAT0 is GPIO20 ->22/9
- DAT1 is GPIO21 ->21/10
- DAT2 is GPIO22 (for 4-bit mode only) ->20/11
- DAT3 is GPIO23 (for 4-bit mode only) ->19/12

## verifying the wiring from the slave side

The host-side diagnostic can only prove its own nets are pulled up and drivable;
it cannot see where a wire lands. Read `GPIO_IN_REG` on the C6 instead, from the
ROM bootloader, while `SDIO_PORT_CK_TOGGLE_MS` sweeps one host pin at a time:

```python
from esptool.cmds import detect_chip
esp = detect_chip("/dev/ttyACM1", 115200)   # the C6 UART bridge
while True:
    print(hex(esp.read_reg(0x6009103C)))    # GPIO_IN_REG, bit N = GPIO N
```

esptool resets the C6 into download mode, where GPIO18-23 are plain inputs, so
whichever bit shows the 2 Hz square wave is the pad that host pin is really on.
A plain reset afterwards brings esp-at back; nothing is reflashed.

Do not leave a host pin parked low on **GPIO9** - it is the BOOT strap.
