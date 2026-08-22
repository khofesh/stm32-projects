# scripts

Host side helpers for the WeAct STM32WB55 + BME280 board.

## bme280_notify.py

Subscribes to the `bme280_char` characteristic and prints each notification, so
you can confirm the device is publishing environmental data.

```bash
uv sync
uv run bme280_notify.py
```

```text
scanning for 'WeEnv' (it advertises about once a second, be patient) ...
found WeEnv at DE:AD:BE:EF:00:01
subscribed, Ctrl-C to stop
silence means nothing moved past the notification threshold

[21:04:11.382]             27.41 C    1006.35 hPa   54.28 %RH
[21:04:31.512]  +20.1s     27.48 C    1006.33 hPa   54.31 %RH
[21:04:33.480]   +2.0s     27.63 C    1006.34 hPa   55.02 %RH
```

Gaps of roughly 20 s are STABLE state. Breathe on the sensor and the gaps
should collapse to about 1 s as the firmware switches to ACTIVE; leave it alone
and they stretch back out after ~30 s.

Notifications are thresholded (`BLE_TEMP_NOTIFY_DELTA_C_X100` and friends in
`Core/Inc/app_config.h`), so a perfectly still room can go minutes without one.
That is the intended behaviour, not a hang.

To check the advertising payload instead, without connecting:

```bash
uv run bme280_notify.py --scan-only
```

That decodes the manufacturer-specific field, which carries the latest
temperature and humidity while the device is disconnected.

### Notes

- Linux needs BlueZ; no root required for scanning in a normal desktop session.
- If the device is already connected to a phone it will not be connectable
  here — this firmware accepts one central at a time.

### Discovery fails right after reflashing

```text
found WeEnv at 00:80:E1:26:06:2E
attempt 1/3 failed: failed to discover services, device disconnected
attempt 2/3 failed: failed to discover services, device disconnected
attempt 3/3 failed: (<BleakGATTProtocolErrorCode.UNLIKELY_ERROR: 14>, ...)
```

BlueZ caches the attribute table per device address, for unbonded devices too.
Flashing firmware that moves attribute handles leaves that cache describing a
database the device no longer has, and discovery fails against the stale copy —
repeatably, until the cache is dropped. Advertising still works throughout,
which is what makes it look like a firmware fault.

Drop the cached entry and reconnect:

```bash
bluetoothctl remove 00:80:E1:26:06:2E
```

Substitute the address the scan printed. Worth doing as the first step any time
discovery breaks after a flash, before suspecting the firmware. To tell the two
apart, the device itself is fine if `STM32_Programmer_CLI -c port=SWD
mode=Hotplug -coreReg` shows CPU1 in Thread mode inside `UTIL_SEQ_Run` — halt
state is in `../README.md` under "verifying on the bench".
