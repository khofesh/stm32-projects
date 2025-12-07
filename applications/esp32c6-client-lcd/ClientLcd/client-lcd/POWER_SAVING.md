# Power Saving Implementation Guide

## Overview

The ESP32-C6 BLE client has been modified to implement power-saving features by entering light sleep mode between sensor data notifications. This significantly reduces power consumption for battery-powered applications.

## Key Changes

### 1. Configuration Constants

```c
#define NOTIFICATION_INTERVAL_MS 60000  // 1 minute
#define SLEEP_DURATION_US (55 * 1000000ULL)  // 55 seconds
```

### 2. Timing Tracking

- `last_notification_time`: Tracks when the last notification was received
- `first_notification_received`: Flag to wait for initial connection before sleeping

### 3. Sleep Management Task

A dedicated FreeRTOS task (`sleep_management_task`) handles the sleep/wake cycle:

- Waits for the first notification to establish timing
- Calculates time until next notification
- **Disconnects BLE** before entering sleep (ESP32-C6 cannot maintain BLE during light sleep)
- Enters light sleep for ~50 seconds (wakes 10s before next notification)
- Turns off LCD backlight during sleep to save additional power
- **Reconnects to BLE** after waking up
- Allows time for reconnection and service discovery before next notification

### 4. Notification Handler Enhancement

The `ESP_GATTC_NOTIFY_EVT` handler now:

- Records the timestamp of each notification
- Sets the flag indicating data has been received
- Logs the sleep schedule

## Power Consumption

### Before (Active Waiting)

- CPU running continuously at full speed
- LCD backlight always on
- BLE connection active continuously
- Estimated: ~80-150mA continuous

### After (Light Sleep with Disconnect/Reconnect)

- CPU in light sleep for ~50 seconds per minute
- LCD backlight off during sleep
- BLE disconnected during sleep (no power consumption)
- BLE reconnection overhead: ~2-5 seconds at ~80-100mA
- Estimated: ~5-15mA average (depending on reconnection frequency)

**Power Savings**: Approximately 85-95% reduction in average power consumption

**Note**: The disconnect/reconnect approach is necessary because ESP32-C6 cannot maintain BLE connections during light sleep. The reconnection overhead is minimal compared to the power saved during sleep.

## Important Notes

### Server Configuration Required

**CRITICAL**: The STM32WB55 server must be configured to send notifications every 60 seconds to match the client's sleep schedule.

If your server currently sends notifications every 5 seconds, you need to:

1. **Option A - Modify Server (Recommended)**:

   - Change the notification timer on STM32WB55 from 5 seconds to 60 seconds
   - This provides maximum power savings on both client and server

2. **Option B - Adjust Client**:
   - Change `NOTIFICATION_INTERVAL_MS` to 5000 (5 seconds)
   - Change `SLEEP_DURATION_US` to 0 (disable sleep)
   - This defeats the power-saving purpose but maintains compatibility

### Light Sleep vs Deep Sleep

This implementation uses **light sleep with disconnect/reconnect** instead of deep sleep because:

- ✅ Fast wake-up time (~few milliseconds)
- ✅ RAM contents preserved
- ✅ FreeRTOS tasks continue seamlessly
- ✅ Lower power than maintaining BLE connection
- ⚠️ Requires BLE reconnection after wake (~2-5 seconds)
- ❌ Higher power consumption than deep sleep (~5-15mA vs ~10µA)

For even lower power consumption, consider deep sleep with periodic wake-up, but this requires:

- Full BLE reconnection on each wake
- Longer wake-up time (~100-300ms)
- More complex state management

## Testing

### Monitor Sleep Behavior

```bash
idf.py -p /dev/ttyUSB0 monitor
```

Expected log output:

```
I (xxxxx) SEN55_CLIENT: *** Notification received ***
I (xxxxx) SEN55_CLIENT: Next reading in 60 seconds. Entering light sleep...
I (xxxxx) SEN55_CLIENT: Preparing to sleep for 50 seconds
I (xxxxx) SEN55_CLIENT: Disconnecting BLE before sleep...
I (xxxxx) SEN55_CLIENT: Disconnected for sleep mode
I (xxxxx) SEN55_CLIENT: Entering light sleep for 50 seconds
I (xxxxx) SEN55_CLIENT: Woke up from light sleep
I (xxxxx) SEN55_CLIENT: Reconnecting to BLE device...
I (xxxxx) SEN55_CLIENT: Scanning started, looking for 'sen55CST'
I (xxxxx) SEN55_CLIENT: *** Found device 'sen55CST' ***
I (xxxxx) SEN55_CLIENT: Connected, conn_id X
```

### Measure Power Consumption

1. Use a multimeter in series with the power supply
2. Observe current draw over a full 60-second cycle
3. Expected pattern:
   - Active (0-5s): ~80-150mA (BLE activity, LCD update)
   - Disconnecting (5-6s): ~50-80mA (BLE disconnect)
   - Sleep (6-56s): ~5-10mA (light sleep, no BLE)
   - Reconnecting (56-60s): ~80-100mA (BLE scan and connect)
   - Average: ~10-20mA over full cycle

## Customization

### Change Notification Interval

To change to a different interval (e.g., 2 minutes):

```c
#define NOTIFICATION_INTERVAL_MS 120000  // 2 minutes
#define SLEEP_DURATION_US (115 * 1000000ULL)  // 115 seconds
```

### Disable Power Saving

To disable power saving and receive all notifications:

```c
#define NOTIFICATION_INTERVAL_MS 5000  // Match server rate
// Comment out or don't create the sleep_management_task
```

### Adjust Wake-Up Margin

The system wakes up 5 seconds before the expected notification. To adjust:

```c
// In sleep_management_task function
int64_t sleep_time = time_until_next - 5000000;  // Change 5000000 (5s) to desired value
```

## Troubleshooting

### Missing Notifications

**Symptom**: Client misses some notifications

**Causes**:

1. Server sending notifications more frequently than expected
2. Sleep duration too long
3. BLE connection instability

**Solutions**:

- Verify server notification interval matches client configuration
- Reduce sleep duration to wake up earlier
- Check BLE signal strength and connection parameters

### High Power Consumption

**Symptom**: Power consumption not reduced as expected

**Causes**:

1. LCD backlight not turning off
2. Other peripherals still active
3. BLE connection parameters not optimized

**Solutions**:

- Verify `gpio_set_level(PIN_NUM_BK_LIGHT, LCD_BK_LIGHT_OFF_LEVEL)` is called
- Disable unused peripherals before sleep
- Adjust BLE connection interval for lower power

### BLE Disconnections

**Symptom**: BLE connection drops during sleep

**Causes**:

1. Sleep duration exceeds BLE supervision timeout
2. Server not maintaining connection during client sleep

**Solutions**:

- Reduce sleep duration
- Increase BLE supervision timeout on both client and server
- Use shorter connection intervals

### Scan Starts Before Sleep

**Symptom**: "Scanning started" appears right after "Entering light sleep", device never actually sleeps

**Causes**:

1. BLE stack hasn't fully processed disconnect event
2. GAP state machine automatically starts scanning
3. Light sleep fails when BLE operations are active

**Solutions**:

- Increased disconnect wait time to 1000ms (implemented)
- Explicitly stop scanning before sleep (implemented)
- Verify connection is closed before sleeping (implemented)
- BLE stack needs time to settle after disconnect

## Future Enhancements

Possible improvements for even better power efficiency:

1. **Deep Sleep Mode**: Implement deep sleep with periodic wake-up for <10µA consumption
2. **Adaptive Sleep**: Adjust sleep duration based on actual notification timing
3. **Display Power Management**: Turn off display completely instead of just backlight
4. **BLE Parameter Optimization**: Tune connection interval and latency for lower power
5. **Battery Monitoring**: Add battery voltage monitoring and low-power warnings

## References

- [ESP32 Light Sleep Documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32c6/api-reference/system/sleep_modes.html)
- [ESP32 Power Management](https://docs.espressif.com/projects/esp-idf/en/latest/esp32c6/api-reference/system/power_management.html)
- [BLE Power Optimization](https://docs.espressif.com/projects/esp-idf/en/latest/esp32c6/api-guides/ble/ble-feature-list.html)
