# AGENTS.md

## Project Goal

Build a battery-powered environmental monitor using:

- STM32WB55
- BME280 temperature / humidity / pressure sensor
- I2C OLED display
- Bluetooth Low Energy

The firmware should prioritize, in this order:

1. Low battery consumption
2. Reliable BLE connectivity
3. Fast response when environmental conditions change
4. Useful display behavior without leaving the OLED continuously powered

The system should implement an **adaptive sampling strategy**.

Do not optimize for maximum continuous update rate. The device should spend most of its time sleeping and temporarily become more responsive when environmental conditions change or when the user interacts with it.

---

# Hardware Assumptions

Target MCU:

- STM32WB55
- Cortex-M4 application core
- Cortex-M0+ wireless core
- STM32CubeWB BLE stack

Sensors:

- BME280 over I2C

Display:

- Small I2C OLED
- Likely SSD1306-compatible
- Exact OLED controller may be changed later

Optional hardware:

- Push button for display wake
- Load switch or MOSFET controlling OLED power

Do not assume the OLED can be completely power-gated unless the board configuration explicitly provides an OLED power-control GPIO.

Keep OLED power control abstracted from the display driver.

---

# Design Philosophy

The firmware must be primarily:

**event-driven**

rather than:

**polling-driven**

Avoid code such as:

```c
while (1)
{
    read_sensor();
    update_display();
    HAL_Delay(1000);
}
```

The application should instead behave approximately as:

```text
             ┌──────────┐
             │  STOP2   │
             └────┬─────┘
                  │
       ┌──────────┼───────────┐
       │          │           │
       ▼          ▼           ▼
    RTC wake    BLE event   Button
       │          │           │
       └──────────┼───────────┘
                  ▼
             process event
                  │
                  ▼
              return to
                STOP2
```

Every subsystem should allow the MCU to sleep when no work is required.

---

# System States

Implement the application around three primary operating states:

```c
typedef enum
{
    APP_STATE_STABLE,
    APP_STATE_ACTIVE,
    APP_STATE_INTERACTIVE
} AppState;
```

## STABLE

Normal operating state.

Environmental conditions are not changing significantly.

Goals:

- maximum sleep time
- slow sensor sampling
- slow BLE activity
- OLED off

Suggested configuration:

```text
BME280 sample interval:       20 seconds
BLE advertising interval:     ~1000 ms
OLED:                         off
MCU:                          STOP2 whenever possible
```

The exact values must be configurable rather than scattered as magic numbers.

---

## ACTIVE

Entered when meaningful environmental change is detected.

Example triggers:

```text
|ΔTemperature| >= 0.10 °C

OR

|ΔHumidity| >= 1 %
```

Suggested behavior:

```text
BME280 sample interval:       1 second
BLE notifications:            enabled on meaningful change
BLE connection interval:      request faster interval if useful
OLED:                         normally remains off
```

Remain in ACTIVE mode while measurements continue changing.

Return to STABLE after the environment has remained sufficiently stable for approximately:

```text
30 seconds
```

Use configurable thresholds.

---

## INTERACTIVE

Entered when a user interaction occurs.

Initial trigger:

- physical button press

Potential future triggers:

- BLE command
- capacitive touch
- motion sensor

Behavior:

```text
OLED power:       ON
display refresh:  immediate
BLE:              normal connected operation
sensor:           fresh measurement if current data is stale
```

OLED should remain active for approximately:

```text
10 seconds
```

Each user interaction resets the timeout.

After timeout:

```text
OLED OFF
```

and return to either ACTIVE or STABLE depending on environmental state.

---

# Adaptive Sampling

The sensor sampling interval must adapt dynamically.

Default values:

```c
#define SENSOR_STABLE_INTERVAL_MS     20000
#define SENSOR_ACTIVE_INTERVAL_MS      1000

#define TEMP_CHANGE_THRESHOLD_C        0.10f
#define HUMIDITY_CHANGE_THRESHOLD      1.00f

#define ACTIVE_TIMEOUT_MS             30000
```

These values belong in a configuration header.

Do not hardcode them inside application logic.

---

# Environmental Change Detection

Keep at least:

```c
typedef struct
{
    float temperature_c;
    float humidity_percent;
    float pressure_pa;
    uint32_t timestamp_ms;
} EnvironmentalReading;
```

Maintain:

```text
current reading
previous reading
last significant change timestamp
```

Determine significant changes using absolute deltas.

Example:

```c
bool environment_has_changed(
    const EnvironmentalReading *current,
    const EnvironmentalReading *previous)
{
    if (fabsf(current->temperature_c - previous->temperature_c)
            >= TEMP_CHANGE_THRESHOLD_C)
    {
        return true;
    }

    if (fabsf(current->humidity_percent - previous->humidity_percent)
            >= HUMIDITY_CHANGE_THRESHOLD)
    {
        return true;
    }

    return false;
}
```

Pressure should initially be measured and exposed over BLE but does not need to trigger ACTIVE state unless explicitly enabled later.

---

# BME280 Operating Strategy

Use the BME280 in:

**forced mode**

rather than continuous normal mode.

Expected measurement cycle:

```text
RTC / timer wakes MCU
        │
        ▼
trigger BME280 forced measurement
        │
        ▼
wait until conversion finishes
        │
        ▼
read T / RH / pressure
        │
        ▼
BME280 automatically returns to sleep
        │
        ▼
process measurement
        │
        ▼
MCU returns to low-power state
```

Do not leave the BME280 continuously sampling unless required for debugging.

Avoid long blocking delays while waiting for BME280 conversion.

Prefer:

- scheduled timer
- short sleep
- asynchronous state transition

over:

```c
HAL_Delay(...)
```

where practical.

---

# Sensor Driver Separation

The BME280 driver must not contain application policy.

Preferred API:

```c
bool bme280_init(void);

bool bme280_start_measurement(void);

bool bme280_measurement_ready(void);

bool bme280_read(EnvironmentalReading *reading);
```

Do not place:

- BLE logic
- OLED logic
- adaptive sampling decisions

inside the BME280 driver.

---

# BLE Architecture

The BLE peripheral exposes environmental data.

At minimum expose:

- temperature
- humidity
- pressure
- battery level if battery measurement becomes available

Prefer standard Bluetooth SIG characteristics where practical.

A custom Environmental Monitor service may be used for project-specific information.

---

# BLE Advertising

When disconnected, the device should remain discoverable without aggressively advertising.

Initial target:

```text
advertising interval ≈ 1000 ms
```

Advertising interval must be configurable.

If practical, include latest temperature and humidity in advertising or service data so a client can obtain basic information without establishing a connection.

Do not increase advertising frequency merely to make sensor sampling faster.

Sensor sampling and BLE advertising are separate concerns.

---

# BLE Connected Mode

A BLE connection must not force the STM32 application core to remain awake.

Allow STM32WB wireless processing to operate independently wherever supported by STM32CubeWB.

Normal connected state should favor a relatively relaxed connection interval.

Example target:

```text
500–1000 ms
```

During ACTIVE or INTERACTIVE operation, firmware may request a faster connection interval such as:

```text
100–250 ms
```

Do not continuously request connection parameter updates.

Connection parameter management should be handled by a dedicated BLE policy module.

---

# BLE Notifications

Do not send BLE notifications every time the sensor is sampled.

Send a notification when:

```text
temperature changed >= notification threshold

OR

humidity changed >= notification threshold

OR

pressure changed >= notification threshold

OR

client explicitly requests a reading
```

Example initial thresholds:

```c
#define BLE_TEMP_NOTIFY_DELTA_C        0.05f
#define BLE_HUMIDITY_NOTIFY_DELTA      0.50f
```

Store the last value sent over BLE separately from the previous sensor measurement.

For example:

```text
previous sensor reading
    !=
last BLE reported reading
```

These represent different concepts.

---

# BLE Connection Must Survive MCU Sleep

Do not disconnect BLE merely because the application enters STOP2.

Low-power behavior should preserve BLE operation whenever the STM32WB architecture and wireless stack support it.

Any change that prevents low-power mode because of BLE should be treated as a power regression and documented.

---

# OLED Strategy

The OLED must **not remain continuously active** under normal battery operation.

Default state:

```text
OLED OFF
```

OLED is primarily an on-demand local user interface.

---

# OLED Wake

Initial wake trigger:

```text
button press
```

On wake:

1. enable OLED power if power switching exists
2. initialize/reinitialize OLED if necessary
3. obtain a fresh sensor reading if existing data is stale
4. render latest values
5. start display timeout

Suggested timeout:

```c
#define OLED_TIMEOUT_MS 10000
```

Subsequent button presses reset the timeout.

---

# OLED Contents

Initial screen:

```text
24.3 °C
58 %RH
1012 hPa
```

Optionally display:

- BLE connected indicator
- battery percentage
- adaptive sampling state

Avoid unnecessary animations.

Avoid continuously refreshing unchanged pixels.

Only redraw the OLED when displayed information actually changes.

---

# OLED Power Abstraction

Implement:

```c
void display_power_on(void);
void display_power_off(void);
bool display_is_powered(void);
```

These functions may initially only send SSD1306 display ON/OFF commands.

Later they may control:

```text
GPIO
 │
 ▼
MOSFET / load switch
 │
 ▼
OLED VCC
```

Application logic must not care which implementation is used.

---

# Button Handling

Button presses should use an interrupt.

Do not continuously poll the button at high frequency.

Implement debouncing.

The interrupt handler must remain minimal.

ISR responsibilities should be limited to:

```text
record event
set flag
wake application
```

Do not:

- update OLED
- access BME280
- perform BLE operations

directly from the ISR.

---

# Low-Power Requirements

The firmware must attempt to enter low-power mode whenever no task is pending.

Primary target:

```text
STOP2
```

Exact STM32WB low-power implementation must follow STM32CubeWB requirements, especially when BLE is active.

Before entering STOP2 verify that:

- no I2C transaction is active
- no sensor conversion requires immediate CPU servicing
- no OLED transaction is active
- BLE stack permits sleep
- required wake sources are configured

Wake sources should include:

- RTC / low-power timer
- BLE event
- user button
- other explicitly configured interrupts

---

# Avoid Busy Waiting

Busy loops are strongly discouraged.

Do not implement:

```c
while (!measurement_ready)
{
}
```

or:

```c
while (HAL_GetTick() - start < timeout)
{
}
```

Prefer state transitions.

For example:

```text
SENSOR_IDLE
    │
    ▼
SENSOR_TRIGGERED
    │
sleep
    ▼
SENSOR_READY
    │
    ▼
SENSOR_READ
```

---

# Application Event Model

Use an event-driven application layer.

Example:

```c
typedef enum
{
    APP_EVENT_NONE            = 0,
    APP_EVENT_SENSOR_TIMER    = 1 << 0,
    APP_EVENT_SENSOR_READY    = 1 << 1,
    APP_EVENT_BUTTON          = 1 << 2,
    APP_EVENT_BLE_CONNECTED   = 1 << 3,
    APP_EVENT_BLE_DISCONNECTED= 1 << 4,
    APP_EVENT_DISPLAY_TIMEOUT = 1 << 5
} AppEvent;
```

Exact implementation may use:

- STM32 sequencer
- bit flags
- task scheduler
- message queue

Prefer STM32CubeWB's existing scheduling mechanisms instead of introducing an RTOS unless an RTOS is already part of the project.

Do **not** introduce FreeRTOS merely for this feature.

---

# Suggested Module Layout

Prefer approximately:

```text
Core/
├── Inc/
│   ├── app_config.h
│   ├── app_environment.h
│   ├── app_power.h
│   ├── app_display.h
│   ├── app_ble_policy.h
│   ├── bme280_driver.h
│   └── oled_driver.h
│
└── Src/
    ├── app_environment.c
    ├── app_power.c
    ├── app_display.c
    ├── app_ble_policy.c
    ├── bme280_driver.c
    └── oled_driver.c
```

If STM32CubeMX generated directories require a different layout, respect the generated project structure.

Do not move generated files unnecessarily.

---

# Responsibility Boundaries

## `bme280_driver`

Responsible for:

- BME280 registers
- compensation formulas
- forced measurements
- raw sensor communication

Not responsible for adaptive policy.

---

## `oled_driver`

Responsible for:

- OLED commands
- framebuffer if required
- drawing primitives
- sending data to display

Not responsible for deciding when the OLED should turn on.

---

## `app_environment`

Responsible for:

- sensor scheduling
- previous/current measurements
- environmental change detection
- STABLE/ACTIVE state transitions

---

## `app_display`

Responsible for:

- display timeout
- deciding when OLED should be awake
- formatting environmental readings
- invoking OLED driver

---

## `app_ble_policy`

Responsible for:

- notification thresholds
- connection parameter policy
- advertisement payload updates
- determining when new environmental values should be published

---

## `app_power`

Responsible for:

- deciding when application may sleep
- low-power entry/exit hooks
- wake-source handling

---

# Main Application Behavior

Conceptually:

```text
BOOT
 │
 ├── initialize MCU
 ├── initialize BLE
 ├── initialize BME280
 ├── initialize OLED
 │
 ▼
take initial measurement
 │
 ▼
update advertising payload
 │
 ▼
OLED OFF
 │
 ▼
STABLE
```

Normal operation:

```text
STABLE
  │
  │ sensor timer
  ▼
measure environment
  │
  ├── insignificant change ────────► STABLE
  │
  └── significant change
              │
              ▼
            ACTIVE
              │
       sample every ~1 sec
              │
              ▼
         environment stable
         for ACTIVE_TIMEOUT
              │
              ▼
            STABLE
```

User interaction:

```text
STABLE / ACTIVE
       │
       │ button
       ▼
   INTERACTIVE
       │
       ├── OLED ON
       ├── fresh values
       └── timeout reset
              │
              ▼
         10 sec timeout
              │
              ▼
          OLED OFF
              │
       ┌──────┴──────┐
       ▼             ▼
     ACTIVE        STABLE
```

---

# Configuration

Centralize tunable values in:

```text
app_config.h
```

Example:

```c
#pragma once

#define SENSOR_STABLE_INTERVAL_MS      20000U
#define SENSOR_ACTIVE_INTERVAL_MS       1000U

#define TEMP_CHANGE_THRESHOLD_C          0.10f
#define HUMIDITY_CHANGE_THRESHOLD        1.00f

#define ACTIVE_STABILITY_TIMEOUT_MS     30000U

#define BLE_TEMP_NOTIFY_DELTA_C           0.05f
#define BLE_HUMIDITY_NOTIFY_DELTA         0.50f

#define OLED_TIMEOUT_MS                 10000U

#define SENSOR_DATA_STALE_MS             5000U
```

Avoid unexplained numeric constants elsewhere.

---

# Timing and Tick Handling

Code must safely handle timer wraparound.

Prefer unsigned subtraction:

```c
if ((uint32_t)(now - last_event) >= timeout)
{
    ...
}
```

Do not write timestamp comparisons that fail when a 32-bit tick counter wraps.

For long low-power intervals, prefer RTC or STM32 low-power timing mechanisms rather than assuming SysTick continues running in STOP2.

---

# Floating Point

STM32WB55 can handle floating-point calculations on the Cortex-M4 application core.

However:

- avoid unnecessary float calculations in high-frequency loops
- do not repeatedly format floats unless OLED/BLE output actually needs updating
- sensor drivers may retain Bosch integer compensation algorithms if convenient

Correctness and maintainability are more important than premature micro-optimization.

---

# Error Handling

Drivers must return explicit errors.

Do not silently ignore:

- I2C NACK
- BME280 not detected
- invalid chip ID
- OLED communication failure
- BLE operation failure

Transient BME280 errors should not crash the application.

Recommended behavior:

```text
measurement fails
       │
       ▼
record error
       │
       ▼
retry on next scheduled cycle
```

Repeated failures may trigger a slower retry schedule.

---

# Logging

UART logging is useful during development but can significantly affect power measurements.

Provide build-time logging control:

```c
#ifdef DEBUG_LOGGING
...
#endif
```

Production / battery tests should be possible with UART logging completely disabled.

Do not wake the MCU solely for debug output.

---

# Power Measurement Support

Add optional instrumentation GPIOs when helpful.

Example:

```c
POWER_TRACE_HIGH();
perform_sensor_measurement();
POWER_TRACE_LOW();
```

This allows measurement with:

- oscilloscope
- logic analyzer
- Nordic Power Profiler
- Joulescope
- STM32 power measurement tools

Instrumentation must compile out in production builds.

---

# Coding Rules

Use:

- fixed-width integer types
- `const` where appropriate
- explicit ownership
- small modules
- descriptive state names
- explicit error handling

Avoid:

- heap allocation during normal operation
- unnecessary dynamic memory
- blocking delays
- infinite polling loops
- duplicated sensor state
- global variables shared across unrelated modules
- continuously refreshing the OLED
- continuously notifying unchanged BLE values

Prefer static allocation.

---

# STM32CubeMX Generated Code

Treat CubeMX-generated sections carefully.

Code that must survive regeneration should be placed inside:

```c
/* USER CODE BEGIN ... */

/* USER CODE END ... */
```

where appropriate.

Do not modify generated initialization code extensively if the same behavior can be implemented through CubeMX configuration or user-code sections.

Do not regenerate the `.ioc` project unless explicitly requested.

Do not upgrade STM32CubeWB automatically.

---

# Implementation Priorities for Codex

When implementing this project, work incrementally.

## Phase 1 — Sensor

Implement:

- BME280 detection
- forced-mode measurement
- temperature
- humidity
- pressure

Verify readings before adding adaptive behavior.

---

## Phase 2 — Basic OLED

Implement:

- OLED initialization
- temperature display
- humidity display
- pressure display

At this phase continuous display is acceptable for debugging.

---

## Phase 3 — Adaptive Sampling

Implement:

- [x] STABLE state
- [x] ACTIVE state
- [x] change detection
- [x] 20-second stable sampling
- [x] 1-second active sampling
- [x] return to stable after timeout

Implemented in `Core/Src/app_env.c`. Sampling is no longer polled from the main
loop: an RTC-backed timer-server timer sets `CFG_TASK_ENV_MEASURE_ID`, a second
timer covers the forced-conversion delay and sets `CFG_TASK_ENV_READ_ID`.
Stability is measured by counting consecutive unchanged 1 Hz samples
(`ACTIVE_STABLE_SAMPLE_COUNT`) rather than by a tick stopwatch, so it stays
correct across STOP2 and across a tick wraparound.

Remaining validation steps (hardware):

1. Flash and verify the BME280 samples once after boot, then about every 20 s.
2. Breathe on the sensor and verify sampling switches to about 1 Hz.
3. Leave it alone and verify it returns to 20 s after ~30 s.

---

## Phase 4 — BLE Environmental Data

- [x] temperature exposed
- [x] humidity exposed
- [x] pressure exposed
- [x] notifications, thresholded

Implemented in `Core/Src/app_ble_policy.c`. The `WB_BME280` service's
`bme280_char` carries a 12-byte little-endian payload:

```text
offset 0  int32   temperature, 0.01 degC
offset 4  uint32  pressure, Pa
offset 8  uint32  humidity, 1/1024 %RH
```

CubeMX generates `SizeBme_C = 1`; it is widened to `BLE_ENV_PAYLOAD_LEN` from
the `SVCCTL_InitService1` user section in `custom_stm.c`, before
`aci_gatt_add_char()` reads it. The characteristic is already declared
`CHAR_VALUE_LEN_VARIABLE`.

The last value reported over BLE is kept separately from the previous sensor
reading, so a slow drift still notifies once it crosses
`BLE_TEMP_NOTIFY_DELTA_C_X100` / `BLE_HUMIDITY_NOTIFY_DELTA_X1024` /
`BLE_PRESSURE_NOTIFY_DELTA_PA`, and an unchanged reading never generates
traffic.

While disconnected, the advertising manufacturer-specific field carries the
latest temperature (int16, 0.01 degC) and humidity (uint16, 0.01 %RH) so a
scanner can read the room without connecting.

Remaining validation step (hardware): connect with nRF Connect, subscribe, and
confirm notifications only arrive on meaningful change.

---

## Phase 5 — OLED Sleep

- [x] OLED wake path
- [x] 10-second timeout
- [x] OLED shutdown
- [x] OLED normally off
- [x] button interrupt — **needs a CubeMX change, see below**

`Core/Src/app_display.c` owns the timeout, `DISPLAY_APP_PowerOn()` /
`DISPLAY_APP_PowerOff()` / `DISPLAY_APP_IsPowered()` in `display_app.c` own the
power abstraction. The panel is powered off at the end of `APP_DISPLAY_Init()`
and only relit by `APP_ENV_OnUserInteraction()`. A frame is recomposed only
when a digit that is actually printed changes.

`HAL_GPIO_EXTI_Callback()` in `main.c` is written and debounced, but it is
compiled out until `USER_BUTTON_Pin` exists, because this `.ioc` has no button
pin. **Do not hand-edit the `.ioc`.** In STM32CubeMX:

1. Open `007_ble_lcd_bme280.ioc`, go to **Pinout & Configuration**.
2. Pick a free pin for the button. On this UFQFPN48 build the unassigned GPIOs
   are PA1–PA6, PA8, PA15, PB0–PB3, PB5–PB7 and PE4 (PA0 is taken by
   `ADCx_IN5`, PA13/PA14 are SWD, PB3 is SWO). Check which of them your board
   actually breaks out.
3. Left-click the pin, set its mode to **GPIO_EXTI\<n\>**.
4. **System Core → GPIO**, select that pin:
   - GPIO mode: _External Interrupt Mode with Falling edge trigger detection_
     (use Rising if the button pulls high).
   - Pull-up/Pull-down: _Pull-up_ (for a button to GND).
   - User Label: `USER_BUTTON` — this is what generates `USER_BUTTON_Pin`.
5. **System Core → NVIC**, enable **EXTI line\<n\> interrupt**. Leave the
   preemption priority at or below the RTC wakeup interrupt's.
6. **Project Manager → Code Generator**, keep _Generate peripheral
   initialization as a pair of .c/.h files_ as configured, then **Generate
   Code**.

No further firmware change is needed: the callback is inside
`/* USER CODE BEGIN 4 */` and becomes live as soon as the macro exists.

---

## Phase 6 — STM32 Low Power

- [x] no busy polling left in the main loop
- [x] all scheduling on RTC timers, which keep running in STOP2
- [ ] STOP2 actually enabled — **needs a CubeMX change, see below**
- [ ] current measured on hardware

`main.c`'s `while (1)` now contains nothing but `MX_APPE_Process()`, so every
pass ends in `UTIL_SEQ_Idle()` → `UTIL_LPM_EnterLowPower()`. That path is inert
today: `app_conf.h` has `CFG_LPM_SUPPORTED 0`, and it is force-cleared anyway
because `CFG_DEBUG_TRACE` is 1.

To enable it in STM32CubeMX:

1. **Middleware and Software Packs → STM32_WPAN**.
2. Under the debug/trace settings, set **CFG_DEBUG_BLE_TRACE**,
   **CFG_DEBUG_APP_TRACE** and **CFG_DEBUG_TRACE_FULL** to _Disabled_, and set
   **CFG_HW_USART1_ENABLED** to _Disabled_ if UART logging is not needed.
3. Confirm **Low Power Manager (TINY_LPM)** stays enabled in
   **Middleware → STM32_WPAN → Utilities**.
4. Generate code, then set `CFG_LPM_SUPPORTED` to `1` in `Core/Inc/app_conf.h`
   (line 477). Keep `CFG_DEBUGGER_SUPPORTED` at 1 only while debugging — it
   keeps the debug domain clocked and inflates the measured current.

Then verify, in this order, before claiming anything:

1. RTC wakes the application: sampling cadence unchanged with LPM on.
2. BLE stays connected across sleep cycles.
3. The button wakes the application (needs Phase 5's pin). Any GPIO EXTI line
   wakes the CPU from STOP2 on STM32WB — the dedicated PWR WKUP pins only
   matter for Standby/Shutdown, which this design does not use.
4. Measure the actual current with a Joulescope/PPK across a full
   STABLE → ACTIVE → INTERACTIVE → STABLE cycle.

---

## Phase 7 — BLE Power Optimization

- [x] advertising interval ~1000 ms
- [x] connection interval policy, relaxed vs fast
- [x] slave latency exposed as a tunable
- [x] notification frequency decoupled from sampling
- [ ] responsiveness checked on hardware

`Adv_Request()` hardcodes `CFG_FAST_CONN_ADV_INTERVAL_*`, and those defines sit
outside any CubeMX user section in `app_conf.h`. They are overridden from
`/* USER CODE BEGIN PD */` in `app_ble.c` to `BLE_ADV_INTERVAL_MIN/MAX`, which
survives regeneration.

`APP_BLE_POLICY_OnStateChanged()` issues
`aci_l2cap_connection_parameter_update_req()` once per transition — never
repeatedly — asking for 500–1000 ms in STABLE and 100–250 ms in
ACTIVE/INTERACTIVE.

Slave latency is `BLE_CONN_LATENCY`, currently 0. Raising it is the next lever;
`BLE_CONN_SUPERVISION_TIMEOUT` must stay above
`(1 + latency) * interval_max * 2`.

---

## Phase 8 — OLED Hardware Power Gating

- [x] abstraction in place
- [ ] hardware — the WeAct module wires OLED VCC directly, no load switch

`DISPLAY_APP_PowerOn()` / `DISPLAY_APP_PowerOff()` currently send the SSD1315
display ON/OFF commands. They already contain the rail sequencing, guarded by
`#if defined(OLED_PWR_Pin)`: rail up before the controller is addressed, rail
down after it is blanked. Nothing above the driver knows the difference.

To add the load switch:

1. Wire a P-channel MOSFET or load switch between 3V3 and OLED VCC.
2. In STM32CubeMX, set the gate-driving pin to **GPIO_Output**, _Low_ initial
   level, push-pull, no pull, and give it the User Label `OLED_PWR`.
3. Generate code. The `#if defined(OLED_PWR_Pin)` blocks in `display_app.c`
   become live with no other change.
4. Re-run `ssd1315_basic_init()` after a rail cycle — the controller loses its
   configuration. Today the rail is never cut, so this is deliberately not
   wired up yet; add it to `DISPLAY_APP_PowerOn()` alongside the `#if`.

---

# Definition of Done

The initial implementation is considered successful when:

- BME280 operates using forced measurements
- stable environment is sampled approximately every 20 seconds
- meaningful temperature/humidity change causes approximately 1 Hz sampling
- sampling returns to low rate after conditions stabilize
- BLE remains connectable
- connected clients receive meaningful environmental changes
- unchanged sensor readings do not generate unnecessary BLE traffic
- button wakes the OLED
- OLED automatically turns off after approximately 10 seconds
- MCU enters an STM32WB-compatible low-power mode between events
- BLE remains functional while the application spends most of its time asleep
- no main-loop busy polling is required
- no long `HAL_Delay()` calls are used for application scheduling
- configurable thresholds reside in one configuration location

---

# Power Optimization Rule

When deciding between two implementations, prefer the implementation that allows this sequence:

```text
wake
  ↓
do minimal work
  ↓
schedule next event
  ↓
sleep
```

over:

```text
wake
  ↓
wait
  ↓
poll
  ↓
wait
  ↓
poll
```

---

# Performance Rule

Low power must not make the product feel unresponsive.

When something meaningful happens:

```text
environment changes
user presses button
BLE client interacts
```

the device should temporarily increase its activity.

When nothing is happening, it should aggressively sleep.

This adaptive behavior is the core architectural principle of the project.

---

# Instructions for Codex

Before modifying code:

1. Inspect the existing repository structure.
2. Inspect the `.ioc` file if present.
3. Determine the exact STM32WB55 part number.
4. Identify the STM32CubeWB version.
5. Identify whether BLE middleware is already configured.
6. Identify which I2C peripheral is connected to the BME280/OLED.
7. Identify existing low-power infrastructure.
8. Identify existing scheduler/sequencer infrastructure.
9. Reuse existing drivers where reasonable.
10. Preserve currently working functionality.

Do not assume the repository is empty.

Before introducing a new abstraction, search for an existing equivalent.

For every substantial change:

- keep the change focused
- compile when possible
- fix compiler warnings introduced by the change
- explain hardware assumptions
- avoid unrelated refactoring

If hardware information is missing, choose a reasonable implementation boundary rather than hardcoding an assumption.

For example, if OLED power-control hardware is unknown, implement:

```c
display_power_on();
display_power_off();
```

but leave the underlying implementation suitable for later replacement.

---

# Important Non-Goals

Do not initially add:

- FreeRTOS
- filesystem
- external flash
- Wi-Fi
- cloud connectivity
- complex menu UI
- continuous sensor sampling
- continuous OLED operation
- high-rate BLE streaming

unless explicitly requested.

The first goal is a **small, understandable, measurable low-power BLE environmental monitor**.
