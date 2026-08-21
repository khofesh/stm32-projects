# AGENTS.md

## Project Goal

Build a battery-powered environmental monitor using:

- STM32WB54
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

- STM32WB54
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

STM32WB54 can handle floating-point calculations on the Cortex-M4 application core.

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

- STABLE state
- ACTIVE state
- change detection
- 20-second stable sampling
- 1-second active sampling
- return to stable after timeout

Validate state transitions through logs.

---

## Phase 4 — BLE Environmental Data

Expose:

- temperature
- humidity
- pressure

Implement notifications.

Do not optimize BLE intervals yet.

---

## Phase 5 — OLED Sleep

Implement:

- button interrupt
- OLED wake
- 10-second timeout
- OLED shutdown

Verify OLED is normally off.

---

## Phase 6 — STM32 Low Power

Implement STOP2 integration.

Verify:

- RTC wakes application
- BLE continues operating correctly
- button wakes application
- sensor scheduling remains correct

Measure actual current.

Do not claim low-power success solely because firmware calls a STOP function.

---

## Phase 7 — BLE Power Optimization

Tune:

- advertising interval
- connection interval
- slave latency if appropriate
- BLE notification frequency

Check that interactive responsiveness remains acceptable.

---

## Phase 8 — OLED Hardware Power Gating

If hardware supports it, add:

```text
STM32 GPIO
    │
    ▼
load switch / MOSFET
    │
    ▼
OLED VCC
```

Implement this behind the existing display power abstraction.

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
3. Determine the exact STM32WB54 part number.
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