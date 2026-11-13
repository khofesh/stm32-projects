# Hardware

## Board

| Item | Value |
|---|---|
| Board | STM32F3DISCOVERY, MB1035-F303C-E02 |
| MCU | STM32F303VCT6, LQFP100, 72 MHz, 256 KB flash, 48 KB RAM |
| Accel / mag | LSM303AGR (on-board, I2C1) |
| Gyro | I3G4250D (on-board, SPI1, CS = PE3) |
| Baro | BMP280 breakout (Adafruit), I2C2 |
| Console | USART1 PC4/PC5 → ST-LINK/V2-1 VCP, 460800 8N1 |

### Identity (T-02)

| Device | Register | Expected | Verified |
|---|---|---|---|
| I3G4250D | `WHO_AM_I` `0x0F` | `0xD3` | 2026-11-13 |
| LSM303AGR accel | `WHO_AM_I_A` `0x0F` | `0x33` | 2026-11-13 |
| LSM303AGR mag | `WHO_AM_I_M` `0x4F` | `0x40` | 2026-11-13 |
| BMP280 | `id` `0xD0` | `0x58` | 2026-11-13 |

## BMP280 wiring

Adafruit BMP280 breakout (3.3 V/5 V tolerant, onboard 10 kΩ pull-ups and
regulator). Address `0x77`: the breakout pulls SDO high; do not tie it low.

| Breakout pin | STM32F3DISCOVERY | Note |
|---|---|---|
| VIN | 3V | 3.3 V rail; 5V is also acceptable (onboard regulator) |
| GND | GND | — |
| SCK | PA9 | I2C2_SCL, AF4 open-drain, 400 kHz |
| SDI | PA10 | I2C2_SDA, AF4 open-drain, 400 kHz |
| CSB | 3V | must be high for I2C mode |
| SDO | leave open | high → `0x77` |

I2C2 needs external pull-ups; the Adafruit breakout provides them. Keep the
leads short: 400 kHz with a few centimetres of wire.

## Body frame

Body frame is **FRD**: +X forward (towards the USB connectors), +Y right,
+Z down (through the board, away from the component side). Earth frame is
NED; the fusion output quaternion rotates body → earth.

Each sensor has its own package orientation. `board_axes.c` holds one signed
permutation per sensor:

```
body = M * sensor
```

`M` must be a signed permutation (`axis_map_is_signed_permutation`). Its
determinant may be -1: the accelerometer triad on this board is left-handed.

### Current matrices

Measured by T-03 on MB1035-F303C-E02, 2026-11-13 (gyro assumed to share the
LSM303AGR orientation; confirm with the yaw check in M4).

| Sensor | Matrix | Note |
|---|---|---|
| accel | `{{0,1,0},{-1,0,0},{0,0,-1}}` | det -1 (left-handed triad) |
| gyro | `{{0,1,0},{-1,0,0},{0,0,-1}}` | signs pending yaw check |
| mag | `{{0,1,0},{-1,0,0},{0,0,-1}}` | same package as accel |

Reading the accel matrix: body X = sensor Y, body Y = -sensor X, body Z =
-sensor Z.

Expected body-frame values after a correct matrix, used to check the result:

| Pose (component side up unless noted) | Accel body (m/s²) | Gyro while moving |
|---|---|---|
| Flat on the desk | `(0, 0, -9.81)` | yaw CCW seen from above: `gz < 0` |
| USB end up | `(+9.81, 0, 0)` | pitching USB end up: `gy > 0` |
| USB end down | `(-9.81, 0, 0)` | pitching USB end down: `gy < 0` |
| Right edge down | `(0, -9.81, 0)` | rolling right edge down: `gx > 0` |
| Left edge down | `(0, +9.81, 0)` | rolling left edge down: `gx < 0` |
| Component side down | `(0, 0, +9.81)` | — |

Mag in the northern hemisphere: flat, `mz > 0` (field points down). Exact
axes are checked by yawing the board: the heading must follow the rotation
with no sign inversion.

## T-03 procedure

1. Flash and start the raw logger:

   ```bash
   cmake --preset Debug && cmake --build --preset Debug --target flash
   python tools/log.py --port /dev/ttyACM0 --csv t03.csv
   ```

2. Hold each pose from the table above for 5 s, keeping the board still, in
   this order: flat, USB up, USB down, right down, left down, component down.
   Note the row range of each pose in the CSV (`t_ms` column helps).

3. Average `ax ay az` per pose. The mean is `±9.81` on exactly one sensor
   axis; that axis/sign is the pose's direction. Write each body row from
   the measured sensor axis:

   - flat pose → row Z of `M_accel` is the *negated* measured unit vector;
   - USB-up pose → row X is the measured unit vector;
   - right-down pose → row Y is the negated measured unit vector.

   Check `axis_map_is_signed_permutation(M) != 0`. `det(M)` may be -1.

4. Repeat the moving checks for the gyro and yaw checks for the mag. The mag
   sits in the same package as the accel; if its axes match, reuse
   `M_accel` and confirm with the yaw check.

5. Fill `App/Src/board_axes.c`, rebuild, and re-check the table: flat must
   read `(0, 0, -9.81)`, USB-up `(+9.81, 0, 0)`, right-down `(0, -9.81, 0)`.

### T-03 result (2026-11-13)

Measured means (sensor frame, m/s², board still):

| Pose | ax | ay | az |
|---|---|---|---|
| flat, component up | -1.30 | -0.84 | +9.29 |
| USB end up | -0.51 | +9.59 | +0.10 |
| right edge down | +9.65 | +0.05 | +0.40 |
| left edge down | -10.27 | -0.33 | +0.79 |

The flat pose was about 8° off level, but the dominant axis is unambiguous.
Result: `M_accel = {{0,1,0},{-1,0,0},{0,0,-1}}`, det = -1, so the sensor
triad is left-handed. The gyro mapping was not resolved on the bench (hand
tracking was too coarse); it uses the same matrix until the M4 yaw check.
Mag axes are assumed to match the accel (same package); hard-iron offsets
dominate the raw magnitude and are left to M3.
