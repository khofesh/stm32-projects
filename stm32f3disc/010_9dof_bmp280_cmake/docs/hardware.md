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

Operator reference for every pose and rotation below: hold the board
component side up, **USB end facing away from you, the board's right edge
on your right-hand side**. "Right edge", "USB end" and "CCW seen from above"
are all relative to that grip.

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

Positive gyro rotations follow the right-hand rule about the FRD axes:
`+gx` = right edge going down, `+gy` = USB end going up, `+gz` = clockwise
seen from above.

Mag: `mz` is positive in the northern hemisphere and negative in the
southern one, and it is weak near the magnetic equator. The mag
procedure (T-03 step 5) therefore uses only the horizontal field and
differences between opposite headings, which cancel hard-iron offsets.

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

   The firmware applies `board_axes` in `imu_task`, so the log is in the body
   frame. To measure a sensor in its own frame, temporarily set that
   sensor's matrix in `App/Src/board_axes.c` to identity
   (`{{1,0,0},{0,1,0},{0,0,1}}`), rebuild and flash.

4. Gyro (I3G4250D: a separate package, measure it on its own; do not copy
   `M_accel`). Gyro matrix at identity. From the flat pose, make each
   rotation below by about 90° in about 1 s, return, and pause 3 s
   between moves:

   | Move | Positive body axis |
   |---|---|
   | roll: right edge down | `+X` |
   | pitch: USB end up | `+Y` |
   | yaw: clockwise seen from above | `+Z` |

   For each move, take the sensor axis with the largest `|g|` peak. Row
   `X`/`Y`/`Z` of `M_gyro` is that axis's unit vector times the peak's sign.
   The other two axes should stay near zero. Check
   `axis_map_is_signed_permutation(M_gyro) != 0` and record `det(M_gyro)`.

5. Mag (LSM303AGR, same package as the accel, but measure it rather than
   assume). Mag matrix at identity. Find magnetic north with a compass
   or phone, keep at least 1 m from steel, laptops and speakers, and
   hold each pose still for 5 s. Average `mx my mz` per pose.

   | Pair | Pose A | Pose B | `A − B` in body frame |
   |---|---|---|---|
   | X | flat, USB end north | flat, USB end south | `+2·Bh` on body X |
   | Y | flat, USB end west | flat, USB end east | `+2·Bh` on body Y |
   | Z | USB end up, component side facing south | USB end up, component side facing north | `+2·Bh` on body Z |

   For each pair, compute `A − B` per sensor axis. Its dominant sensor
   axis and sign give that body row of `M_mag`. Hard-iron offsets cancel
   in the difference, so no calibration is needed, and the method works in
   either hemisphere. Check `axis_map_is_signed_permutation(M_mag) != 0`.

6. Fill `App/Src/board_axes.c` with the measured matrices, rebuild, and
   re-check in the body frame:

   - accel: flat `(0, 0, -9.81)`, USB up `(+9.81, 0, 0)`, USB down
     `(-9.81, 0, 0)`, right down `(0, -9.81, 0)`, left down
     `(0, +9.81, 0)`, component down `(0, 0, +9.81)`;
   - gyro: each step-4 move peaks positive on its own body axis;
   - mag: each step-5 pair gives a positive `A − B` on its own body axis.

   Record the results under "T-03 result".

### T-03 result (2026-11-13)

Measured means (sensor frame, m/s², board still). Grip: USB end facing
away from the operator, the board's right edge on the operator's right.

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
