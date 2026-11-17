# Calibration procedure

Operator procedure for T-04 (calibration) and T-15 (persistence), for
`010_9dof_bmp280_cmake` on the STM32F3DISCOVERY + BMP280.

## 1. Prerequisites

- Firmware built and flashed; `tools/log.py` or any binary-frame reader running
  on USART1 (ST-LINK VCP) at **460800 8N1**.
- Board on a level, non-magnetic surface, away from steel, magnets, motors and
  powered supplies (for the mag step especially).
- Telemetry frames arriving; check the four sensor-OK bits in `status`
  (`gyro`, `accel`, `mag`, `baro`).
- The command channel is the same USART1 link, one ASCII byte per command
  (`G`, `A`, `M`, `B`, `S`, `D`, `R`). Send no line ending; unknown bytes are
  ignored. Frames are not paused during calibration.

At boot the firmware loads the stored parameters and starts a gyro-bias attempt
automatically. It succeeds only if the board is still for the first 2 s;
otherwise the stored bias is kept.

## 2. Commands (SPEC §22.4)

| Byte | Action |
|---|---|
| `G` | Gyro bias: average 200 samples (2 s) |
| `A` | Capture one accelerometer orientation; six poses needed |
| `M` | Start/stop magnetometer min/max capture |
| `B` | Barometer reference pressure `p0`, 50 samples (2 s) |
| `S` | Save the current parameters to flash |
| `D` | Restore defaults in RAM only (`S` to persist) |
| `R` | Software reset |

Calibration results apply immediately. `S` only persists them.

## 3. Gyro bias — `G`

1. Place the board flat and leave it completely still.
2. Send `G`.
3. Wait 2 s. `calib_state` bit 0 is set while the capture runs.
4. On success `status` bit 4 (`gyro calibrated`) is set.
5. If the per-axis standard error of the mean (std / √200) exceeds 0.05 °/s
   (per-sample std ≈ 0.7 °/s) the capture is rejected: `calib_state` bit 7 is
   set and the previous bias and flag are unchanged. Repeat on a rigid,
   vibration-free surface.

Boot auto-calibration performs the same check and only replaces the stored bias
when it passes.

## 4. Accelerometer, six positions — `A` ×6

The firmware calibrates the body-frame axes after the T-03 axis map. Each `A`
captures 100 samples (1 s) and detects the dominant axis and sign; the mean must
lie within 10° of that axis and the per-axis standard deviation must stay below
0.1 m/s² (board still).

1. Place the board in one static orientation with one body axis vertical
   (face up or face down), leave it still, and send `A`.
2. Wait 1 s. `calib_state` bits 6:4 count accepted orientations (0…6).
3. Repeat for six distinct orientations: ±X, ±Y, ±Z. Any order.
4. After the sixth distinct pose the offsets and diagonal scales are solved and
   applied; `status` bit 5 (`accel calibrated`) is set.
5. A repeated pose, a moving capture or a mean more than 10° from an axis is
   rejected without changing the count; `calib_state` bit 7 is set. Retry that
   pose.
6. If the six-pose solve is rejected, bit 7 is set and the sequence resets to
   zero; start again.

Poses must be distinct: the plus and minus orientation of each body axis.

## 5. Magnetometer — `M` start / `M` stop

1. Move the board well away from ferrous objects and magnetic fields.
2. Send `M`; `calib_state` bit 2 is set.
3. Rotate the board slowly through all orientations for **at least 20 s**
   (figure-eight and full 3-D tumbling), covering every axis direction.
4. Send `M` again to stop. `calib_state` bit 2 clears.
5. Acceptance:
   - span on every axis > 30 µT;
   - max span / min span < 1.5.
6. On success the hard-iron centre and diagonal soft-iron scale are applied and
   `status` bit 6 (`mag calibrated`) is set. Otherwise `calib_state` bit 7 is
   set and the previous values remain; repeat the rotation with a longer, more
   complete motion.

Stopping before 20 s (2000 samples) always rejects the capture.

## 6. Barometer reference — `B`

1. Leave the board at the altitude that should read 0 m.
2. Send `B`; `calib_state` bit 3 is set.
3. Wait 2 s (50 samples). Pressure standard deviation must be below 3 Pa.
4. On success `p0` is stored and `status` bit 7 (`baro reference set`) is set;
   otherwise `calib_state` bit 7 is set.

## 7. Save and verify — `S`, T-15

1. Send `S` to erase and program the calibration flash page.
2. The erase stalls the CPU for 20–40 ms. The firmware measures the stall,
   corrects the HAL and scheduler ticks and realigns the task deadlines, so no
   overrun is counted; telemetry has a gap of about one frame period (sequence
   numbers stay contiguous). A failed write sets `calib_state` bit 7.
3. Power-cycle the board and read `status`: bits 4–7 must be set again and the
   data-frame values (gyro at rest, |a|, |m|, pressure) must match.
4. `D` restores defaults in RAM. Power-cycling without `S` must reload the
   previously stored values (T-15).

## 8. `calib_state` encoding

The telemetry field `calib_state` (data frame offset 75) is defined here
because SPEC §23 lists only the field name:

| Bits | Meaning |
|---|---|
| 0 | gyro capture active |
| 1 | accel capture active (one 1 s capture) |
| 2 | mag capture active (between `M` start and `M` stop) |
| 3 | baro capture active |
| 6:4 | number of accepted accel orientations, 0…6 |
| 7 | last attempt rejected (also: command for an absent sensor, failed `S`, boot gyro attempt while moving); cleared by the next command |

`status` bit 9 (`calibration in progress`) is set while any capture runs or
while an accel sequence has accepted 1…5 poses and is waiting for the next `A`.

A finished capture with bits 3:0 clear and bit 7 clear succeeded.

## 9. Storage format

Last 2 KB flash page, `0x0803F800`:

| Offset | Size | Field |
|---|---|---|
| 0 | 4 | magic `0x31434C41` (`"ALC1"`) |
| 4 | 2 | version `2` |
| 6 | 2 | parameter length `68` |
| 8 | 64 | 16 little-endian floats: gyro bias (3), accel bias (3), accel scale (3), mag bias (3), mag scale (3), `p0` (1) |
| 72 | 2 | flags (bit 0 gyro, 1 accel, 2 mag, 3 baro) |
| 74 | 2 | reserved `0` |
| 76 | 2 | CRC16-CCITT-FALSE over bytes 0…75 |

Version 1 records (flags stored as a float) are rejected. Invalid magic,
version, length, CRC or unknown flag bits load defaults (zero bias, unit scale,
`p0` = 0) and clears the calibrated status bits. `S` overwrites the whole page;
the image is kept below 64 KB so it never reaches this page.

## 10. Quick acceptance checklist

| Item | Pass |
|---|---|
| Gyro at rest after `G` | residual < 0.1 °/s |
| Accel, six orientations after `A` ×6 | \|a\| = 9.81 ± 0.05 m/s² |
| Mag after `M` rotation | \|m\| standard deviation < 5 % over a full rotation |
| Baro after `B` | pressure deviation < 3 Pa during capture |
| Persistence after `S` + power-cycle | identical parameters, status bits 4–7 set |
