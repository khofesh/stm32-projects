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
- `tools/cal.py <CMD>` sends one command and prints the result
  (`status`, `calib_state`, |a|, |m|); exit code 1 on reject. `tools/cal.py M 40`
  runs a 40 s magnetometer capture, `tools/cal.py watch` prints the current state.

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

The boot attempt runs on a cold board; the bias shifts as it warms. On
2026-11-18 it left 0.26 °/s on X. Run `G` again after a few minutes powered,
then `S`; residual dropped to < 0.03 °/s.

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
On the F3DISCOVERY these are: flat face up, face down, USB end up, USB end
down, and each long edge up.

Hand-held poses fail the 0.1 m/s² motion limit even when the tilt is only
4–6°. Prop the board against a rigid block, let go, wait ~1 s, then send `A`.
A rejected pose keeps the count; just retry it.

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

After calibration, check |m| at the spot where the board will be used. A clean
field reads 25–65 µT and stays constant as the board rotates. On 2026-11-18 the
rotation spot read 31 µT, while the desk spot next to equipment read ≈ 100 µT:
that is a local disturbance, not a calibration fault, and heading is wrong
there. Keep the USB cable slack during the tumble so it does not limit
coverage.

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

## 11. Example session with `tools/cal.py`

Full T-04 run (2026-11-18). Each command blocks until the capture ends and
prints `status`, `calib_state`, |a| and |m|; exit code 1 means rejected.

```bash
tools/cal.py watch          # sensors OK? status bits 0-3
tools/cal.py A              # repeat per pose until accel_n=6 / status bit 5
tools/cal.py M 40           # start, tumble 40 s, stop; status bit 6, IMU-only clears
tools/cal.py G              # board warm and still; status bit 4
tools/cal.py B              # optional: board at the 0 m reference
tools/cal.py S              # persist
tools/log.py --port /dev/ttyACM0 --csv run.csv --limit 2000   # verify, board still
```

Expected after `S`: `status` `0x80FF` in a Debug build (`0x00FF` Release),
`calib_state` `0x00`, |a| ≈ 9.80 m/s², gyro mean < 0.1 °/s, |m| 25–65 µT.

`tools/log.py` does not flush the serial buffer on open. If another tool just
used the port, drop the first frames (look for a `t_ms` jump) before computing
statistics.
