#!/usr/bin/env python3
"""Send one calibration command (docs/calibration.md §2) and report the result.

Flushes stale input, sends the byte, follows calib_state until the capture
ends and prints status, calib_state, |a| and |m|. For M, the script sends the
start byte, prints progress every 5 s and sends the stop byte after HOLD
seconds; rotate the board during that time.

Examples:
    tools/cal.py watch
    tools/cal.py G
    tools/cal.py A          # once per pose, six poses
    tools/cal.py M 40       # 40 s magnetometer capture
    tools/cal.py S
"""

import argparse
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from log import DATA, TYPE_DATA, Stats, frames  # noqa: E402

CMDS = "GAMBSDR"


def data(stream):
    for f in frames(stream, Stats()):
        if f[2] == TYPE_DATA:
            yield DATA.unpack_from(f, 4)


def show(v, tag=""):
    a = math.sqrt(v[6] ** 2 + v[7] ** 2 + v[8] ** 2)
    m = math.sqrt(v[12] ** 2 + v[13] ** 2 + v[14] ** 2)
    cs = v[20]
    print(
        f"{tag}status=0x{v[18]:04X} calib_state=0x{cs:02X} "
        f"(active=0x{cs & 0xF:X} accel_n={(cs >> 4) & 7} rej={cs >> 7}) "
        f"a=({v[6]:.2f},{v[7]:.2f},{v[8]:.2f}) |a|={a:.3f} |m|={m:.1f}"
    )


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("cmd", choices=list(CMDS) + ["watch"])
    p.add_argument("hold", nargs="?", type=float, default=40.0, help="M capture length, s")
    p.add_argument("--port", default="/dev/ttyACM0")
    p.add_argument("--baud", type=int, default=460800)
    p.add_argument("--timeout", type=float, default=5.0, help="max wait for a capture, s")
    a = p.parse_args()

    try:
        import serial
    except ImportError:
        sys.exit("ERROR: pyserial is required (pip install pyserial)")

    s = serial.Serial(a.port, a.baud, timeout=0.2)
    s.reset_input_buffer()
    g = data(s)

    if a.cmd == "watch":
        show(next(g))
        return

    s.write(a.cmd.encode())
    t0 = time.time()
    if a.cmd == "M":
        last = 0.0
        for v in g:
            now = time.time()
            if now - t0 >= a.hold:
                break
            if now - last > 5:
                show(v, f"t={now - t0:4.1f}s ")
                last = now
        s.write(b"M")
        t0 = time.time()

    active = False
    for v in g:
        busy = v[20] & 0xF
        active |= bool(busy)
        if (active and not busy) or time.time() - t0 > a.timeout:
            v = next(g)
            show(v, "result: ")
            sys.exit(1 if v[20] & 0x80 else 0)


if __name__ == "__main__":
    main()
