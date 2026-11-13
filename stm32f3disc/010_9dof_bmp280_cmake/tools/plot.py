#!/usr/bin/env python3
"""Plot a run captured with tools/log.py.

Shows attitude (roll/pitch/yaw from the quaternion), filtered relative altitude
and body rates. Requires matplotlib (and numpy, pulled in by matplotlib).

Examples:
    tools/plot.py run.csv
    tools/plot.py run.csv --save run.png --no-show
"""

import argparse
import csv
import math
import sys


def load(path):
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        rows = list(reader)
    if not rows:
        sys.exit(f"ERROR: {path} contains no data frames")
    return rows


def attitude(qw, qx, qy, qz):
    roll = math.atan2(
        2.0 * (qw * qx + qy * qz),
        1.0 - 2.0 * (qx * qx + qy * qy),
    )
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (qw * qy - qz * qx))))
    yaw = math.atan2(
        2.0 * (qw * qz + qx * qy),
        1.0 - 2.0 * (qy * qy + qz * qz),
    )
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", help="CSV written by tools/log.py")
    parser.add_argument("--save", help="write the figure to this path")
    parser.add_argument("--no-show", action="store_true", help="do not open a window")
    args = parser.parse_args()

    try:
        import matplotlib
        if args.no_show or args.save:
            matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        import numpy as np
    except ImportError:
        sys.exit("ERROR: matplotlib is required (pip install matplotlib)")

    rows = load(args.csv)
    t = np.array([float(r["t_ms"]) for r in rows]) * 1e-3
    t -= t[0]

    roll = []
    pitch = []
    yaw = []
    for r in rows:
        a = attitude(
            float(r["qw"]), float(r["qx"]), float(r["qy"]), float(r["qz"])
        )
        roll.append(a[0])
        pitch.append(a[1])
        yaw.append(a[2])
    yaw = np.rad2deg(np.unwrap(np.deg2rad(np.array(yaw))))

    alt = np.array([float(r["altitude_m"]) for r in rows])
    gx = np.rad2deg([float(r["gx"]) for r in rows])
    gy = np.rad2deg([float(r["gy"]) for r in rows])
    gz = np.rad2deg([float(r["gz"]) for r in rows])

    fig, axes = plt.subplots(3, 1, sharex=True, figsize=(11, 8))
    axes[0].plot(t, roll, label="roll")
    axes[0].plot(t, pitch, label="pitch")
    axes[0].plot(t, yaw, label="yaw")
    axes[0].set_ylabel("attitude (deg)")
    axes[0].grid(True)
    axes[0].legend(loc="best")

    axes[1].plot(t, alt, color="tab:green")
    axes[1].set_ylabel("altitude (m)")
    axes[1].grid(True)

    axes[2].plot(t, gx, label="gx")
    axes[2].plot(t, gy, label="gy")
    axes[2].plot(t, gz, label="gz")
    axes[2].set_ylabel("gyro (deg/s)")
    axes[2].set_xlabel("time (s)")
    axes[2].grid(True)
    axes[2].legend(loc="best")

    fig.suptitle(args.csv)
    fig.tight_layout()

    if args.save:
        fig.savefig(args.save, dpi=150)
        print(f"saved {args.save}")
    if not args.no_show:
        plt.show()


if __name__ == "__main__":
    main()
