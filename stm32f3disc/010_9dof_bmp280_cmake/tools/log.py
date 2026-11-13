#!/usr/bin/env python3
"""Decode the binary telemetry frames of SPEC §23 into CSV.

Validates sync, frame length, CRC16-CCITT-FALSE and sequence continuity, then
writes one CSV row per data frame. Debug frames are reported on stdout.

Examples:
    tools/log.py --port /dev/ttyACM0 --csv run.csv
    tools/log.py --file capture.bin --csv run.csv
    tools/log.py --port /dev/ttyACM0 --limit 500
"""

import argparse
import csv
import struct
import sys

FRAME_LEN = 84
SYNC = b"\xAA\x55"
TYPE_DATA = 0x01
TYPE_DEBUG = 0x02

DATA = struct.Struct("<H I 4f 3f 3f 3f f f h H B B H H H")
DEBUG = struct.Struct("<H I 32s H H H H H H")

DATA_COLUMNS = [
    "seq",
    "t_ms",
    "qw",
    "qx",
    "qy",
    "qz",
    "ax",
    "ay",
    "az",
    "gx",
    "gy",
    "gz",
    "mx",
    "my",
    "mz",
    "pressure_pa",
    "altitude_m",
    "temperature_c",
    "status",
    "cpu_load",
    "calib_state",
    "overruns",
    "frames_skipped",
]


def crc16_ccitt(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


class Stats:
    def __init__(self):
        self.data_frames = 0
        self.debug_frames = 0
        self.crc_errors = 0
        self.length_errors = 0
        self.sequence_gaps = 0
        self.resyncs = 0
        self.last_seq = None

    def sequence(self, seq):
        if self.last_seq is not None:
            expected = (self.last_seq + 1) & 0xFFFF
            if seq != expected:
                self.sequence_gaps += (seq - expected) & 0xFFFF
        self.last_seq = seq


def frames(stream):
    """Yield complete candidate frames, resynchronising on the sync word."""
    buf = bytearray()
    stats = Stats()
    while True:
        chunk = stream.read(4096)
        if not chunk:
            return
        buf.extend(chunk)
        while True:
            start = buf.find(SYNC)
            if start < 0:
                keep = 1 if buf.endswith(SYNC[:1]) else 0
                if len(buf) > keep:
                    stats.resyncs += len(buf) - keep
                    del buf[: len(buf) - keep]
                break
            if start > 0:
                stats.resyncs += start
                del buf[:start]
            if len(buf) < FRAME_LEN:
                break
            frame = bytes(buf[:FRAME_LEN])
            declared = frame[3]
            if declared != FRAME_LEN - 6:
                stats.length_errors += 1
                del buf[:2]
                continue
            crc = struct.unpack_from("<H", frame, 82)[0]
            if crc != crc16_ccitt(frame[2:82]):
                stats.crc_errors += 1
                del buf[:2]
                continue
            del buf[:FRAME_LEN]
            yield frame, stats


def open_stream(args):
    if args.port:
        try:
            import serial  # noqa: F401
        except ImportError:
            sys.exit("ERROR: pyserial is required for --port (pip install pyserial)")
        import serial

        return serial.Serial(args.port, args.baud, timeout=1)
    if args.file:
        return open(args.file, "rb")
    return sys.stdin.buffer


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group()
    source.add_argument("--port", help="serial device, e.g. /dev/ttyACM0")
    source.add_argument("--file", help="capture file to decode (default: stdin)")
    parser.add_argument("--baud", type=int, default=460800)
    parser.add_argument("--csv", help="CSV output file (default: stdout)")
    parser.add_argument("--limit", type=int, default=0, help="stop after N data frames")
    parser.add_argument("--quiet", action="store_true", help="no debug-frame report")
    args = parser.parse_args()

    stream = open_stream(args)
    out = open(args.csv, "w", newline="") if args.csv else sys.stdout
    writer = csv.writer(out)
    writer.writerow(DATA_COLUMNS)

    stats = Stats()
    try:
        for frame, stats in frames(stream):
            if frame[2] == TYPE_DATA:
                values = DATA.unpack_from(frame, 4)
                stats.data_frames += 1
                stats.sequence(values[0])
                row = list(values[:17]) + [values[17] / 100.0] + list(values[18:])
                writer.writerow(row)
                if args.limit and stats.data_frames >= args.limit:
                    break
            elif frame[2] == TYPE_DEBUG:
                stats.debug_frames += 1
                if not args.quiet:
                    (seq, t_ms, desc, e1, e2, e3, loop_us, cpu, stack) = DEBUG.unpack_from(
                        frame, 4
                    )
                    build = desc.split(b"\0")[0].decode(errors="replace")
                    print(
                        f"debug seq={seq} t={t_ms} ms build={build} "
                        f"i2c1={e1} i2c2={e2} spi1={e3} max_loop={loop_us} us "
                        f"cpu={cpu / 10.0:.1f}% free_stack={stack} B",
                        file=sys.stderr,
                    )
    except KeyboardInterrupt:
        pass
    finally:
        if args.csv:
            out.close()
        if hasattr(stream, "close"):
            stream.close()

    print(
        f"frames={stats.data_frames} debug={stats.debug_frames} crc_errors={stats.crc_errors} "
        f"length_errors={stats.length_errors} sequence_gaps={stats.sequence_gaps} "
        f"resynced_bytes={stats.resyncs}",
        file=sys.stderr,
    )
    return 0 if stats.crc_errors == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
