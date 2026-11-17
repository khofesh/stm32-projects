#!/usr/bin/env python3
"""T-16: compare SPI2 frames (forwarded by the L432KC master) with the F3 UART frames.

usage: t16_compare.py --f3 /dev/ttyACM0 --l4 /dev/ttyACM1 [--duration 600] [--pairs-at 300]
"""

import argparse
import struct
import sys
import threading
import time

import serial

FRAME_LEN = 84
SYNC = b"\xaa\x55"
T_DATA, T_STATS = 0x01, 0x10
STATS = [
    "drdy", "missed_drdy", "reads", "ok", "zero", "bad_hdr", "bad_crc", "gaps", "dups",
    "hal_err", "pair_done", "pair_fail", "uart_drop", "max_lat_us", "max_xfer_us",
    "last_seq", "pairs_left",
]
F3_OFF_SKIPPED = 78


def crc16(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if crc & 0x8000 else (crc << 1)
            crc &= 0xFFFF
    return crc


def frames(port, stop):
    buf = bytearray()
    while not stop.is_set():
        buf += port.read(256)
        while True:
            i = buf.find(SYNC)
            if i < 0:
                del buf[:-1]
                break
            if len(buf) - i < FRAME_LEN:
                del buf[:i]
                break
            f = bytes(buf[i:i + FRAME_LEN])
            if f[3] == 78 and struct.unpack_from("<H", f, 82)[0] == crc16(f[2:82]):
                del buf[:i + FRAME_LEN]
                yield f
            else:
                del buf[:i + 1]


class Link:
    def __init__(self):
        self.lock = threading.Lock()
        self.f3 = {}  # seq -> (frame, t)
        self.l4 = {}
        self.match = self.mismatch = self.f3_only = self.l4_only = 0
        self.stats = None
        self.f3_skipped = None
        self.f3_skipped0 = None
        self.last_seq = None
        self.min_seq = None  # frames before it are warm-up, ignored

    def warm(self, seq):
        if self.min_seq is None:
            self.last_seq = seq
            return False
        return ((seq - self.min_seq) & 0xFFFF) < 0x8000

    def pair(self, seq):
        a, b = self.f3.get(seq), self.l4.get(seq)
        if a and b:
            if a[0] == b[0]:
                self.match += 1
            else:
                self.mismatch += 1
                print(f"MISMATCH seq={seq}\n  uart {a[0].hex()}\n  spi  {b[0].hex()}")
            del self.f3[seq], self.l4[seq]

    def expire(self, now, age=5.0):
        for d, attr in ((self.f3, "f3_only"), (self.l4, "l4_only")):
            old = [s for s, (_, t) in d.items() if now - t > age]
            for s in old:
                del d[s]
            setattr(self, attr, getattr(self, attr) + len(old))


def reader(name, port, link, stop):
    for f in frames(port, stop):
        now = time.monotonic()
        seq = struct.unpack_from("<H", f, 4)[0]
        with link.lock:
            if f[2] == T_DATA:
                if not link.warm(seq):
                    continue
                if name == "f3":
                    sk = struct.unpack_from("<H", f, F3_OFF_SKIPPED)[0]
                    if link.f3_skipped0 is None:
                        link.f3_skipped0 = sk
                    link.f3_skipped = sk
                (link.f3 if name == "f3" else link.l4)[seq] = (f, now)
                link.pair(seq)
            elif f[2] == T_STATS and name == "l4":
                link.stats = dict(zip(STATS, struct.unpack_from(f"<{len(STATS)}I", f, 10)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--f3", required=True, help="F3DISCOVERY VCP (UART telemetry)")
    ap.add_argument("--l4", required=True, help="NUCLEO-L432KC VCP (SPI master)")
    ap.add_argument("--baud", type=int, default=460800)
    ap.add_argument("--duration", type=float, default=600.0)
    ap.add_argument("--pairs-at", type=float, default=None, help="send 'p' (100 partial/full pairs) after N s")
    a = ap.parse_args()

    f3 = serial.Serial(a.f3, a.baud, timeout=0.1)
    l4 = serial.Serial(a.l4, a.baud, timeout=0.1)
    link, stop = Link(), threading.Event()
    l4.write(b"fr")
    for n, p in (("f3", f3), ("l4", l4)):
        threading.Thread(target=reader, args=(n, p, link, stop), daemon=True).start()

    t0 = time.monotonic()
    sent_p = False
    try:
        while (now := time.monotonic()) - t0 < a.duration:
            time.sleep(1.0)
            with link.lock:
                if link.min_seq is None and now - t0 >= 2.0 and link.last_seq is not None:
                    link.min_seq = (link.last_seq + 10) & 0xFFFF
            if a.pairs_at is not None and not sent_p and now - t0 >= a.pairs_at:
                l4.write(b"p")
                sent_p = True
            with link.lock:
                link.expire(now)
                s = link.stats or {}
                print(f"{now - t0:6.0f}s match={link.match} mismatch={link.mismatch} "
                      f"spi_only={link.l4_only} uart_only={link.f3_only} | "
                      + " ".join(f"{k}={s[k]}" for k in STATS if k in s), flush=True)
    except KeyboardInterrupt:
        pass
    stop.set()
    time.sleep(0.3)

    with link.lock:
        link.expire(time.monotonic())  # frames younger than 5 s are run-end leftovers, dropped
        s = link.stats or {}
        uart_skips = (link.f3_skipped - link.f3_skipped0) if link.f3_skipped is not None else 0
        checks = {
            "stats received": bool(s),
            "bad_crc == 0": s.get("bad_crc", 1) == 0,
            "bad_hdr == 0": s.get("bad_hdr", 1) == 0,
            "gaps == 0": s.get("gaps", 1) == 0,
            "missed_drdy == 0": s.get("missed_drdy", 1) == 0,
            "hal_err == 0": s.get("hal_err", 1) == 0,
            "mismatch == 0": link.mismatch == 0,
            "match > 0": link.match > 0,
            f"spi_only <= F3 UART skips ({uart_skips})": link.l4_only <= uart_skips,
            "uart_only == 0": link.f3_only == 0,
        }
        if a.pairs_at is not None:
            checks["pair_done >= 100"] = s.get("pair_done", 0) >= 100
            checks["pair_fail == 0"] = s.get("pair_fail", 1) == 0
    print()
    for k, ok in checks.items():
        print(f"{'PASS' if ok else 'FAIL'}  {k}")
    print(f"max DRDY->NSS {s.get('max_lat_us')} us, max NSS low {s.get('max_xfer_us')} us")
    sys.exit(0 if all(checks.values()) else 1)


if __name__ == "__main__":
    main()
