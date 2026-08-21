#!/usr/bin/env python3
"""Watch the WeEnv (STM32WB55 + BME280) environmental notifications.

Scans for the device, subscribes to the environmental characteristic and prints
every notification as it arrives. The firmware only notifies on a meaningful
change, so long silences in a still room are the expected behaviour, not a bug
-- the elapsed column makes that easy to see.

    uv run bme280_notify.py                 # connect and watch notifications
    uv run bme280_notify.py --scan-only     # just watch the advertisements
"""

from __future__ import annotations

import argparse
import asyncio
import struct
import sys
from datetime import datetime

from bleak import BleakClient, BleakScanner
from bleak.backends.device import BLEDevice
from bleak.backends.scanner import AdvertisementData

# STM32_WPAN/App/custom_stm.c, COPY_BME280_CHAR_UUID / COPY_WB_BME280_UUID
ENV_CHAR_UUID = "0000fe41-8e22-4541-9d4c-21edae82ed19"
ENV_SERVICE_UUID = "0000fe40-cc7a-482a-984a-7f2ed5b3e58f"

DEFAULT_NAME = "WeEnv"

# Core/Inc/app_config.h
COMPANY_ID = 0x0030
PAYLOAD_LEN = 12


def decode_payload(data: bytes) -> tuple[float, float, float]:
    """Unpack the 12 byte characteristic value.

    offset 0  int32   temperature, 0.01 degC
    offset 4  uint32  pressure, Pa
    offset 8  uint32  humidity, 1/1024 %RH
    """
    if len(data) != PAYLOAD_LEN:
        raise ValueError(f"expected {PAYLOAD_LEN} bytes, got {len(data)}: {data.hex()}")

    raw_t, raw_p, raw_h = struct.unpack("<iII", data)

    return raw_t / 100.0, raw_p / 100.0, raw_h / 1024.0


def decode_adv(manufacturer_data: dict[int, bytes]) -> tuple[float, float] | None:
    """Unpack the advertised temperature and humidity, if present.

    int16 temperature in 0.01 degC, uint16 humidity in 0.01 %RH.
    """
    blob = manufacturer_data.get(COMPANY_ID)
    if blob is None or len(blob) < 4:
        return None

    raw_t, raw_h = struct.unpack("<hH", blob[:4])

    return raw_t / 100.0, raw_h / 100.0


def stamp() -> str:
    return datetime.now().strftime("%H:%M:%S.%f")[:-3]


async def find_device(name: str, address: str | None, timeout: float) -> BLEDevice:
    if address is not None:
        print(f"looking for {address} ...")
        device = await BleakScanner.find_device_by_address(address, timeout=timeout)
    else:
        print(f"scanning for {name!r} (it advertises about once a second, be patient) ...")
        device = await BleakScanner.find_device_by_name(name, timeout=timeout)

    if device is None:
        raise SystemExit(f"not found within {timeout:.0f} s")

    return device


def matches(name: str, device: BLEDevice, advert: AdvertisementData) -> bool:
    """Loose name match.

    BlueZ reports the name in advert.local_name, but only when it survived AD
    parsing; device.name can also come from a stale BlueZ cache. Accept either,
    case-insensitively, so a name that is merely decorated still matches.
    """
    wanted = name.casefold()

    for candidate in (advert.local_name, device.name):
        if candidate and wanted in candidate.casefold():
            return True

    return False


def describe(device: BLEDevice, advert: AdvertisementData) -> str:
    label = advert.local_name or device.name or "(no name)"
    line = f"[{stamp()}] {device.address}  rssi {advert.rssi:4d} dBm  {label}"

    values = decode_adv(advert.manufacturer_data)
    if values is not None:
        temperature, humidity = values
        line += f"   {temperature:6.2f} C  {humidity:5.2f} %RH"
    elif advert.manufacturer_data:
        ids = ", ".join(f"0x{key:04x}" for key in advert.manufacturer_data)
        line += f"   (manufacturer data {ids}, not ours)"

    return line


async def scan_only(name: str, timeout: float, show_all: bool) -> None:
    """Print advertisements without connecting."""
    seen = 0
    others: set[str] = set()

    def on_advert(device: BLEDevice, advert: AdvertisementData) -> None:
        nonlocal seen

        if show_all or matches(name, device, advert):
            seen += 1
            print(describe(device, advert))
        else:
            others.add(device.address)

    async with BleakScanner(on_advert):
        target = "every advertiser" if show_all else repr(name)
        print(f"watching {target} for {timeout:.0f} s, Ctrl-C to stop ...")
        await asyncio.sleep(timeout)

    print(f"\n{seen} advertisement(s) seen")

    if seen == 0 and not show_all:
        print(
            f"nothing matched {name!r}, but {len(others)} other device(s) were "
            "advertising, so the adapter is fine.\n"
            "  - is a phone still connected? this firmware stops advertising "
            "while connected, and accepts one central at a time\n"
            "  - re-run with --all to see every advertiser and check the name"
        )


async def watch_notifications(name: str, address: str | None, timeout: float) -> None:
    device = await find_device(name, address, timeout)
    print(f"found {device.name} at {device.address}")

    last: float | None = None
    count = 0

    def on_notify(_sender, data: bytearray) -> None:
        nonlocal last, count

        try:
            temperature, pressure, humidity = decode_payload(bytes(data))
        except ValueError as exc:
            print(f"[{stamp()}] bad payload: {exc}")
            return

        count += 1
        now = asyncio.get_running_loop().time()
        gap = "        " if last is None else f"+{now - last:6.1f}s"
        last = now

        print(
            f"[{stamp()}] {gap}  "
            f"{temperature:6.2f} C   {pressure:8.2f} hPa   {humidity:5.2f} %RH"
        )

    async with BleakClient(device) as client:
        if client.services.get_characteristic(ENV_CHAR_UUID) is None:
            raise SystemExit(
                f"characteristic {ENV_CHAR_UUID} not found -- wrong device, or the "
                "firmware predates the environmental service.\nservices present: "
                + ", ".join(s.uuid for s in client.services)
            )

        await client.start_notify(ENV_CHAR_UUID, on_notify)
        print("subscribed, Ctrl-C to stop")
        print("silence means nothing moved past the notification threshold\n")

        try:
            while client.is_connected:
                await asyncio.sleep(1.0)
        finally:
            print(f"\n{count} notification(s) received")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--name", default=DEFAULT_NAME, help="advertised name to look for")
    parser.add_argument("--address", default=None, help="connect straight to this BD address")
    parser.add_argument(
        "--timeout",
        type=float,
        default=30.0,
        help="scan timeout in seconds, also the run time of --scan-only",
    )
    parser.add_argument(
        "--scan-only",
        action="store_true",
        help="print advertised temperature and humidity without connecting",
    )
    parser.add_argument(
        "--all",
        action="store_true",
        help="with --scan-only, print every advertiser instead of filtering by name",
    )
    args = parser.parse_args()

    try:
        if args.scan_only:
            asyncio.run(scan_only(args.name, args.timeout, args.all))
        else:
            asyncio.run(watch_notifications(args.name, args.address, args.timeout))
    except KeyboardInterrupt:
        print()

    return 0


if __name__ == "__main__":
    sys.exit(main())
