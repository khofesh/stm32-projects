#!/usr/bin/env python3
"""Enforce the flash/RAM budget from `arm-none-eabi-size -A` output.

flash = read-only loadable sections (vector table, code, constants, init/fini
        arrays, ARM unwind tables) + load copies of .data, .tdata, .ccmram
ram   = .data + .bss + ._user_heap_stack (CCM is not part of the budget)
"""

import argparse
import shutil
import subprocess
import sys

FLASH_SECTIONS = (
    ".isr_vector",
    ".text",
    ".rodata",
    ".init_array",
    ".fini_array",
    ".preinit_array",
    ".ARM.extab",
    ".ARM",
    ".data",
    ".tdata",
    ".ccmram",
)
RAM_SECTIONS = (".data", ".bss", "._user_heap_stack")


def parse_size_output(text):
    sizes = {}
    for line in text.splitlines():
        fields = line.split()
        if len(fields) < 2 or fields[0] == "Total":
            continue
        try:
            sizes[fields[0]] = int(fields[1], 0)
        except ValueError:
            continue
    return sizes


def total(sizes, names):
    return sum(sizes.get(name, 0) for name in names)


def report(label, used, budget):
    pct = 100.0 * used / budget if budget else 0.0
    print(f"{label}: {used:6d} / {budget:6d} B ({pct:5.2f}%)")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--elf", required=True, help="ELF file to measure")
    parser.add_argument("--size-tool", default=None, help="size executable")
    parser.add_argument("--flash-max", type=int, default=65536)
    parser.add_argument("--ram-max", type=int, default=12288)
    args = parser.parse_args()

    size_tool = args.size_tool or shutil.which("arm-none-eabi-size")
    if not size_tool:
        print("ERROR: arm-none-eabi-size not found", file=sys.stderr)
        return 2

    try:
        output = subprocess.run(
            [size_tool, "-A", args.elf],
            check=True,
            capture_output=True,
            text=True,
        ).stdout
    except subprocess.CalledProcessError as exc:
        print(exc.stderr, file=sys.stderr)
        return 2

    sizes = parse_size_output(output)
    flash = total(sizes, FLASH_SECTIONS)
    ram = total(sizes, RAM_SECTIONS)

    report("flash", flash, args.flash_max)
    report("ram  ", ram, args.ram_max)

    overflow = False
    if flash > args.flash_max:
        print(
            f"ERROR: flash {flash} B exceeds budget {args.flash_max} B",
            file=sys.stderr,
        )
        overflow = True
    if ram > args.ram_max:
        print(
            f"ERROR: ram {ram} B exceeds budget {args.ram_max} B",
            file=sys.stderr,
        )
        overflow = True
    return 1 if overflow else 0


if __name__ == "__main__":
    sys.exit(main())
