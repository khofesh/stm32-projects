#!/usr/bin/env python3
"""
STM32 memory usage analyzer
Parses arm-none-eabi-size output and shows meaningful statistics
"""

import re
import sys

# Common STM32 chip definitions (add more as needed)
CHIPS = {
    "STM32G031F8": {"flash": 64 * 1024, "ram": 8 * 1024},
    "STM32G031F6": {"flash": 32 * 1024, "ram": 8 * 1024},
    "STM32G031F4": {"flash": 16 * 1024, "ram": 8 * 1024},
    "STM32F401CC": {"flash": 256 * 1024, "ram": 64 * 1024},
    "STM32F411CE": {"flash": 512 * 1024, "ram": 128 * 1024},
    "STM32WB55RG": {"flash": 1024 * 1024, "ram": 256 * 1024},
    "STM32H743ZI": {"flash": 2048 * 1024, "ram": 1024 * 1024},
    "STM32L552ZE": {"flash": 512 * 1024, "ram": 256 * 1024},
}

def parse_size_output(text):
    """Parse arm-none-eabi-size output"""
    # Look for the line with numbers: text, data, bss, dec, hex, filename
    pattern = r'^\s*(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s+([0-9a-fA-F]+)\s+(\S+)'
    
    for line in text.strip().split('\n'):
        match = re.match(pattern, line)
        if match:
            return {
                "text": int(match.group(1)),
                "data": int(match.group(2)),
                "bss": int(match.group(3)),
                "total": int(match.group(4)),
                "filename": match.group(6),
            }
    return None

def format_bytes(b):
    """Format bytes in human readable form"""
    if b >= 1024 * 1024:
        return f"{b / (1024 * 1024):.1f} MB"
    elif b >= 1024:
        return f"{b / 1024:.1f} KB"
    return f"{b} B"

def bar_graph(used, total, width=30):
    """Create a simple ASCII bar graph"""
    percent = used / total
    filled = int(width * percent)
    empty = width - filled
    
    # Color based on usage
    if percent > 0.9:
        color = "\033[91m"  # Red
    elif percent > 0.75:
        color = "\033[93m"  # Yellow
    else:
        color = "\033[92m"  # Green
    reset = "\033[0m"
    
    return f"{color}[{'█' * filled}{'░' * empty}]{reset} {percent * 100:5.1f}%"

def analyze(sizes, chip_name):
    """Print analysis for a specific chip"""
    if chip_name not in CHIPS:
        print(f"Unknown chip: {chip_name}")
        print(f"Available: {', '.join(sorted(CHIPS.keys()))}")
        return
    
    chip = CHIPS[chip_name]
    flash_total = chip["flash"]
    ram_total = chip["ram"]
    
    # Calculate usage
    flash_used = sizes["text"] + sizes["data"]
    ram_used = sizes["data"] + sizes["bss"]
    
    print(f"\n{'=' * 60}")
    print(f"  Memory Analysis: {sizes['filename']}")
    print(f"  Target: {chip_name}")
    print(f"{'=' * 60}\n")
    
    print("Section Breakdown:")
    print(f"  .text (code + const)  : {format_bytes(sizes['text']):>10}")
    print(f"  .data (initialized)   : {format_bytes(sizes['data']):>10}")
    print(f"  .bss  (uninitialized) : {format_bytes(sizes['bss']):>10}")
    print()
    
    print("Memory Usage:")
    print(f"  FLASH: {format_bytes(flash_used):>8} / {format_bytes(flash_total):<8} {bar_graph(flash_used, flash_total)}")
    print(f"  RAM:   {format_bytes(ram_used):>8} / {format_bytes(ram_total):<8} {bar_graph(ram_used, ram_total)}")
    print()
    
    print("Remaining:")
    print(f"  FLASH: {format_bytes(flash_total - flash_used)}")
    print(f"  RAM:   {format_bytes(ram_total - ram_used)}")
    print()
    
    # Warnings
    if flash_used / flash_total > 0.9:
        print("⚠️  WARNING: Flash usage > 90%!")
    if ram_used / ram_total > 0.8:
        print("⚠️  WARNING: RAM usage > 80% (watch your stack!)")

def main():
    if len(sys.argv) < 2:
        print("Usage: stm32_size.py <chip_name> [size_output]")
        print("       stm32_size.py --list")
        print()
        print("Examples:")
        print("  arm-none-eabi-size app.elf | ./stm32_size.py STM32G031F8")
        print("  ./stm32_size.py STM32G031F8 '  37632  468  2700  40800  9f60  app.elf'")
        sys.exit(1)
    
    if sys.argv[1] == "--list":
        print("Supported chips:")
        for name, spec in sorted(CHIPS.items()):
            print(f"  {name}: Flash={format_bytes(spec['flash'])}, RAM={format_bytes(spec['ram'])}")
        sys.exit(0)
    
    chip_name = sys.argv[1].upper()
    
    # Get input from argument or stdin
    if len(sys.argv) > 2:
        text = sys.argv[2]
    else:
        text = sys.stdin.read()
    
    sizes = parse_size_output(text)
    if not sizes:
        print("Error: Could not parse size output")
        print("Expected format: text data bss dec hex filename")
        sys.exit(1)
    
    analyze(sizes, chip_name)

if __name__ == "__main__":
    main()