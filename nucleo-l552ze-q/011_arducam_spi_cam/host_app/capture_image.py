#!/usr/bin/env python3
"""
ArduCAM Image Capture Host Application for STM32

This script communicates with an STM32 board running the ArduCAM firmware
to capture images via serial port.

Usage:
    python capture_image.py --port /dev/ttyACM0 --baud-rate 115200 --save-location ./images --size 640x480

Commands sent to STM32:
    0x00-0x06: Set resolution (320x240 to 2592x1944)
    0x10: Single capture (JPEG)
    0x20: Start video streaming
    0x21: Stop video streaming
    0xD0-0xD2: Set JPEG quality (high/default/low)

Frame format received:
    Start marker: 0xFF 0xAA
    Length: 4 bytes (little-endian)
    Image data: JPEG bytes
    End marker: 0xBB 0xCC
"""

import argparse
import os
import struct
import sys
import time
from datetime import datetime
from pathlib import Path

import serial

# Resolution command mapping
RESOLUTIONS = {
    "320x240": 0x00,
    "640x480": 0x01,
    "1024x768": 0x02,
    "1280x960": 0x03,
    "1600x1200": 0x04,
    "2048x1536": 0x05,
    "2592x1944": 0x06,
}

# Quality command mapping
QUALITIES = {
    "high": 0xD0,
    "default": 0xD1,
    "low": 0xD2,
}

# Camera model (for future expansion)
CAMERA_MODELS = {
    "OV5642": "ov5642",
    "OV2640": "ov2640",
}

# Commands
CMD_SINGLE_CAPTURE = 0x10
CMD_START_STREAMING = 0x20
CMD_STOP_STREAMING = 0x21

# Frame markers
START_MARKER = bytes([0xFF, 0xAA])
END_MARKER = bytes([0xBB, 0xCC])


class ArduCAMCapture:
    """ArduCAM image capture class for STM32 communication."""

    def __init__(self, port: str, baud_rate: int = 115200, timeout: float = 10.0):
        """
        Initialize the ArduCAM capture interface.

        Args:
            port: Serial port (e.g., /dev/ttyACM0, COM3)
            baud_rate: Baud rate for serial communication
            timeout: Read timeout in seconds
        """
        self.port = port
        self.baud_rate = baud_rate
        self.timeout = timeout
        self.serial = None

    def connect(self) -> bool:
        """
        Connect to the STM32 board.

        Returns:
            True if connection successful, False otherwise
        """
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baud_rate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=self.timeout,
            )
            # Wait for board to be ready
            time.sleep(2)
            # Flush any pending data
            self.serial.reset_input_buffer()
            self.serial.reset_output_buffer()
            print(f"Connected to {self.port} at {self.baud_rate} baud")
            return True
        except serial.SerialException as e:
            print(f"Error connecting to {self.port}: {e}")
            return False

    def disconnect(self):
        """Disconnect from the STM32 board."""
        if self.serial and self.serial.is_open:
            self.serial.close()
            print("Disconnected")

    def send_command(self, cmd: int) -> str:
        """
        Send a command byte to the STM32.

        Args:
            cmd: Command byte to send

        Returns:
            Response string from STM32
        """
        if not self.serial or not self.serial.is_open:
            return ""

        self.serial.write(bytes([cmd]))
        time.sleep(0.2)  # Wait for STM32 to process command

        # Read response (ACK message) with timeout
        response = ""
        start_time = time.time()
        while time.time() - start_time < 0.5:  # 500ms timeout for ACK
            if self.serial.in_waiting:
                try:
                    response += self.serial.read(self.serial.in_waiting).decode('utf-8', errors='ignore')
                except:
                    pass
                if "ACK" in response:
                    break
            time.sleep(0.05)

        return response.strip()

    def set_resolution(self, resolution: str) -> bool:
        """
        Set camera resolution.

        Args:
            resolution: Resolution string (e.g., "640x480")

        Returns:
            True if successful
        """
        if resolution not in RESOLUTIONS:
            print(f"Invalid resolution: {resolution}")
            print(f"Valid options: {', '.join(RESOLUTIONS.keys())}")
            return False

        cmd = RESOLUTIONS[resolution]
        response = self.send_command(cmd)
        print(f"Set resolution to {resolution}: {response}")
        time.sleep(1)  # Wait for sensor to stabilize
        return "ACK" in response

    def set_quality(self, quality: str) -> bool:
        """
        Set JPEG quality.

        Args:
            quality: Quality string (high, default, low)

        Returns:
            True if successful
        """
        if quality not in QUALITIES:
            print(f"Invalid quality: {quality}")
            print(f"Valid options: {', '.join(QUALITIES.keys())}")
            return False

        cmd = QUALITIES[quality]
        response = self.send_command(cmd)
        print(f"Set quality to {quality}: {response}")
        return "ACK" in response

    def wait_for_frame(self) -> bytes:
        """
        Wait for and receive a complete image frame.

        Returns:
            JPEG image data or empty bytes on error
        """
        if not self.serial or not self.serial.is_open:
            return b""

        # Wait for start marker
        buffer = b""
        start_time = time.time()

        while time.time() - start_time < self.timeout:
            if self.serial.in_waiting:
                byte = self.serial.read(1)
                buffer += byte

                # Check for start marker
                if len(buffer) >= 2 and buffer[-2:] == START_MARKER:
                    break

                # Keep buffer small while searching
                if len(buffer) > 1024:
                    buffer = buffer[-2:]
        else:
            print("Timeout waiting for start marker")
            return b""

        # Read length (4 bytes, little-endian)
        length_bytes = self.serial.read(4)
        if len(length_bytes) != 4:
            print("Failed to read length")
            return b""

        length = struct.unpack("<I", length_bytes)[0]
        print(f"Receiving image: {length} bytes")

        if length == 0 or length > 10 * 1024 * 1024:  # Max 10MB
            print(f"Invalid length: {length}")
            return b""

        # Read image data
        image_data = b""
        remaining = length
        while remaining > 0:
            chunk_size = min(remaining, 4096)
            chunk = self.serial.read(chunk_size)
            if not chunk:
                print("Timeout reading image data")
                return b""
            image_data += chunk
            remaining -= len(chunk)

        # Read end marker
        end_marker = self.serial.read(2)
        if end_marker != END_MARKER:
            print(f"Invalid end marker: {end_marker.hex()}")
            # Still return data, might be usable
            return image_data

        return image_data

    def capture_single(self, save_path: str) -> bool:
        """
        Capture a single image.

        Args:
            save_path: Path to save the image

        Returns:
            True if successful
        """
        print("Starting single capture...")
        response = self.send_command(CMD_SINGLE_CAPTURE)
        print(f"Response: {response}")

        # Wait for image data
        image_data = self.wait_for_frame()

        if not image_data:
            print("Failed to receive image")
            return False

        # Validate JPEG
        if not self._validate_jpeg(image_data):
            print("Warning: Image may not be valid JPEG")
        else:
            # Trim padding after EOI marker
            image_data = self._trim_jpeg(image_data)

        # Save image
        with open(save_path, "wb") as f:
            f.write(image_data)

        print(f"Image saved to {save_path} ({len(image_data)} bytes)")
        return True

    def start_streaming(self, save_dir: str, max_frames: int = 0) -> int:
        """
        Start video streaming and save frames.

        Args:
            save_dir: Directory to save frames
            max_frames: Maximum frames to capture (0 = unlimited)

        Returns:
            Number of frames captured
        """
        print("Starting video streaming...")
        response = self.send_command(CMD_START_STREAMING)
        print(f"Response: {response}")

        frame_count = 0
        try:
            while max_frames == 0 or frame_count < max_frames:
                image_data = self.wait_for_frame()

                if not image_data:
                    print("Failed to receive frame, retrying...")
                    continue

                # Trim padding after EOI marker
                if self._validate_jpeg(image_data):
                    image_data = self._trim_jpeg(image_data)

                # Generate filename with timestamp
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
                filename = f"frame_{timestamp}.jpg"
                filepath = os.path.join(save_dir, filename)

                with open(filepath, "wb") as f:
                    f.write(image_data)

                frame_count += 1
                print(f"Frame {frame_count}: {filename} ({len(image_data)} bytes)")

        except KeyboardInterrupt:
            print("\nStopping streaming...")

        # Stop streaming
        self.send_command(CMD_STOP_STREAMING)
        print(f"Streaming stopped. Captured {frame_count} frames.")
        return frame_count

    def _validate_jpeg(self, data: bytes) -> bool:
        """
        Validate JPEG data.

        Args:
            data: Image data

        Returns:
            True if valid JPEG
        """
        if len(data) < 4:
            return False

        # Check JPEG markers
        # SOI (Start of Image): 0xFF 0xD8
        # EOI (End of Image): 0xFF 0xD9
        has_soi = data[:2] == b'\xFF\xD8'

        # EOI might not be at the very end due to FIFO padding
        # Search for EOI in the last 256 bytes
        search_region = data[-256:] if len(data) > 256 else data
        has_eoi = b'\xFF\xD9' in search_region

        return has_soi and has_eoi

    def _trim_jpeg(self, data: bytes) -> bytes:
        """
        Trim JPEG data to remove padding after EOI marker.

        Args:
            data: Raw image data

        Returns:
            Trimmed JPEG data
        """
        # Find the last occurrence of EOI marker
        eoi_pos = data.rfind(b'\xFF\xD9')
        if eoi_pos != -1:
            return data[:eoi_pos + 2]
        return data


def main():
    parser = argparse.ArgumentParser(
        description="ArduCAM Image Capture Host Application for STM32",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
    # Capture single image
    python capture_image.py --port /dev/ttyACM0 --size 640x480 --save-location ./images

    # Start video streaming
    python capture_image.py --port /dev/ttyACM0 --mode streaming --save-location ./frames

    # Capture with specific settings
    python capture_image.py --port COM3 --baud-rate 115200 --size 1024x768 --quality high
        """
    )

    parser.add_argument(
        "--port",
        type=str,
        required=True,
        help="Serial port (e.g., /dev/ttyACM0, COM3)"
    )

    parser.add_argument(
        "--baud-rate",
        type=int,
        default=115200,
        help="Baud rate (default: 115200)"
    )

    parser.add_argument(
        "--save-location",
        type=str,
        default="./images",
        help="Directory to save images (default: ./images)"
    )

    parser.add_argument(
        "--size",
        type=str,
        default="640x480",
        choices=list(RESOLUTIONS.keys()),
        help="Image resolution (default: 640x480)"
    )

    parser.add_argument(
        "--quality",
        type=str,
        default="default",
        choices=list(QUALITIES.keys()),
        help="JPEG quality (default: default)"
    )

    parser.add_argument(
        "--camera-model",
        type=str,
        default="OV5642",
        choices=list(CAMERA_MODELS.keys()),
        help="Camera model (default: OV5642)"
    )

    parser.add_argument(
        "--mode",
        type=str,
        default="single",
        choices=["single", "streaming"],
        help="Capture mode (default: single)"
    )

    parser.add_argument(
        "--max-frames",
        type=int,
        default=0,
        help="Maximum frames to capture in streaming mode (0 = unlimited)"
    )

    parser.add_argument(
        "--timeout",
        type=float,
        default=10.0,
        help="Read timeout in seconds (default: 10.0)"
    )

    args = parser.parse_args()

    # Create save directory
    save_dir = Path(args.save_location)
    save_dir.mkdir(parents=True, exist_ok=True)

    # Initialize capture
    capture = ArduCAMCapture(
        port=args.port,
        baud_rate=args.baud_rate,
        timeout=args.timeout
    )

    # Connect
    if not capture.connect():
        sys.exit(1)

    try:
        # Set resolution
        if not capture.set_resolution(args.size):
            print("Warning: Failed to set resolution")

        # Set quality
        if not capture.set_quality(args.quality):
            print("Warning: Failed to set quality")

        # Capture based on mode
        if args.mode == "single":
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"capture_{timestamp}.jpg"
            filepath = save_dir / filename

            if capture.capture_single(str(filepath)):
                print("Capture successful!")
            else:
                print("Capture failed!")
                sys.exit(1)

        elif args.mode == "streaming":
            frame_count = capture.start_streaming(
                str(save_dir),
                max_frames=args.max_frames
            )
            print(f"Captured {frame_count} frames")

    finally:
        capture.disconnect()


if __name__ == "__main__":
    main()
