#!/usr/bin/env python3
"""
Host-side test support script for programAndTest firmware.

This script communicates with the ESP32 test fixture over serial and handles
firmware upload requests by invoking PlatformIO commands.
"""

import argparse
import logging
import os
import re
import serial
import subprocess
import sys
from pathlib import Path

# Commands from ESP32
CMD_PING = ">>PING<<"
CMD_START_UPLOAD = ">>START_FIRMWARE_UPLOAD<<"
CMD_SERIAL_PREFIX = ">>SERIAL:"  # Serial number format: >>SERIAL:AABBCCDDEEFF00112233<<

# Responses to ESP32
RESP_ACK = ">>ACK<<"
RESP_SUCCESS = ">>SUCCESS<<"
RESP_FAILURE = ">>FAILURE<<"


class TestHost:
    def __init__(self, port: str, updi_port: str = None, baud: int = 115200, dummy: bool = False):
        self.port = port
        self.updi_port = updi_port
        self.baud = baud
        self.dummy = dummy
        self.serial = None
        self.serial_number = None  # Store the last read serial number

        # Determine paths relative to this script
        self.script_dir = Path(__file__).parent.absolute()
        self.repo_root = self.script_dir.parent.parent

        logging.info(f"Script directory: {self.script_dir}")
        logging.info(f"Repository root: {self.repo_root}")
        if self.dummy:
            logging.warning("DUMMY MODE ENABLED - Firmware uploads will be simulated")

    def connect(self):
        """Open serial connection to ESP32."""
        try:
            self.serial = serial.Serial(self.port, self.baud, timeout=1)
            logging.info(f"Connected to {self.port} at {self.baud} baud")
        except serial.SerialException as e:
            logging.error(f"Failed to open serial port {self.port}: {e}")
            raise

    def disconnect(self):
        """Close serial connection."""
        if self.serial and self.serial.is_open:
            self.serial.close()
            logging.info("Serial connection closed")

    def send_response(self, response: str):
        """Send a response to the ESP32."""
        if self.serial and self.serial.is_open:
            msg = f"{response}\n"
            self.serial.write(msg.encode('utf-8'))
            self.serial.flush()
            logging.info(f"Sent: {response}")

    def upload_firmware(self) -> bool:
        """
        Upload firmware to ATtiny1616 using PlatformIO.

        Returns:
            True if upload succeeded, False otherwise
        """
        logging.info("Starting firmware upload...")

        # Clear serial number from previous upload
        self.serial_number = None

        # Dummy mode: simulate upload without actually running it
        if self.dummy:
            import time
            logging.info("DUMMY MODE: Simulating firmware upload (1 second delay)")
            time.sleep(1)
            logging.info("DUMMY MODE: Simulated upload complete - SUCCESS")
            return True

        try:
            # PlatformIO Core uses a fixed virtual environment location
            # See: https://docs.platformio.org/en/latest/core/installation/methods/installer-script.html
            home_dir = os.path.expanduser("~")
            pio_venv = os.path.join(home_dir, ".platformio", "penv")
            activate_script = os.path.join(pio_venv, "bin", "activate")

            # Check if PlatformIO venv exists
            if not os.path.exists(activate_script):
                logging.error(f"PlatformIO virtual environment not found at {pio_venv}")
                logging.error("Please ensure PlatformIO is installed correctly")
                return False

            logging.info(f"Using PlatformIO venv: {pio_venv}")

            # Build PIO command
            pio_cmd = "pio run -e motor_fader --target upload --verbose"

            # Override upload port if specified
            if self.updi_port:
                pio_cmd += f" --upload-port {self.updi_port}"
                logging.info(f"Using UPDI port override: {self.updi_port}")

            # Combine into shell command with activation
            full_cmd = f"source {activate_script} && {pio_cmd}"

            logging.info(f"Running: {full_cmd}")

            # Run the command
            result = subprocess.run(
                full_cmd,
                shell=True,
                cwd=self.repo_root,
                capture_output=True,
                text=True,
                timeout=60,  # 60 second timeout for upload
                executable='/bin/bash'
            )

            # Log output
            if result.stdout:
                for line in result.stdout.splitlines():
                    logging.debug(f"PIO stdout: {line}")
            if result.stderr:
                for line in result.stderr.splitlines():
                    logging.debug(f"PIO stderr: {line}")

            # Check for success
            # PlatformIO returns 0 on success
            if result.returncode == 0:
                logging.info("Firmware upload succeeded!")
                return True
            else:
                logging.error(f"Firmware upload failed with return code {result.returncode}")
                return False

        except subprocess.TimeoutExpired:
            logging.error("Firmware upload timed out")
            return False
        except Exception as e:
            logging.error(f"Exception during firmware upload: {e}")
            return False

    def process_command(self, line: str):
        """Process a command received from the ESP32."""
        line = line.strip()

        if CMD_PING in line:
            logging.info(f"Received: {CMD_PING}")
            self.send_response(RESP_ACK)

        elif CMD_START_UPLOAD in line:
            logging.info(f"Received: {CMD_START_UPLOAD}")
            success = self.upload_firmware()
            if success:
                self.send_response(RESP_SUCCESS)
            else:
                self.send_response(RESP_FAILURE)

        elif CMD_SERIAL_PREFIX in line:
            # Parse serial number from message: >>SERIAL:AABBCCDDEEFF00112233<<
            try:
                start_idx = line.index(CMD_SERIAL_PREFIX) + len(CMD_SERIAL_PREFIX)
                end_idx = line.index("<<", start_idx)
                serial_hex = line[start_idx:end_idx]

                # Validate it's 20 hex characters (10 bytes)
                if len(serial_hex) == 20 and all(c in '0123456789ABCDEFabcdef' for c in serial_hex):
                    self.serial_number = serial_hex.upper()
                    logging.info(f"Received serial number: {self.serial_number}")
                    print(f"\n*** SERIAL NUMBER: {self.serial_number} ***\n")
                else:
                    logging.warning(f"Invalid serial number format: {serial_hex}")
            except (ValueError, IndexError) as e:
                logging.error(f"Failed to parse serial number from: {line} - {e}")

    def run(self):
        """Main loop - listen for commands and respond."""
        logging.info("Test host started. Listening for commands...")

        try:
            while True:
                if self.serial and self.serial.is_open:
                    try:
                        # Read line with timeout
                        if self.serial.in_waiting > 0:
                            line = self.serial.readline().decode('utf-8', errors='ignore')
                            if line:
                                # Echo all serial output for debugging
                                logging.debug(f"Serial: {line.rstrip()}")

                                # Check if it's a command
                                if ">>" in line and "<<" in line:
                                    self.process_command(line)
                    except serial.SerialException as e:
                        logging.error(f"Serial error: {e}")
                        break
                    except UnicodeDecodeError:
                        # Ignore decode errors, just continue
                        pass

        except KeyboardInterrupt:
            logging.info("Received keyboard interrupt, shutting down...")
        finally:
            self.disconnect()


def main():
    parser = argparse.ArgumentParser(
        description="Host-side test support for programAndTest firmware"
    )
    parser.add_argument(
        "--port",
        required=True,
        help="Serial port for ESP32 communication (e.g., /dev/ttyUSB0)"
    )
    parser.add_argument(
        "--updi-port",
        help="UPDI programming port override (defaults to platformio.ini setting)"
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=115200,
        help="Serial baud rate (default: 115200)"
    )
    parser.add_argument(
        "-v", "--verbose",
        action="store_true",
        help="Enable verbose debug logging"
    )
    parser.add_argument(
        "--dummy",
        action="store_true",
        help="Dummy mode: simulate firmware uploads without actually running them"
    )

    args = parser.parse_args()

    # Configure logging
    log_level = logging.DEBUG if args.verbose else logging.INFO
    logging.basicConfig(
        level=log_level,
        format='%(asctime)s - %(levelname)s - %(message)s'
    )

    # Create and run test host
    test_host = TestHost(args.port, args.updi_port, args.baud, args.dummy)

    try:
        test_host.connect()
        test_host.run()
    except Exception as e:
        logging.error(f"Fatal error: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
