#!/usr/bin/env python3
# Copyright 2026 Scott Bezek
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Host-side test support script for programAndTest firmware.

This script communicates with the ESP32 test fixture over serial and handles
firmware upload requests by invoking PlatformIO commands.
"""

import argparse
import csv
import logging
import os
import re
import serial
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

# Commands from ESP32
CMD_PING = ">>PING<<"
CMD_START_UPLOAD = ">>START_FIRMWARE_UPLOAD<<"
CMD_SERIAL_PREFIX = ">>SERIAL:"  # Serial number format: >>SERIAL:AABBCCDDEEFF00112233<<
CMD_TEST_START = ">>TEST_START<<"  # Sent once when the full test sequence begins
CMD_TEST_RESULT_PREFIX = ">>TEST_RESULT:"  # Followed by PASS<< or FAIL:<reason><<

# Responses to ESP32
RESP_ACK = ">>ACK<<"
RESP_SUCCESS = ">>SUCCESS<<"
RESP_FAILURE = ">>FAILURE<<"

LOG_FORMAT = '%(asctime)s - %(levelname)s - %(message)s'

CSV_COLUMNS = [
    "Batch ID",
    "Serial",
    "Start Time (epoch ms)",
    "End Time (epoch ms)",
    "End Time (ISO 8601)",
    "Duration (ms)",
    "Result",
    "Failure Details",
    "Test Host Git Commit",
    "Test Host Startup Time (epoch ms)",
]


class TestHost:
    def __init__(self, port: str, updi_port: str = None, baud: int = 115200, dummy: bool = False,
                 batch_id: str = None):
        self.port = port
        self.updi_port = updi_port
        self.baud = baud
        self.dummy = dummy
        self.serial = None
        self.serial_number = None  # Store the last read serial number

        # Determine paths relative to this script
        self.script_dir = Path(__file__).parent.absolute()
        self.repo_root = self.script_dir.parent.parent
        self.logs_dir = self.script_dir / "logs"

        # CSV result logging
        self.batch_id = batch_id or None
        self.csv_path = self.logs_dir / "results.csv"
        self.startup_time_ms = int(time.time() * 1000)
        self.git_commit = self._get_git_commit()
        self.current_test_start_time_ms = None  # Set when >>TEST_START<< is received

        if self.batch_id:
            self._setup_batch_log_file()

        logging.info(f"Script directory: {self.script_dir}")
        logging.info(f"Repository root: {self.repo_root}")
        logging.info(f"Test host git commit: {self.git_commit}")
        if self.batch_id:
            logging.info(f"Batch ID: {self.batch_id} - results will be logged to {self.csv_path}")
        else:
            logging.warning("No Batch ID set - test results will NOT be logged to CSV")
        if self.dummy:
            logging.warning("DUMMY MODE ENABLED - Firmware uploads will be simulated")

    def _setup_batch_log_file(self):
        """Append all INFO-level (and above) log output for this run to a per-batch raw log file."""
        self.logs_dir.mkdir(parents=True, exist_ok=True)
        safe_batch_id = re.sub(r'[^A-Za-z0-9_-]', '_', self.batch_id)
        log_path = self.logs_dir / f"batch_{safe_batch_id}_raw.txt"

        handler = logging.FileHandler(log_path)
        handler.setLevel(logging.INFO)
        handler.setFormatter(logging.Formatter(LOG_FORMAT))
        logging.getLogger().addHandler(handler)

        logging.info(f"Appending log output to {log_path}")

    def _get_git_commit(self) -> str:
        """Get the current git commit hash, with a '-dirty' suffix if this file has local changes."""
        try:
            result = subprocess.run(
                ["git", "rev-parse", "--short", "HEAD"],
                cwd=self.script_dir, capture_output=True, text=True, timeout=5,
            )
            commit = result.stdout.strip()
            if result.returncode != 0 or not commit:
                return "unknown"

            dirty = subprocess.run(
                ["git", "diff", "--quiet", "--", "test_host.py"],
                cwd=self.script_dir, timeout=5,
            ).returncode != 0
            return f"{commit}-dirty" if dirty else commit
        except Exception as e:
            logging.warning(f"Failed to determine git commit: {e}")
            return "unknown"

    def record_test_result(self, end_time_ms: int, result: str, failure_details: str):
        """Append a completed test's result to the CSV, if a batch ID and serial number are known."""
        if not self.batch_id:
            logging.debug("Not logging test result to CSV: no Batch ID set")
            return
        if not self.serial_number:
            logging.warning("Not logging test result to CSV: serial number is not known for this test")
            return
        if self.current_test_start_time_ms is None:
            logging.warning("Not logging test result to CSV: no recorded start time for this test")
            return

        start_time_ms = self.current_test_start_time_ms
        duration_ms = end_time_ms - start_time_ms
        end_time_iso = datetime.fromtimestamp(end_time_ms / 1000).astimezone().isoformat()

        self.logs_dir.mkdir(parents=True, exist_ok=True)
        write_header = not self.csv_path.exists()
        with open(self.csv_path, 'a', newline='') as f:
            writer = csv.writer(f)
            if write_header:
                writer.writerow(CSV_COLUMNS)
            writer.writerow([
                self.batch_id,
                self.serial_number,
                start_time_ms,
                end_time_ms,
                end_time_iso,
                duration_ms,
                result,
                failure_details,
                self.git_commit,
                self.startup_time_ms,
            ])

        logging.info(f"Logged test result to {self.csv_path}: {result} (serial={self.serial_number})")
        self.current_test_start_time_ms = None

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
            logging.info("DUMMY MODE: Simulating firmware upload (4 second delay)")
            time.sleep(4)
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
            pio_cmd = "pio run -e fader_buddy --target upload --verbose --upload-port /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"

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

        elif CMD_TEST_START in line:
            logging.info(f"Received: {CMD_TEST_START}")
            # Reset per-test state so a stale serial number from a prior unit can't
            # be attributed to this one if this test fails before re-reading it.
            self.serial_number = None
            self.current_test_start_time_ms = int(time.time() * 1000)

        elif CMD_TEST_RESULT_PREFIX in line:
            # Parse overall test result: >>TEST_RESULT:PASS<<, >>TEST_RESULT:FAIL:<reason><<,
            # or >>TEST_RESULT:CANCELLED:<step><<
            end_time_ms = int(time.time() * 1000)
            try:
                start_idx = line.index(CMD_TEST_RESULT_PREFIX) + len(CMD_TEST_RESULT_PREFIX)
                end_idx = line.index("<<", start_idx)
                payload = line[start_idx:end_idx]
            except (ValueError, IndexError) as e:
                logging.error(f"Failed to parse test result from: {line} - {e}")
                return

            if payload == "PASS":
                logging.info("Received test result: PASS")
                self.record_test_result(end_time_ms, "PASS", "")
            elif payload.startswith("FAIL:"):
                failure_details = payload[len("FAIL:"):]
                logging.info(f"Received test result: FAIL ({failure_details})")
                self.record_test_result(end_time_ms, "FAIL", failure_details)
            elif payload.startswith("CANCELLED:"):
                cancelled_during = payload[len("CANCELLED:"):]
                logging.info(f"Received test result: CANCELLED (during {cancelled_during})")
                self.record_test_result(end_time_ms, "CANCELLED", cancelled_during)
            else:
                logging.warning(f"Unrecognized test result payload: {payload}")

    def run(self):
        """Main loop - listen for commands and respond."""
        logging.info("Test host started. Listening for commands...")

        try:
            while True:
                if self.serial and self.serial.is_open:
                    try:
                        # Read line with timeout (blocking call with 1 second timeout)
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
        format=LOG_FORMAT
    )

    # Ask for a Batch ID up front; leave empty to disable CSV result logging for this run.
    batch_id = input("Batch ID (leave empty to disable CSV result logging): ").strip()

    # Create and run test host
    test_host = TestHost(args.port, args.updi_port, args.baud, args.dummy, batch_id)

    try:
        test_host.connect()
        test_host.run()
    except Exception as e:
        logging.error(f"Fatal error: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
