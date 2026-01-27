'''
-----------------------------------------------------------------------
NE-1000 Syringe Pump Control Library Using RS232 Communication Protocol
-----------------------------------------------------------------------

Adapted from: https://blog.darwin-microfluidics.com/how-to-control-your-ne1000-syringe-pump-with-python/

Features:
- Automatic response logging from pump (Basic Mode protocol)
- Human-readable status messages
- Optional text file logging
- Optional CSV logging of signed rates (positive for infusing, negative for withdrawing)
- Status parsing (Infusing, Withdrawing, Stopped, Alarms, etc.)

Usage:
    # Basic usage with console logging
    with SyringePump('/dev/ttyUSB0', DIAMETER_50_ML) as pump:
        pump.start_pump(10, "INF")  # Start infusing at 10 mL/hr

    # With file logging and CSV logging
    with SyringePump('/dev/ttyUSB0', DIAMETER_50_ML,
                     log_file='pump_session.log',
                     csv_log_file='pump_rates.csv') as pump:
        pump.start_pump(10, "WDR")
        # Periodically query rate for CSV logging
        pump.send_command("RAT")

    # Without logging
    with SyringePump('/dev/ttyUSB0', DIAMETER_50_ML, log_responses=False) as pump:
        pump.start_pump(10, "INF")

Command line usage:
    python syringe_pump.py --port /dev/ttyUSB0
    python syringe_pump.py --port /dev/ttyUSB0 --log-file pump_session
    python syringe_pump.py --port /dev/ttyUSB0 --csv-log pump_rates
    python syringe_pump.py --port /dev/ttyUSB0 --no-logging

CSV logging:
    - CSV logs are created when csv_log_file is specified
    - Entries are written in response to RAT commands (queries and rate changes)
    - Format: Timestamp, Signed_Rate_mL_hr
    - Positive rates = Infusing, Negative rates = Withdrawing, Zero = Stopped/Paused
    - Automatically logs when calling start_pump() or change_rate()

'''

import serial
import time
import argparse
import logging
import csv
from datetime import datetime

DELAY_TIME = 0.075
STX = chr(2) # Start-of-text ASCII character
ETX = chr(3) # End-of-text ASCII character

DIAMETER_3_ML = 8.66
DIAMETER_50_ML = 26.59

DIRECTIONS = ["WDR", "INF"]

# Flow rate units - mapping from display format to NE-1000 command format
# Per RS232 manual: UM=µL/min, MM=mL/min, UH=µL/hr, MH=mL/hr
RATE_UNITS_CMD = {'UL/HR': 'UH', 'UL/MN': 'UM', 'ML/HR': 'MH', 'ML/MN': 'MM'}
RATE_UNITS = list(RATE_UNITS_CMD.keys())  # For GUI display

# Syringe size options (diameter in mm)
SYRINGE_SIZES = {
    '3 mL': DIAMETER_3_ML,
    '50 mL': DIAMETER_50_ML,
}

class SyringePump:
    # Open serial port, send data to syringe pump, and set up logs
    def __init__(self, port, diameter, log_responses=True, log_file=None, csv_log_file=None):
        self.port = port
        self.log_responses = log_responses
        self.log_file = log_file
        self.csv_log_file = csv_log_file

        self.diameter = diameter
        self.rate_units = 'MH'

        # CSV logging
        self.csv_file = None
        self.csv_writer = None

        if self.log_responses:
            self._setup_logging()

    def __enter__(self):
        self.ser = serial.Serial(self.port, 19200, parity=serial.PARITY_NONE, bytesize=8, stopbits=1, timeout=None, xonxoff=0, rtscts=0) # data format corresponding to the NE-1000

        # Setup CSV logging if specified
        if self.csv_log_file:
            self.csv_file = open(self.csv_log_file, 'w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow(['Timestamp', 'Signed_Rate_mL_hr'])
            self.csv_file.flush()

        self.reset_pump()
        self.set_diameter(self.diameter)
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.stop_pump()
        self.ser.close()

        # Close CSV file
        if self.csv_file:
            self.csv_file.close()

    def _setup_logging(self):
        """Setup logging configuration for pump responses."""
        self.logger = logging.getLogger('SyringePump')
        self.logger.setLevel(logging.INFO)

        # Avoid duplicate handlers
        if not self.logger.handlers:
            # Console handler
            console_handler = logging.StreamHandler()
            console_handler.setLevel(logging.INFO)
            console_formatter = logging.Formatter('[%(asctime)s] %(message)s', datefmt='%H:%M:%S')
            console_handler.setFormatter(console_formatter)
            self.logger.addHandler(console_handler)

            # File handler if specified
            if self.log_file:
                file_handler = logging.FileHandler(self.log_file)
                file_handler.setLevel(logging.INFO)
                file_formatter = logging.Formatter('[%(asctime)s] %(message)s', datefmt='%Y-%m-%d %H:%M:%S')
                file_handler.setFormatter(file_formatter)
                self.logger.addHandler(file_handler)

    def read_response(self, timeout=1.0):
        """Read and parse response from the pump (Basic Mode)."""
        # Set temporary timeout for reading
        original_timeout = self.ser.timeout
        self.ser.timeout = timeout

        response_bytes = b''

        try:
            # Read until we get STX
            while True:
                byte = self.ser.read(1)
                if not byte:
                    # Timeout - no data received
                    return None
                if byte == STX.encode():
                    break

            # Read until ETX
            while True:
                byte = self.ser.read(1)
                if not byte:
                    # Timeout waiting for ETX
                    return None
                if byte == ETX.encode():
                    break
                response_bytes += byte

            # Decode response
            try:
                response = response_bytes.decode('ascii').strip()
                return response
            except UnicodeDecodeError:
                return response_bytes.hex()

        finally:
            # Restore original timeout
            self.ser.timeout = original_timeout

    def parse_response(self, response):
        """Parse the response to extract address, status, and data.

        Response format: <address><status>[<data>]
        - address: 2 characters (00-99)
        - status: 1 character (I, W, S, P, T, U, X, or A for alarm)
        - data: remaining characters (optional)

        Example: '00S' = address 00, status S (stopped), no data
        Example: '00W20.00MH' = address 00, status W (withdrawing), data '20.00MH'
        """
        if not response:
            return None, None, None

        # Address is first 2 characters
        if len(response) < 2:
            return None, None, response

        address = response[0:2]

        # Status is 3rd character (or alarm which is 'A?X' format)
        if len(response) < 3:
            return address, None, None

        # Check if it's an alarm (format: A?<type>)
        if response[2] == 'A' and len(response) >= 4 and response[3] == '?':
            status = response[2:5] if len(response) >= 5 else response[2:]  # 'A?R', 'A?S', etc.
            data = response[5:] if len(response) > 5 else None
        else:
            status = response[2]
            data = response[3:] if len(response) > 3 else None

        return address, status, data

    def get_status_description(self, status):
        """Get human-readable description of status code."""
        status_map = {
            'I': 'Infusing',
            'W': 'Withdrawing',
            'S': 'Stopped',
            'P': 'Paused',
            'T': 'Timed Pause',
            'U': 'User Wait',
            'X': 'Purging'
        }

        # Check for alarm (format: 'A?R', 'A?S', etc.)
        if status and len(status) >= 3 and status.startswith('A?'):
            alarm_map = {
                'R': 'Reset (power interrupted)',
                'S': 'Motor stalled',
                'T': 'Communications timeout',
                'E': 'Program error',
                'O': 'Phase out of range'
            }
            alarm_type = status[2]  # Third character is the alarm type
            return f"ALARM: {alarm_map.get(alarm_type, 'Unknown')}"

        return status_map.get(status, f'Unknown ({status})')

    def extract_rate_from_command(self, command, status):
        """Extract signed rate value from a RAT command.

        Args:
            command: The command string (e.g., 'RAT 10 MH' or 'RAT 20')
            status: Status code from pump response ('I', 'W', 'S', etc.)

        Returns:
            float or None: Signed rate (positive for infusing, negative for withdrawing),
                          or None if not a rate-setting command
        """
        parts = command.strip().split()
        if len(parts) >= 2:
            try:
                # Second element should be the rate
                rate = float(parts[1])

                # Apply sign based on pump status
                if status == 'W':  # Withdrawing
                    return -rate
                elif status == 'I':  # Infusing
                    return rate
                else:  # Stopped, paused, etc.
                    return 0.0
            except (ValueError, IndexError):
                return None
        return None

    def extract_signed_rate_from_response(self, status, data):
        """Extract signed rate from RAT query response.

        Args:
            status: Status code ('I', 'W', 'S', etc.)
            data: Data portion of response (e.g., '20.00MH')

        Returns:
            float: Positive for infusing, negative for withdrawing, zero for stopped/paused
        """
        # If stopped, paused, or in other non-active states, rate is zero
        if not status or status in ['S', 'P', 'T', 'U']:
            return 0.0

        # Try to extract rate from data field
        rate = 0.0
        if data:
            try:
                # Data format is typically like "20.00MH"
                # Extract the numeric part (everything before the units)
                rate_str = ''
                for char in data:
                    if char.isdigit() or char == '.':
                        rate_str += char
                    elif rate_str:  # We've hit the units, stop parsing
                        break

                if rate_str:
                    rate = float(rate_str)
            except (ValueError, IndexError):
                rate = 0.0

        # Apply sign based on direction
        if status == 'W':  # Withdrawing
            return -rate
        elif status == 'I':  # Infusing
            return rate
        else:
            return 0.0

    # Write a command to the syringe pump
    def send_command(self, command):
        # Flush any pending data in the input buffer
        self.ser.reset_input_buffer()

        # Send command
        self.ser.write(command.encode() + b'\r\n')

        # Small delay to let pump process (pump needs time before responding)
        time.sleep(DELAY_TIME)

        # Read and log response
        response = self.read_response()

        if response:
            address, status, data = self.parse_response(response)

            # Track direction from DIR commands for CSV logging
            if command.strip().upper().startswith('DIR'):
                parts = command.strip().upper().split()
                if len(parts) >= 2:
                    self._last_dir_command = parts[1]  # Store 'INF' or 'WDR'

            # Log to CSV if this was a RAT command (query or set) and CSV logging is enabled
            if self.csv_writer and command.strip().upper().startswith('RAT'):
                # Try to extract rate from command (for rate-setting commands)
                # Use last direction command if pump is stopped
                effective_status = status
                if status == 'S' and hasattr(self, '_last_dir_command'):
                    # Pump is stopped but we know the direction from the last DIR command
                    effective_status = 'I' if self._last_dir_command == 'INF' else 'W'

                signed_rate = self.extract_rate_from_command(command, effective_status)

                # If not a rate-setting command, extract from response (for queries)
                if signed_rate is None:
                    signed_rate = self.extract_signed_rate_from_response(status, data)

                timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')
                self.csv_writer.writerow([timestamp, signed_rate])
                self.csv_file.flush()

            # Log to text log
            if self.log_responses:
                status_desc = self.get_status_description(status) if status else 'No status'

                log_msg = f"CMD: '{command}' -> Status: {status_desc}"
                if data:
                    log_msg += f", Data: {data}"

                self.logger.info(log_msg)
        else:
            if self.log_responses:
                self.logger.warning(f"CMD: '{command}' -> No response received")

        return response

    # Start the pump
    def start_pump(self, rate, direction, units=None):
        if rate <= 0:
            raise ValueError("rate must be positive")
        if units is not None:
            self.rate_units = units
        self.set_direction(direction)
        self.send_command(f"RAT {rate} {self.rate_units}")
        self.send_command("RUN") # RUN to run the pump

    # Stop the pump
    def stop_pump(self):
        self.send_command("STP") # STP to stop the pump

    # Set the syringe diameter
    def set_diameter(self, diameter):
        command = f"DIA {diameter}" # DIA for syringe inside diameter (float)
        self.send_command(command) # set the syringe diameter using the "send_command" function

    # Change the pumping rate
    def change_rate(self, rate):
        if rate <= 0:
            raise ValueError("rate must be positive")
        command = f"RAT {rate}" # RAT for pumping rate - cannot change units during operation
        self.send_command(command)

    # Reset the pump
    def reset_pump(self):
        self.send_command("RESET") # RESET to reset the pump

    def set_direction(self, direction):
        if direction not in DIRECTIONS:
            raise ValueError(f'Direction {direction} not in {DIRECTIONS}')
        self.send_command(f"DIR {direction}")



if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=str,
        help='Serial port for RS232 connection to syringe pump.'
    )
    parser.add_argument('--log-file', type=str, default=None,
        help='Optional log file to save pump responses'
    )
    parser.add_argument('--csv-log', type=str, default=None,
        help='Optional CSV file to log signed rates (from RAT queries)'
    )
    parser.add_argument('--no-logging', action='store_true',
        help='Disable response logging'
    )
    args = parser.parse_args()

    # Set up log file
    log_file = None
    if args.log_file and not args.no_logging:
        # Use provided log file name with timestamp
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        log_file = f'{args.log_file}_{timestamp}.log' if not args.log_file.endswith('.log') else args.log_file

    # Set up CSV log file
    csv_log_file = None
    if args.csv_log:
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        csv_log_file = f'{args.csv_log}_{timestamp}.csv' if not args.csv_log.endswith('.csv') else args.csv_log

    with SyringePump(args.port, DIAMETER_50_ML, log_responses=not args.no_logging, log_file=log_file, csv_log_file=csv_log_file) as pump:
        time.sleep(1)
        pump.start_pump(10, "WDR")
        time.sleep(1)
        pump.change_rate(20)
        time.sleep(1)
        pump.change_rate(5)
        time.sleep(1)
        
        while True:
            pump.send_command("RAT")
            time.sleep(0.5)

