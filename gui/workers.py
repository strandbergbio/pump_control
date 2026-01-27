"""
Worker threads for serial communication.
"""

import time
from datetime import datetime
from queue import Queue, Empty
import threading

import serial
from PyQt5.QtCore import QThread, QObject, pyqtSignal

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from syringe_pump import SyringePump


class SensorWorker(QThread):
    """Background thread for reading sensor data from serial port."""

    # Emits (vacuum_pressure, fluid_pressure, timestamp)
    data_received = pyqtSignal(float, float, object)
    error_occurred = pyqtSignal(str)
    connection_lost = pyqtSignal()

    def __init__(self, port, baud_rate=9600, parent=None):
        super().__init__(parent)
        self.port = port
        self.baud_rate = baud_rate
        self.running = False
        self.serial_connection = None

    def run(self):
        """Main thread loop - read and parse sensor data."""
        try:
            self.serial_connection = serial.Serial(
                port=self.port,
                baudrate=self.baud_rate,
                timeout=1
            )
            self.running = True

            while self.running:
                try:
                    if self.serial_connection.in_waiting:
                        line = self.serial_connection.readline().decode('utf-8', errors='ignore').strip()
                        if line:
                            try:
                                vacuum_pressure, fluid_pressure = self.parse_line(line)
                                timestamp = datetime.now()
                                self.data_received.emit(vacuum_pressure, fluid_pressure, timestamp)
                            except ValueError as e:
                                pass  # Ignore malformed lines
                    time.sleep(0.01)
                except serial.SerialException as e:
                    self.error_occurred.emit(f"Serial error: {e}")
                    self.connection_lost.emit()
                    break

        except serial.SerialException as e:
            self.error_occurred.emit(f"Failed to connect to sensor: {e}")
        finally:
            if self.serial_connection and self.serial_connection.is_open:
                self.serial_connection.close()

    def parse_line(self, line):
        """Parse sensor line in format: vacuum_pressure;fluid_pressure"""
        parts = line.strip().split(';')
        if len(parts) == 2:
            return float(parts[0]), float(parts[1])
        raise ValueError(f"Expected format: value1;value2, got: {line}")

    def stop(self):
        """Stop the worker thread."""
        self.running = False


class PumpController(QObject):
    """Controller for a single syringe pump with thread-safe command execution."""

    rate_changed = pyqtSignal(float, str)  # rate, direction
    status_changed = pyqtSignal(str)
    error_occurred = pyqtSignal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.pump = None
        self.port = None
        self.diameter = None
        self.current_rate = 0.0
        self.current_direction = "INF"
        self.is_running = False
        self.lock = threading.Lock()

    def connect(self, port, diameter, csv_log_file=None):
        """Connect to the pump."""
        try:
            with self.lock:
                self.port = port
                self.diameter = diameter
                self.pump = SyringePump(
                    port,
                    diameter,
                    log_responses=True,
                    csv_log_file=csv_log_file
                )
                self.pump.__enter__()
                self.status_changed.emit("Connected")
                return True
        except Exception as e:
            self.error_occurred.emit(f"Failed to connect to pump: {e}")
            return False

    def disconnect(self):
        """Disconnect from the pump."""
        with self.lock:
            if self.pump:
                try:
                    self.pump.__exit__(None, None, None)
                except:
                    pass
                self.pump = None
                self.is_running = False
                self.current_rate = 0.0
                self.status_changed.emit("Disconnected")

    def start_pump(self, rate, direction, units=None):
        """Start the pump at the specified rate, direction, and units.

        If already running in the same direction, just changes the rate.
        If direction changed, stops and restarts with new direction.
        """
        with self.lock:
            if not self.pump:
                self.error_occurred.emit("Pump not connected")
                return False
            try:
                # If already running in the same direction, just change the rate
                if self.is_running and self.current_direction == direction:
                    self.pump.change_rate(rate)
                else:
                    # Need to start fresh (stopped, or direction changed)
                    if self.is_running:
                        self.pump.stop_pump()
                    self.pump.start_pump(rate, direction, units)

                self.current_rate = rate
                self.current_direction = direction
                self.is_running = True
                self.rate_changed.emit(rate, direction)
                self.status_changed.emit(f"Running: {rate} ({direction})")
                return True
            except Exception as e:
                self.error_occurred.emit(f"Failed to start pump: {e}")
                return False

    def stop_pump(self):
        """Stop the pump."""
        with self.lock:
            if not self.pump:
                return
            try:
                self.pump.stop_pump()
                self.is_running = False
                self.current_rate = 0.0
                self.rate_changed.emit(0.0, self.current_direction)
                self.status_changed.emit("Stopped")
            except Exception as e:
                self.error_occurred.emit(f"Failed to stop pump: {e}")

    def change_rate(self, rate):
        """Change the pump rate without changing direction."""
        with self.lock:
            if not self.pump:
                self.error_occurred.emit("Pump not connected")
                return False
            try:
                if self.is_running:
                    self.pump.change_rate(rate)
                else:
                    self.pump.start_pump(rate, self.current_direction)
                    self.is_running = True
                self.current_rate = rate
                self.rate_changed.emit(rate, self.current_direction)
                self.status_changed.emit(f"Running: {rate} ({self.current_direction})")
                return True
            except Exception as e:
                self.error_occurred.emit(f"Failed to change rate: {e}")
                return False

    def set_direction(self, direction):
        """Set the pump direction."""
        with self.lock:
            if not self.pump:
                return
            try:
                # If running, need to stop and restart with new direction
                was_running = self.is_running
                if was_running:
                    self.pump.stop_pump()
                    self.pump.start_pump(self.current_rate, direction)
                self.current_direction = direction
                self.rate_changed.emit(self.current_rate, direction)
            except Exception as e:
                self.error_occurred.emit(f"Failed to set direction: {e}")

    def set_diameter(self, diameter):
        """Update the syringe diameter."""
        with self.lock:
            if not self.pump:
                return
            try:
                self.pump.set_diameter(diameter)
                self.diameter = diameter
            except Exception as e:
                self.error_occurred.emit(f"Failed to set diameter: {e}")

    def get_signed_rate(self):
        """Get the current rate with sign based on direction."""
        with self.lock:
            if not self.is_running:
                return 0.0
            rate = self.current_rate
            if self.current_direction == "WDR":
                rate = -rate
            return rate
