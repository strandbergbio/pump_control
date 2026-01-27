#!/usr/bin/env python3
"""
Combined Syringe Pump Control and Sensor Data Live Plotting with PID Control
Controls a syringe pump using PID feedback from a serial sensor, displaying both data streams in real-time.
"""

import sys
import time
import threading
import argparse
from datetime import datetime
from collections import deque
import csv

import serial
import serial.tools.list_ports
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import matplotlib.dates as mdates

# Import SyringePump from syringe_pump module
from syringe_pump import SyringePump, DIAMETER_50_ML, DIAMETER_3_ML


class PIDController:
    """PID Controller for closed-loop control."""

    def __init__(self, kp=1.0, ki=0.0, kd=0.0, setpoint=0.0,
                 output_limits=None, sample_time=0.1):
        """
        Initialize PID controller.

        Args:
            kp: Proportional gain
            ki: Integral gain
            kd: Derivative gain
            setpoint: Desired process variable value
            output_limits: Tuple of (min, max) output limits
            sample_time: Minimum time between updates (seconds)
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.output_limits = output_limits or (-float('inf'), float('inf'))
        self.sample_time = sample_time

        # Internal state
        self._integral = 0.0
        self._last_error = 0.0
        self._last_time = None
        self._last_output = 0.0

    def update(self, measurement, current_time=None):
        """
        Calculate PID output based on measurement.

        Args:
            measurement: Current process variable value
            current_time: Current time (optional, uses time.time() if None)

        Returns:
            float: Control output value
        """
        if current_time is None:
            current_time = time.time()

        # Calculate time delta
        if self._last_time is None:
            dt = self.sample_time
        else:
            dt = current_time - self._last_time
            if dt < self.sample_time:
                # Not enough time has passed
                return self._last_output

        # Calculate error
        error = self.setpoint - measurement

        # Proportional term
        p_term = self.kp * error

        # Integral term with anti-windup
        self._integral += error * dt
        # Clamp integral to prevent windup
        max_integral = (self.output_limits[1] - p_term) / self.ki if self.ki != 0 else float('inf')
        min_integral = (self.output_limits[0] - p_term) / self.ki if self.ki != 0 else -float('inf')
        self._integral = max(min_integral, min(max_integral, self._integral))
        i_term = self.ki * self._integral

        # Derivative term
        if dt > 0:
            d_term = self.kd * (error - self._last_error) / dt
        else:
            d_term = 0.0

        # Calculate output
        output = p_term + i_term + d_term

        # Apply output limits
        output = max(self.output_limits[0], min(self.output_limits[1], output))

        # Update state
        self._last_error = error
        self._last_time = current_time
        self._last_output = output

        return output

    def reset(self):
        """Reset controller state."""
        self._integral = 0.0
        self._last_error = 0.0
        self._last_time = None
        self._last_output = 0.0

    def set_setpoint(self, setpoint):
        """Update setpoint."""
        self.setpoint = setpoint


class CombinedPumpSensorPlotter:
    def __init__(self, pump_port, sensor_port, diameter=DIAMETER_50_ML,
                 sensor_baud=9600, max_points=500, use_pid=False,
                 kp=1.0, ki=0.0, kd=0.0, setpoint=0.0,
                 min_rate=0.1, max_rate=50.0, update_interval=0.5):
        """
        Initialize combined pump and sensor plotter.

        Args:
            pump_port: Serial port for syringe pump
            sensor_port: Serial port for sensor data
            diameter: Syringe diameter
            sensor_baud: Baud rate for sensor serial port
            max_points: Maximum number of points to display
            use_pid: Enable PID control
            kp: PID proportional gain
            ki: PID integral gain
            kd: PID derivative gain
            setpoint: PID setpoint (target sensor value)
            min_rate: Minimum pump rate (mL/hr)
            max_rate: Maximum pump rate (mL/hr)
            update_interval: PID update interval (seconds)
        """
        self.pump_port = pump_port
        self.sensor_port = sensor_port
        self.diameter = diameter
        self.sensor_baud = sensor_baud
        self.max_points = max_points
        self.use_pid = use_pid
        self.min_rate = min_rate
        self.max_rate = max_rate
        self.update_interval = update_interval
        self.setpoint = setpoint

        # Data storage for plotting
        self.pump_timestamps = deque(maxlen=max_points)
        self.pump_rates = deque(maxlen=max_points)
        self.sensor_timestamps = deque(maxlen=max_points)
        self.sensor_values = deque(maxlen=max_points)

        # Latest sensor value for PID
        self.latest_sensor_value = None
        self.sensor_value_lock = threading.Lock()

        # Threading control
        self.running = False
        self.data_lock = threading.Lock()

        # Components
        self.pump = None
        self.sensor_connection = None
        self.pump_csv_file = None

        # PID controller
        self.pid = None
        if use_pid:
            self.pid = PIDController(
                kp=kp, ki=ki, kd=kd, setpoint=setpoint,
                output_limits=(-max_rate, max_rate),
                sample_time=update_interval
            )
            print(f"\nPID Control Enabled:")
            print(f"  Kp: {kp}, Ki: {ki}, Kd: {kd}")
            print(f"  Setpoint: {setpoint}")
            print(f"  Rate limits: {min_rate} to {max_rate} mL/hr")
            print(f"  Update interval: {update_interval}s")

        # Setup file names
        timestamp_str = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.pump_csv_filename = f'pump_rates_{timestamp_str}.csv'
        self.sensor_csv_filename = f'sensor_data_{timestamp_str}.csv'

    def setup_sensor(self):
        """Connect to sensor serial port."""
        try:
            self.sensor_connection = serial.Serial(
                port=self.sensor_port,
                baudrate=self.sensor_baud,
                timeout=1
            )
            print(f"Connected to sensor on {self.sensor_port} at {self.sensor_baud} baud")

            # Setup sensor CSV
            self.sensor_csv_file = open(self.sensor_csv_filename, 'w', newline='')
            self.sensor_csv_writer = csv.writer(self.sensor_csv_file)
            self.sensor_csv_writer.writerow(['Timestamp', 'Value'])
            print(f"Logging sensor data to {self.sensor_csv_filename}")

            return True
        except serial.SerialException as e:
            print(f"Error connecting to sensor port: {e}")
            return False

    def setup_pump(self):
        """Initialize syringe pump."""
        try:
            self.pump = SyringePump(
                self.pump_port,
                self.diameter,
                log_responses=True,
                csv_log_file=self.pump_csv_filename
            )
            self.pump.__enter__()  # Initialize pump
            print(f"Pump initialized on {self.pump_port}")
            return True
        except Exception as e:
            print(f"Error initializing pump: {e}")
            return False

    def read_sensor_data(self):
        """Read sensor data in background thread."""
        while self.running:
            try:
                if self.sensor_connection and self.sensor_connection.in_waiting:
                    line = self.sensor_connection.readline().decode('utf-8', errors='ignore').strip()

                    if line:
                        timestamp = datetime.now()
                        try:
                            value = float(line)

                            # Store data
                            with self.data_lock:
                                self.sensor_timestamps.append(timestamp)
                                self.sensor_values.append(value)

                            # Update latest sensor value for PID
                            with self.sensor_value_lock:
                                self.latest_sensor_value = value

                            # Log to CSV
                            self.sensor_csv_writer.writerow([
                                timestamp.strftime('%Y-%m-%d %H:%M:%S.%f'),
                                value
                            ])
                            self.sensor_csv_file.flush()

                        except ValueError:
                            pass  # Ignore non-numeric data

                time.sleep(0.01)
            except Exception as e:
                print(f"Error reading sensor: {e}")
                time.sleep(0.1)

    def read_pump_csv(self):
        """Read pump rate data from CSV file in background thread."""
        # Wait for CSV file to be created
        time.sleep(0.5)

        while self.running:
            try:
                if self.pump_csv_file is None:
                    # Open the pump CSV file for reading
                    try:
                        self.pump_csv_file = open(self.pump_csv_filename, 'r')
                        csv_reader = csv.reader(self.pump_csv_file)
                        next(csv_reader)  # Skip header
                        self.pump_csv_reader = csv_reader
                    except FileNotFoundError:
                        time.sleep(0.1)
                        continue

                # Read new lines from CSV
                line = self.pump_csv_file.readline()
                if line:
                    try:
                        row = next(csv.reader([line]))
                        if len(row) >= 2:
                            timestamp = datetime.strptime(row[0], '%Y-%m-%d %H:%M:%S.%f')
                            rate = float(row[1])

                            with self.data_lock:
                                self.pump_timestamps.append(timestamp)
                                self.pump_rates.append(rate)
                    except (ValueError, StopIteration):
                        pass
                else:
                    time.sleep(0.01)

            except Exception as e:
                print(f"Error reading pump CSV: {e}")
                time.sleep(0.1)

    def update_plot(self, frame):
        """Update both plots."""
        with self.data_lock:
            # Clear both axes
            self.ax1.clear()
            self.ax2.clear()

            # Plot pump rate
            if len(self.pump_timestamps) > 0:
                self.ax1.plot(self.pump_timestamps, self.pump_rates,
                             'r-', linewidth=1.5, label='Pump Rate')
                self.ax1.axhline(y=0, color='k', linestyle='--', alpha=0.3)
                self.ax1.set_ylabel('Pump Rate (mL/hr)', color='r')
                self.ax1.tick_params(axis='y', labelcolor='r')
                self.ax1.grid(True, alpha=0.3)
                self.ax1.legend(loc='upper left')

            # Plot sensor data
            if len(self.sensor_timestamps) > 0:
                self.ax2.plot(self.sensor_timestamps, self.sensor_values,
                             'b-', linewidth=1.5, label='Sensor (PV)')

                # Add setpoint line if PID is enabled
                if self.use_pid:
                    self.ax2.axhline(y=self.setpoint, color='g', linestyle='--',
                                    linewidth=2, label=f'Setpoint ({self.setpoint})')

                self.ax2.set_ylabel('Sensor Value', color='b')
                self.ax2.tick_params(axis='y', labelcolor='b')
                self.ax2.set_xlabel('Time')
                self.ax2.legend(loc='upper right')
                self.ax2.grid(True, alpha=0.3)

                # Format x-axis
                self.ax2.xaxis.set_major_formatter(mdates.DateFormatter('%H:%M:%S'))
                plt.setp(self.ax2.xaxis.get_majorticklabels(), rotation=45, ha='right')

            title = 'Pump Control and Sensor Monitoring'
            if self.use_pid:
                title += ' (PID Control Active)'
            self.fig.suptitle(title)
            self.fig.tight_layout()

    def run_pump_protocol(self):
        """Run pump control protocol."""
        if self.use_pid:
            self.run_pid_control()
        else:
            self.run_manual_protocol()

    def run_manual_protocol(self):
        """Run a manual pump protocol - override this method for custom protocols."""
        print("\nStarting manual pump protocol...")
        print("Override run_manual_protocol() for custom behavior")

        try:
            # Example: ramp up, hold, ramp down
            self.pump.start_pump(10, "WDR")
            time.sleep(5)

            self.pump.change_rate(20)
            time.sleep(5)

            self.pump.change_rate(5)
            time.sleep(5)

            # Query rate periodically
            while self.running:
                self.pump.send_command("RAT")
                time.sleep(0.5)

        except KeyboardInterrupt:
            print("\nProtocol interrupted")

    def run_pid_control(self):
        """Run PID control loop."""
        print("\nStarting PID control...")

        # Wait for initial sensor reading
        print("Waiting for sensor data...")
        while self.running and self.latest_sensor_value is None:
            time.sleep(0.1)

        if not self.running:
            return

        print(f"Initial sensor value: {self.latest_sensor_value}")
        print("PID control active\n")

        current_direction = None
        current_rate = 0.0

        try:
            while self.running:
                # Get latest sensor value
                with self.sensor_value_lock:
                    sensor_value = self.latest_sensor_value

                if sensor_value is None:
                    time.sleep(self.update_interval)
                    continue

                # Calculate PID output
                pid_output = self.pid.update(sensor_value)

                # Determine direction and rate
                if abs(pid_output) < self.min_rate:
                    # Output too small, stop pump
                    if current_rate != 0.0:
                        self.pump.stop_pump()
                        current_rate = 0.0
                        current_direction = None
                        print(f"PV: {sensor_value:.2f}, Error: {self.pid.setpoint - sensor_value:.2f}, Output: {pid_output:.2f} -> STOPPED")
                else:
                    # Determine direction
                    new_direction = "INF" if pid_output > 0 else "WDR"
                    new_rate = abs(pid_output)

                    # Clamp to min/max rates
                    new_rate = max(self.min_rate, min(self.max_rate, new_rate))

                    # Update pump if needed
                    if current_direction != new_direction:
                        # Direction change - need to restart pump
                        if current_rate != 0.0:
                            self.pump.stop_pump()
                        self.pump.start_pump(new_rate, new_direction)
                        current_direction = new_direction
                        current_rate = new_rate
                        print(f"PV: {sensor_value:.2f}, Error: {self.pid.setpoint - sensor_value:.2f}, Output: {pid_output:.2f} -> {new_direction} {new_rate:.2f} mL/hr")
                    elif abs(new_rate - current_rate) > 0.1:
                        # Rate change
                        self.pump.change_rate(new_rate)
                        current_rate = new_rate
                        print(f"PV: {sensor_value:.2f}, Error: {self.pid.setpoint - sensor_value:.2f}, Output: {pid_output:.2f} -> {new_direction} {new_rate:.2f} mL/hr")

                # Query current rate for logging
                self.pump.send_command("RAT")

                # Wait for next update
                time.sleep(self.update_interval)

        except KeyboardInterrupt:
            print("\nPID control interrupted")

    def start(self):
        """Start the combined system."""
        # Setup sensor
        if not self.setup_sensor():
            return

        # Setup pump
        if not self.setup_pump():
            return

        # Start background threads
        self.running = True

        self.sensor_thread = threading.Thread(target=self.read_sensor_data, daemon=True)
        self.sensor_thread.start()

        self.pump_csv_thread = threading.Thread(target=self.read_pump_csv, daemon=True)
        self.pump_csv_thread.start()

        self.pump_protocol_thread = threading.Thread(target=self.run_pump_protocol, daemon=True)
        self.pump_protocol_thread.start()

        # Setup plotting
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, figsize=(12, 8), sharex=True)

        # Create animation
        self.ani = FuncAnimation(
            self.fig,
            self.update_plot,
            interval=100,
            blit=False
        )

        print("\nStarting live plot...")
        print("Close the plot window to stop\n")

        try:
            plt.show()
        except KeyboardInterrupt:
            print("\nStopped by user")
        finally:
            self.stop()

    def stop(self):
        """Stop and clean up."""
        self.running = False

        # Stop threads
        if hasattr(self, 'sensor_thread') and self.sensor_thread.is_alive():
            self.sensor_thread.join(timeout=2)

        if hasattr(self, 'pump_csv_thread') and self.pump_csv_thread.is_alive():
            self.pump_csv_thread.join(timeout=2)

        # Close connections
        if self.sensor_connection and self.sensor_connection.is_open:
            self.sensor_connection.close()
            print("Sensor connection closed")

        if self.pump:
            self.pump.__exit__(None, None, None)
            print("Pump stopped")

        # Close files
        if hasattr(self, 'sensor_csv_file') and self.sensor_csv_file:
            self.sensor_csv_file.close()
            print(f"Sensor data saved to {self.sensor_csv_filename}")

        if self.pump_csv_file:
            self.pump_csv_file.close()
            print(f"Pump data saved to {self.pump_csv_filename}")


def list_serial_ports():
    """List available serial ports."""
    ports = list(serial.tools.list_ports.comports())

    if not ports:
        print("No serial ports found")
        return []

    print("\nAvailable serial ports:")
    for i, port in enumerate(ports, 1):
        print(f"{i}. {port.device}: {port.description}")

    return ports


def main():
    """Main function."""
    parser = argparse.ArgumentParser(
        description='Combined Syringe Pump Control and Sensor Monitoring with PID Control'
    )
    parser.add_argument('--pump-port', type=str, required=True,
                       help='Serial port for syringe pump')
    parser.add_argument('--sensor-port', type=str, required=True,
                       help='Serial port for sensor')
    parser.add_argument('--diameter', type=float, default=DIAMETER_50_ML,
                       help=f'Syringe diameter (default: {DIAMETER_50_ML})')
    parser.add_argument('--sensor-baud', type=int, default=9600,
                       help='Sensor baud rate (default: 9600)')
    parser.add_argument('--max-points', type=int, default=500,
                       help='Max points to display (default: 500)')

    # PID control arguments
    parser.add_argument('--pid', action='store_true',
                       help='Enable PID control')
    parser.add_argument('--kp', type=float, default=1.0,
                       help='PID proportional gain (default: 1.0)')
    parser.add_argument('--ki', type=float, default=0.0,
                       help='PID integral gain (default: 0.0)')
    parser.add_argument('--kd', type=float, default=0.0,
                       help='PID derivative gain (default: 0.0)')
    parser.add_argument('--setpoint', type=float, default=0.0,
                       help='PID setpoint (target sensor value, default: 0.0)')
    parser.add_argument('--min-rate', type=float, default=0.1,
                       help='Minimum pump rate in mL/hr (default: 0.1)')
    parser.add_argument('--max-rate', type=float, default=50.0,
                       help='Maximum pump rate in mL/hr (default: 50.0)')
    parser.add_argument('--update-interval', type=float, default=0.5,
                       help='PID update interval in seconds (default: 0.5)')

    args = parser.parse_args()

    print("=" * 60)
    print("Combined Pump and Sensor Live Plotter")
    if args.pid:
        print("PID Control Mode")
    print("=" * 60)

    plotter = CombinedPumpSensorPlotter(
        pump_port=args.pump_port,
        sensor_port=args.sensor_port,
        diameter=args.diameter,
        sensor_baud=args.sensor_baud,
        max_points=args.max_points,
        use_pid=args.pid,
        kp=args.kp,
        ki=args.ki,
        kd=args.kd,
        setpoint=args.setpoint,
        min_rate=args.min_rate,
        max_rate=args.max_rate,
        update_interval=args.update_interval
    )

    plotter.start()


if __name__ == "__main__":
    # If run with --list-ports, just list ports and exit
    if '--list-ports' in sys.argv:
        list_serial_ports()
    else:
        main()
