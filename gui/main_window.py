"""
Main window for the Pump and Sensor Control GUI.
"""

import time
from datetime import datetime

from PyQt5.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QStatusBar, QLabel, QMessageBox, QDialog, QGridLayout,
    QComboBox, QPushButton, QDialogButtonBox, QGroupBox,
    QLineEdit, QFileDialog
)
from PyQt5.QtCore import QTimer, pyqtSlot, Qt

import csv

import serial.tools.list_ports

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from pump_and_sensor_plotter import PIDController
from syringe_pump import RATE_UNITS_CMD

from .widgets import PumpControlPanel
from .plot_widget import LivePlotWidget
from .workers import SensorWorker, PumpController


class SerialPortDialog(QDialog):
    """Modal dialog for selecting serial ports and log file at startup."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Setup")
        self.setModal(True)
        self.setMinimumWidth(500)

        self.sensor_port = None
        self.vacuum_port = None
        self.fluid_port = None
        self.log_file_path = None

        self._setup_ui()
        self.refresh_ports()

    def _get_default_log_path(self):
        """Generate default log file path with timestamp."""
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        logs_dir = os.path.join(os.getcwd(), 'logs')
        return os.path.join(logs_dir, f'{timestamp}.csv')

    def _setup_ui(self):
        layout = QVBoxLayout(self)

        # Serial Ports section
        ports_group = QGroupBox("Serial Ports")
        ports_layout = QGridLayout(ports_group)

        ports_layout.addWidget(QLabel("Sensor Input:"), 0, 0)
        self.sensor_combo = QComboBox()
        ports_layout.addWidget(self.sensor_combo, 0, 1)

        ports_layout.addWidget(QLabel("Vacuum Pump:"), 1, 0)
        self.vacuum_combo = QComboBox()
        ports_layout.addWidget(self.vacuum_combo, 1, 1)

        ports_layout.addWidget(QLabel("Fluid Pump:"), 2, 0)
        self.fluid_combo = QComboBox()
        ports_layout.addWidget(self.fluid_combo, 2, 1)

        # Refresh button
        self.refresh_btn = QPushButton("Refresh Ports")
        self.refresh_btn.clicked.connect(self.refresh_ports)
        ports_layout.addWidget(self.refresh_btn, 3, 0, 1, 2)

        layout.addWidget(ports_group)

        # Logging section
        log_group = QGroupBox("Data Logging")
        log_layout = QGridLayout(log_group)

        log_layout.addWidget(QLabel("Log File:"), 0, 0)
        self.log_path_edit = QLineEdit()
        self.log_path_edit.setText(self._get_default_log_path())
        log_layout.addWidget(self.log_path_edit, 0, 1)

        self.browse_btn = QPushButton("Browse...")
        self.browse_btn.clicked.connect(self._browse_log_file)
        log_layout.addWidget(self.browse_btn, 0, 2)

        layout.addWidget(log_group)

        # Dialog buttons
        button_box = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        button_box.accepted.connect(self._on_accept)
        button_box.rejected.connect(self.reject)
        layout.addWidget(button_box)

    def _browse_log_file(self):
        """Open file dialog to select log file location."""
        current_path = self.log_path_edit.text()
        current_dir = os.path.dirname(current_path) if current_path else os.getcwd()

        file_path, _ = QFileDialog.getSaveFileName(
            self,
            "Select Log File",
            current_path,
            "CSV Files (*.csv);;All Files (*)"
        )
        if file_path:
            if not file_path.endswith('.csv'):
                file_path += '.csv'
            self.log_path_edit.setText(file_path)

    def refresh_ports(self):
        """Refresh the list of available serial ports."""
        ports = list(serial.tools.list_ports.comports())

        for combo in [self.sensor_combo, self.vacuum_combo, self.fluid_combo]:
            current = combo.currentData()
            combo.clear()
            for port in ports:
                combo.addItem(f"{port.device}: {port.description}", port.device)
            # Restore selection if still available
            if current:
                for i in range(combo.count()):
                    if combo.itemData(i) == current:
                        combo.setCurrentIndex(i)
                        break

    def _on_accept(self):
        """Validate and accept the dialog."""
        self.sensor_port = self.sensor_combo.currentData()
        self.vacuum_port = self.vacuum_combo.currentData()
        self.fluid_port = self.fluid_combo.currentData()
        self.log_file_path = self.log_path_edit.text().strip()

        if not all([self.sensor_port, self.vacuum_port, self.fluid_port]):
            QMessageBox.warning(self, "Selection Required",
                              "Please select all three serial ports.")
            return

        if not self.log_file_path:
            QMessageBox.warning(self, "Log File Required",
                              "Please specify a log file path.")
            return

        # Ensure the logs directory exists
        log_dir = os.path.dirname(self.log_file_path)
        if log_dir and not os.path.exists(log_dir):
            try:
                os.makedirs(log_dir)
            except OSError as e:
                QMessageBox.warning(self, "Directory Error",
                                  f"Could not create log directory: {e}")
                return

        self.accept()


class MainWindow(QMainWindow):
    """Main application window for dual pump control."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Dual Pump and Sensor Control")
        self.setMinimumSize(1000, 700)

        # State
        self.connected = False
        self.sensor_worker = None
        self.vacuum_controller = None
        self.fluid_controller = None
        self.vacuum_pid = None
        self.fluid_pid = None
        self.vacuum_pid_enabled = False
        self.fluid_pid_enabled = False

        # Port info for display
        self.vacuum_port_name = ""
        self.fluid_port_name = ""

        # Latest pressure values for PID
        self.latest_vacuum_pressure = None
        self.latest_fluid_pressure = None

        # CSV logging
        self.csv_file = None
        self.csv_writer = None
        self.log_file_path = None

        self._setup_ui()
        self._setup_connections()

        # PID update timer
        self.pid_timer = QTimer(self)
        self.pid_timer.timeout.connect(self._update_pid)
        self.pid_timer.setInterval(500)  # 500ms update interval

        # Show serial port dialog on startup
        QTimer.singleShot(100, self._show_port_dialog)

    def _setup_ui(self):
        """Setup the main UI layout."""
        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        main_layout = QVBoxLayout(central_widget)
        main_layout.setContentsMargins(5, 5, 5, 5)
        main_layout.setSpacing(5)

        # Top section: Pump controls (side by side, compact)
        controls_layout = QHBoxLayout()
        controls_layout.setSpacing(10)

        # Vacuum pump panel - defaults to withdraw direction
        self.vacuum_panel = PumpControlPanel("Vacuum Pump", default_syringe='50 mL', default_units='ML/HR', default_direction='WDR', default_mode='pressure')
        self.vacuum_panel.setEnabled(False)
        self.vacuum_panel.setMaximumHeight(300)
        controls_layout.addWidget(self.vacuum_panel)

        # Fluid pump panel - defaults to infuse direction
        self.fluid_panel = PumpControlPanel("Fluid Pump", default_syringe='3 mL', default_units='UL/MN', default_direction='INF')
        self.fluid_panel.setEnabled(False)
        self.fluid_panel.setMaximumHeight(300)
        controls_layout.addWidget(self.fluid_panel)

        main_layout.addLayout(controls_layout)

        # Bottom section: Live plots (takes most space)
        self.plot_widget = LivePlotWidget()
        main_layout.addWidget(self.plot_widget, stretch=1)

        # Status bar
        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)

        self.connection_label = QLabel("Disconnected")
        self.status_bar.addWidget(self.connection_label)

        self.vacuum_status_label = QLabel("Vacuum: --")
        self.status_bar.addWidget(self.vacuum_status_label)

        self.fluid_status_label = QLabel("Fluid: --")
        self.status_bar.addWidget(self.fluid_status_label)

    def _setup_connections(self):
        """Connect signals and slots."""
        # Vacuum pump panel
        self.vacuum_panel.start_requested.connect(self._on_vacuum_start)
        self.vacuum_panel.stop_requested.connect(self._on_vacuum_stop)
        self.vacuum_panel.syringe_changed.connect(self._on_vacuum_syringe_changed)
        self.vacuum_panel.pid_enabled_changed.connect(self._on_vacuum_pid_enabled)
        self.vacuum_panel.pid_params_changed.connect(self._on_vacuum_pid_params)
        self.vacuum_panel.pid_reset_requested.connect(self._on_vacuum_pid_reset)

        # Fluid pump panel
        self.fluid_panel.start_requested.connect(self._on_fluid_start)
        self.fluid_panel.stop_requested.connect(self._on_fluid_stop)
        self.fluid_panel.syringe_changed.connect(self._on_fluid_syringe_changed)
        self.fluid_panel.pid_enabled_changed.connect(self._on_fluid_pid_enabled)
        self.fluid_panel.pid_params_changed.connect(self._on_fluid_pid_params)
        self.fluid_panel.pid_reset_requested.connect(self._on_fluid_pid_reset)

    def _show_port_dialog(self):
        """Show the serial port selection dialog."""
        dialog = SerialPortDialog(self)
        if dialog.exec_() == QDialog.Accepted:
            self._on_connect(dialog.sensor_port, dialog.vacuum_port, dialog.fluid_port, dialog.log_file_path)
        else:
            # User cancelled - close the application
            self.close()

    def _on_connect(self, sensor_port, vacuum_port, fluid_port, log_file_path=None):
        """Handle connection request."""
        try:
            # Store port names for display
            self.vacuum_port_name = vacuum_port
            self.fluid_port_name = fluid_port

            # Setup CSV logging
            if log_file_path:
                self.log_file_path = log_file_path
                self.csv_file = open(log_file_path, 'w', newline='')
                self.csv_writer = csv.writer(self.csv_file)
                self.csv_writer.writerow([
                    'Timestamp',
                    'Vacuum_Pressure',
                    'Vacuum_Flow_Rate',
                    'Vacuum_Setpoint',
                    'Fluid_Pressure',
                    'Fluid_Flow_Rate',
                    'Fluid_Setpoint'
                ])
                self.csv_file.flush()

            # Update panel titles with port info
            self.vacuum_panel.setTitle(f"Vacuum Pump ({vacuum_port})")
            self.fluid_panel.setTitle(f"Fluid Pump ({fluid_port})")

            # Create and start sensor worker
            self.sensor_worker = SensorWorker(sensor_port)
            self.sensor_worker.data_received.connect(self._on_sensor_data)
            self.sensor_worker.error_occurred.connect(self._on_sensor_error)
            self.sensor_worker.connection_lost.connect(self._on_connection_lost)
            self.sensor_worker.start()

            # Create pump controllers
            self.vacuum_controller = PumpController()
            vacuum_diameter = self.vacuum_panel.get_syringe_diameter()
            if not self.vacuum_controller.connect(vacuum_port, vacuum_diameter):
                raise Exception("Failed to connect to vacuum pump")

            self.fluid_controller = PumpController()
            fluid_diameter = self.fluid_panel.get_syringe_diameter()
            if not self.fluid_controller.connect(fluid_port, fluid_diameter):
                raise Exception("Failed to connect to fluid pump")

            # Initialize PID controllers
            vacuum_params = self.vacuum_panel.get_pid_params()
            self.vacuum_pid = PIDController(
                kp=vacuum_params['kp'],
                ki=vacuum_params['ki'],
                kd=vacuum_params['kd'],
                setpoint=vacuum_params['setpoint'],
                output_limits=(-vacuum_params['max_rate'], vacuum_params['max_rate']),
                sample_time=vacuum_params['sample_time']
            )
            self.vacuum_max_rate = vacuum_params['max_rate']

            fluid_params = self.fluid_panel.get_pid_params()
            self.fluid_pid = PIDController(
                kp=fluid_params['kp'],
                ki=fluid_params['ki'],
                kd=fluid_params['kd'],
                setpoint=fluid_params['setpoint'],
                output_limits=(-fluid_params['max_rate'], fluid_params['max_rate']),
                sample_time=fluid_params['sample_time']
            )
            self.fluid_max_rate = fluid_params['max_rate']

            # Update UI state
            self.connected = True
            self.vacuum_panel.setEnabled(True)
            self.fluid_panel.setEnabled(True)
            self.connection_label.setText(f"Connected (Sensor: {sensor_port})")

            # Start PID timer
            self.pid_timer.start()

            # Clear old plot data
            self.plot_widget.clear_data()

        except Exception as e:
            self._on_disconnect()
            QMessageBox.critical(self, "Connection Error", str(e))
            # Re-show port dialog on error
            QTimer.singleShot(100, self._show_port_dialog)

    def _on_disconnect(self):
        """Handle disconnection."""
        # Stop PID timer
        self.pid_timer.stop()

        # Stop sensor worker
        if self.sensor_worker:
            self.sensor_worker.stop()
            self.sensor_worker.wait(2000)
            self.sensor_worker = None

        # Disconnect pumps
        if self.vacuum_controller:
            self.vacuum_controller.stop_pump()
            self.vacuum_controller.disconnect()
            self.vacuum_controller = None

        if self.fluid_controller:
            self.fluid_controller.stop_pump()
            self.fluid_controller.disconnect()
            self.fluid_controller = None

        # Reset PID
        self.vacuum_pid = None
        self.fluid_pid = None
        self.vacuum_pid_enabled = False
        self.fluid_pid_enabled = False

        # Close CSV file
        if self.csv_file:
            self.csv_file.close()
            self.csv_file = None
            self.csv_writer = None

        # Update UI state
        self.connected = False
        self.vacuum_panel.setEnabled(False)
        self.fluid_panel.setEnabled(False)
        self.connection_label.setText("Disconnected")

    @pyqtSlot(float, float, object)
    def _on_sensor_data(self, vacuum_pressure, fluid_pressure, timestamp):
        """Handle incoming sensor data."""
        self.latest_vacuum_pressure = vacuum_pressure
        self.latest_fluid_pressure = fluid_pressure

        # Get current rates
        vacuum_rate = self.vacuum_controller.get_signed_rate() if self.vacuum_controller else 0.0
        fluid_rate = self.fluid_controller.get_signed_rate() if self.fluid_controller else 0.0

        # Get current setpoints (None if PID not enabled)
        vacuum_setpoint = self.vacuum_pid.setpoint if self.vacuum_pid_enabled and self.vacuum_pid else None
        fluid_setpoint = self.fluid_pid.setpoint if self.fluid_pid_enabled and self.fluid_pid else None

        # Log to CSV (empty string for None setpoints)
        if self.csv_writer:
            self.csv_writer.writerow([
                timestamp.strftime('%Y-%m-%d %H:%M:%S.%f'),
                vacuum_pressure,
                vacuum_rate,
                vacuum_setpoint if vacuum_setpoint is not None else '',
                fluid_pressure,
                fluid_rate,
                fluid_setpoint if fluid_setpoint is not None else ''
            ])
            self.csv_file.flush()

        # Add to plots (pass setpoints for time series plotting)
        self.plot_widget.add_vacuum_data(vacuum_pressure, vacuum_rate, timestamp, vacuum_setpoint)
        self.plot_widget.add_fluid_data(fluid_pressure, fluid_rate, timestamp, fluid_setpoint)

        # Update status
        self.vacuum_status_label.setText(f"Vacuum: P={vacuum_pressure:.2f}, R={vacuum_rate:.2f}")
        self.fluid_status_label.setText(f"Fluid: P={fluid_pressure:.2f}, R={fluid_rate:.2f}")

    @pyqtSlot(str)
    def _on_sensor_error(self, error):
        """Handle sensor error."""
        self.status_bar.showMessage(f"Sensor error: {error}", 5000)

    @pyqtSlot()
    def _on_connection_lost(self):
        """Handle connection lost."""
        QMessageBox.warning(self, "Connection Lost", "Lost connection to sensor")
        self._on_disconnect()

    def _update_pid(self):
        """Update PID controllers and send commands to pumps."""
        # Vacuum pump PID
        if self.vacuum_pid_enabled and self.vacuum_pid and self.latest_vacuum_pressure is not None:
            output = self.vacuum_pid.update(self.latest_vacuum_pressure)
            self._apply_pid_output(self.vacuum_controller, output, self.vacuum_max_rate)

        # Fluid pump PID
        if self.fluid_pid_enabled and self.fluid_pid and self.latest_fluid_pressure is not None:
            output = self.fluid_pid.update(self.latest_fluid_pressure)
            self._apply_pid_output(self.fluid_controller, output, self.fluid_max_rate)

    def _apply_pid_output(self, controller, output, max_rate):
        """Apply PID output to a pump controller."""
        if not controller:
            return

        min_rate = 0.1
        if abs(output) < min_rate:
            controller.stop_pump()
        else:
            direction = "INF" if output > 0 else "WDR"
            rate = abs(output)
            rate = max(min_rate, min(max_rate, rate))  # Clamp rate

            if controller.current_direction != direction:
                controller.stop_pump()
                controller.start_pump(rate, direction)
            elif abs(rate - controller.current_rate) > 0.1:
                controller.change_rate(rate)

    # Vacuum pump handlers
    @pyqtSlot(float, str, str)
    def _on_vacuum_start(self, rate, direction, units):
        """Handle vacuum pump start request."""
        if self.vacuum_controller:
            pump_units = RATE_UNITS_CMD.get(units, 'MH')
            self.vacuum_controller.start_pump(rate, direction, pump_units)

    @pyqtSlot()
    def _on_vacuum_stop(self):
        """Handle vacuum pump stop request."""
        if self.vacuum_controller:
            self.vacuum_controller.stop_pump()

    @pyqtSlot(float)
    def _on_vacuum_syringe_changed(self, diameter):
        """Handle vacuum syringe change."""
        if self.vacuum_controller:
            self.vacuum_controller.set_diameter(diameter)

    @pyqtSlot(bool)
    def _on_vacuum_pid_enabled(self, enabled):
        """Handle vacuum PID enable/disable."""
        self.vacuum_pid_enabled = enabled
        if enabled:
            params = self.vacuum_panel.get_pid_params()
            self.plot_widget.set_vacuum_setpoint(params['setpoint'])
        else:
            self.plot_widget.set_vacuum_setpoint(None)
            if self.vacuum_controller:
                self.vacuum_controller.stop_pump()

    @pyqtSlot(float, float, float, float, float, float)
    def _on_vacuum_pid_params(self, kp, ki, kd, setpoint, max_rate, sample_time):
        """Handle vacuum PID parameter change."""
        if self.vacuum_pid:
            self.vacuum_pid.kp = kp
            self.vacuum_pid.ki = ki
            self.vacuum_pid.kd = kd
            self.vacuum_pid.setpoint = setpoint
            self.vacuum_pid.output_limits = (-max_rate, max_rate)
            self.vacuum_pid.sample_time = sample_time
        self.vacuum_max_rate = max_rate
        if self.vacuum_pid_enabled:
            self.plot_widget.set_vacuum_setpoint(setpoint)

    @pyqtSlot()
    def _on_vacuum_pid_reset(self):
        """Handle vacuum PID reset."""
        if self.vacuum_pid:
            self.vacuum_pid.reset()

    # Fluid pump handlers
    @pyqtSlot(float, str, str)
    def _on_fluid_start(self, rate, direction, units):
        """Handle fluid pump start request."""
        if self.fluid_controller:
            pump_units = RATE_UNITS_CMD.get(units, 'MH')
            self.fluid_controller.start_pump(rate, direction, pump_units)

    @pyqtSlot()
    def _on_fluid_stop(self):
        """Handle fluid pump stop request."""
        if self.fluid_controller:
            self.fluid_controller.stop_pump()

    @pyqtSlot(float)
    def _on_fluid_syringe_changed(self, diameter):
        """Handle fluid syringe change."""
        if self.fluid_controller:
            self.fluid_controller.set_diameter(diameter)

    @pyqtSlot(bool)
    def _on_fluid_pid_enabled(self, enabled):
        """Handle fluid PID enable/disable."""
        self.fluid_pid_enabled = enabled
        if enabled:
            params = self.fluid_panel.get_pid_params()
            self.plot_widget.set_fluid_setpoint(params['setpoint'])
        else:
            self.plot_widget.set_fluid_setpoint(None)
            if self.fluid_controller:
                self.fluid_controller.stop_pump()

    @pyqtSlot(float, float, float, float, float, float)
    def _on_fluid_pid_params(self, kp, ki, kd, setpoint, max_rate, sample_time):
        """Handle fluid PID parameter change."""
        if self.fluid_pid:
            self.fluid_pid.kp = kp
            self.fluid_pid.ki = ki
            self.fluid_pid.kd = kd
            self.fluid_pid.setpoint = setpoint
            self.fluid_pid.output_limits = (-max_rate, max_rate)
            self.fluid_pid.sample_time = sample_time
        self.fluid_max_rate = max_rate
        if self.fluid_pid_enabled:
            self.plot_widget.set_fluid_setpoint(setpoint)

    @pyqtSlot()
    def _on_fluid_pid_reset(self):
        """Handle fluid PID reset."""
        if self.fluid_pid:
            self.fluid_pid.reset()

    def closeEvent(self, event):
        """Handle window close - ensure pumps are stopped."""
        self._on_disconnect()
        event.accept()
