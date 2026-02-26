"""
Control panel widgets for the pump and sensor GUI.
"""

import serial.tools.list_ports

from PyQt5.QtWidgets import (
    QWidget, QGroupBox, QVBoxLayout, QHBoxLayout, QGridLayout,
    QLabel, QComboBox, QPushButton, QDoubleSpinBox, QStackedWidget,
    QButtonGroup
)
from PyQt5.QtCore import pyqtSignal, Qt

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from syringe_pump import RATE_UNITS, SYRINGE_SIZES, DIRECTIONS


class SerialPortPanel(QGroupBox):
    """Panel for selecting serial ports for sensor and pumps."""

    refresh_requested = pyqtSignal()
    connect_requested = pyqtSignal(str, str, str)  # sensor_port, vacuum_port, fluid_port
    disconnect_requested = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__("Serial Ports", parent)
        self._setup_ui()
        self.refresh_ports()

    def _setup_ui(self):
        layout = QGridLayout(self)

        # Sensor port
        layout.addWidget(QLabel("Sensor Input:"), 0, 0)
        self.sensor_combo = QComboBox()
        layout.addWidget(self.sensor_combo, 0, 1)

        # Vacuum pump port
        layout.addWidget(QLabel("Vacuum Pump:"), 1, 0)
        self.vacuum_combo = QComboBox()
        layout.addWidget(self.vacuum_combo, 1, 1)

        # Fluid pump port
        layout.addWidget(QLabel("Fluid Pump:"), 2, 0)
        self.fluid_combo = QComboBox()
        layout.addWidget(self.fluid_combo, 2, 1)

        # Buttons
        button_layout = QHBoxLayout()
        self.refresh_btn = QPushButton("Refresh Ports")
        self.refresh_btn.clicked.connect(self.refresh_ports)
        button_layout.addWidget(self.refresh_btn)

        self.connect_btn = QPushButton("Connect All")
        self.connect_btn.clicked.connect(self._on_connect_clicked)
        button_layout.addWidget(self.connect_btn)

        self.disconnect_btn = QPushButton("Disconnect All")
        self.disconnect_btn.clicked.connect(self.disconnect_requested.emit)
        self.disconnect_btn.setEnabled(False)
        button_layout.addWidget(self.disconnect_btn)

        layout.addLayout(button_layout, 3, 0, 1, 2)

    def refresh_ports(self):
        """Refresh the list of available serial ports."""
        ports = list(serial.tools.list_ports.comports())

        for combo in [self.sensor_combo, self.vacuum_combo, self.fluid_combo]:
            current = combo.currentText()
            combo.clear()
            for port in ports:
                combo.addItem(f"{port.device}: {port.description}", port.device)
            # Restore selection if still available
            idx = combo.findText(current, Qt.MatchStartsWith)
            if idx >= 0:
                combo.setCurrentIndex(idx)

        self.refresh_requested.emit()

    def _on_connect_clicked(self):
        """Handle connect button click."""
        sensor_port = self.sensor_combo.currentData()
        vacuum_port = self.vacuum_combo.currentData()
        fluid_port = self.fluid_combo.currentData()

        if sensor_port and vacuum_port and fluid_port:
            self.connect_requested.emit(sensor_port, vacuum_port, fluid_port)

    def set_connected(self, connected):
        """Update UI state based on connection status."""
        self.connect_btn.setEnabled(not connected)
        self.disconnect_btn.setEnabled(connected)
        self.sensor_combo.setEnabled(not connected)
        self.vacuum_combo.setEnabled(not connected)
        self.fluid_combo.setEnabled(not connected)
        self.refresh_btn.setEnabled(not connected)


class PumpControlPanel(QGroupBox):
    """Control panel for a single pump with toggle between manual and PID modes."""

    start_requested = pyqtSignal(float, str, str)  # rate, direction, units
    stop_requested = pyqtSignal()
    syringe_changed = pyqtSignal(float)  # diameter
    pid_enabled_changed = pyqtSignal(bool)
    pid_params_changed = pyqtSignal(float, float, float, float, float, float)  # kp, ki, kd, setpoint, max_rate, sample_time
    pid_reset_requested = pyqtSignal()

    def __init__(self, title, default_syringe='50 mL', default_units='ML/HR', default_direction='INF', default_mode='flow', parent=None):
        super().__init__(title, parent)
        self.default_syringe = default_syringe
        self.default_units = default_units
        self.default_direction = default_direction
        self.default_mode = default_mode
        self._pid_mode = (default_mode == 'pressure')
        self._pid_active = False
        self._setup_ui()

    def _setup_ui(self):
        layout = QVBoxLayout(self)

        # Syringe selection (no label - title serves as identifier)
        syringe_layout = QHBoxLayout()
        self.syringe_combo = QComboBox()
        for name in SYRINGE_SIZES.keys():
            self.syringe_combo.addItem(name, SYRINGE_SIZES[name])
        # Set default
        idx = self.syringe_combo.findText(self.default_syringe)
        if idx >= 0:
            self.syringe_combo.setCurrentIndex(idx)
        self.syringe_combo.currentIndexChanged.connect(self._on_syringe_changed)
        syringe_layout.addWidget(self.syringe_combo)
        layout.addLayout(syringe_layout)

        # Mode toggle - segmented control style
        toggle_layout = QHBoxLayout()
        toggle_layout.setSpacing(0)

        self.flow_btn = QPushButton("Flow Control")
        self.flow_btn.setCheckable(True)

        self.pressure_btn = QPushButton("Pressure Control")
        self.pressure_btn.setCheckable(True)

        if self.default_mode == 'pressure':
            self.pressure_btn.setChecked(True)
        else:
            self.flow_btn.setChecked(True)

        self.mode_group = QButtonGroup()
        self.mode_group.setExclusive(True)
        self.mode_group.addButton(self.flow_btn, 0)
        self.mode_group.addButton(self.pressure_btn, 1)
        self.mode_group.buttonClicked.connect(self._on_mode_changed)

        toggle_layout.addWidget(self.flow_btn)
        toggle_layout.addWidget(self.pressure_btn)
        layout.addLayout(toggle_layout)

        self._update_toggle_styles()

        # Stacked widget for manual/PID views
        self.stack = QStackedWidget()

        # --- Manual control page (index 0) ---
        manual_widget = QWidget()
        manual_layout = QGridLayout(manual_widget)
        manual_layout.setContentsMargins(0, 0, 0, 0)

        # Rate
        manual_layout.addWidget(QLabel("Rate:"), 0, 0)
        self.rate_spin = QDoubleSpinBox()
        self.rate_spin.setRange(0.1, 1000.0)
        self.rate_spin.setValue(10.0)
        self.rate_spin.setDecimals(2)
        manual_layout.addWidget(self.rate_spin, 0, 1)

        # Units
        manual_layout.addWidget(QLabel("Units:"), 0, 2)
        self.units_combo = QComboBox()
        for unit in RATE_UNITS:
            self.units_combo.addItem(unit)
        # Set default units
        idx = self.units_combo.findText(self.default_units)
        if idx >= 0:
            self.units_combo.setCurrentIndex(idx)
        manual_layout.addWidget(self.units_combo, 0, 3)

        # Direction
        manual_layout.addWidget(QLabel("Direction:"), 1, 0)
        self.direction_combo = QComboBox()
        self.direction_combo.addItem("Infuse", "INF")
        self.direction_combo.addItem("Withdraw", "WDR")
        # Set default direction
        dir_idx = 0 if self.default_direction == "INF" else 1
        self.direction_combo.setCurrentIndex(dir_idx)
        manual_layout.addWidget(self.direction_combo, 1, 1)

        # Start/Stop buttons
        self.start_btn = QPushButton("Start")
        self.start_btn.clicked.connect(self._on_start_clicked)
        manual_layout.addWidget(self.start_btn, 1, 2)

        self.stop_btn = QPushButton("Stop")
        self.stop_btn.clicked.connect(self.stop_requested.emit)
        manual_layout.addWidget(self.stop_btn, 1, 3)

        self.stack.addWidget(manual_widget)

        # --- PID control page (index 1) ---
        pid_widget = QWidget()
        pid_layout = QGridLayout(pid_widget)
        pid_layout.setContentsMargins(0, 0, 0, 0)

        # Setpoint
        pid_layout.addWidget(QLabel("Setpoint:"), 0, 0)
        self.setpoint_spin = QDoubleSpinBox()
        self.setpoint_spin.setRange(-10000.0, 10000.0)
        self.setpoint_spin.setValue(8.0)
        self.setpoint_spin.setDecimals(2)
        pid_layout.addWidget(self.setpoint_spin, 0, 1)

        # Kp
        pid_layout.addWidget(QLabel("Kp:"), 1, 0)
        self.kp_spin = QDoubleSpinBox()
        self.kp_spin.setRange(-1000.0, 1000.0)
        self.kp_spin.setValue(-15.0)
        self.kp_spin.setDecimals(3)
        pid_layout.addWidget(self.kp_spin, 1, 1)

        # Ki
        pid_layout.addWidget(QLabel("Ki:"), 1, 2)
        self.ki_spin = QDoubleSpinBox()
        self.ki_spin.setRange(-1000.0, 1000.0)
        self.ki_spin.setValue(0.0)
        self.ki_spin.setDecimals(3)
        pid_layout.addWidget(self.ki_spin, 1, 3)

        # Kd
        pid_layout.addWidget(QLabel("Kd:"), 2, 0)
        self.kd_spin = QDoubleSpinBox()
        self.kd_spin.setRange(-1000.0, 1000.0)
        self.kd_spin.setValue(0.0)
        self.kd_spin.setDecimals(3)
        pid_layout.addWidget(self.kd_spin, 2, 1)

        # Max Rate
        pid_layout.addWidget(QLabel("Max Rate:"), 2, 2)
        self.max_rate_spin = QDoubleSpinBox()
        self.max_rate_spin.setRange(0.1, 999.9)
        self.max_rate_spin.setValue(999.9)
        self.max_rate_spin.setDecimals(1)
        pid_layout.addWidget(self.max_rate_spin, 2, 3)

        # Sample Time
        pid_layout.addWidget(QLabel("Sample Time:"), 3, 0)
        self.sample_time_spin = QDoubleSpinBox()
        self.sample_time_spin.setRange(0.1, 60.0)
        self.sample_time_spin.setValue(1.0)
        self.sample_time_spin.setDecimals(1)
        pid_layout.addWidget(self.sample_time_spin, 3, 1)

        # Apply and Reset buttons
        self.apply_btn = QPushButton("Apply")
        self.apply_btn.clicked.connect(self._on_apply_clicked)
        pid_layout.addWidget(self.apply_btn, 3, 2)

        self.reset_btn = QPushButton("Reset")
        self.reset_btn.clicked.connect(self.pid_reset_requested.emit)
        pid_layout.addWidget(self.reset_btn, 3, 3)

        # Start/Stop control button
        self.pid_start_stop_btn = QPushButton("Start Control")
        self.pid_start_stop_btn.clicked.connect(self._on_pid_start_stop_clicked)
        pid_layout.addWidget(self.pid_start_stop_btn, 4, 0, 1, 4)

        self.stack.addWidget(pid_widget)

        layout.addWidget(self.stack)

        # Set initial stack page based on default mode
        self.stack.setCurrentIndex(1 if self.default_mode == 'pressure' else 0)

    def _update_toggle_styles(self):
        """Update button styles based on selection state."""
        selected_style = "background-color: #0078d4; color: white; border: 1px solid #0078d4; padding: 5px 10px;"
        unselected_style = "background-color: #f0f0f0; color: #333; border: 1px solid #ccc; padding: 5px 10px;"

        if self.flow_btn.isChecked():
            self.flow_btn.setStyleSheet(selected_style)
            self.pressure_btn.setStyleSheet(unselected_style)
        else:
            self.flow_btn.setStyleSheet(unselected_style)
            self.pressure_btn.setStyleSheet(selected_style)

    def _on_mode_changed(self, button):
        """Handle mode toggle button click."""
        self._pid_mode = (button == self.pressure_btn)
        self._update_toggle_styles()

        # Always deactivate PID when switching modes
        self._pid_active = False
        self.pid_start_stop_btn.setText("Start Control")
        self.pid_enabled_changed.emit(False)

        if self._pid_mode:
            self.stack.setCurrentIndex(1)
        else:
            self.stack.setCurrentIndex(0)

    def _on_syringe_changed(self):
        """Handle syringe size change."""
        diameter = self.syringe_combo.currentData()
        if diameter:
            self.syringe_changed.emit(diameter)

    def _on_start_clicked(self):
        """Handle start button click."""
        rate = self.rate_spin.value()
        direction = self.direction_combo.currentData()
        units = self.units_combo.currentText()
        self.start_requested.emit(rate, direction, units)

    def _on_apply_clicked(self):
        """Handle apply button click."""
        kp = self.kp_spin.value()
        ki = self.ki_spin.value()
        kd = self.kd_spin.value()
        setpoint = self.setpoint_spin.value()
        max_rate = self.max_rate_spin.value()
        sample_time = self.sample_time_spin.value()
        self.pid_params_changed.emit(kp, ki, kd, setpoint, max_rate, sample_time)

    def _on_pid_start_stop_clicked(self):
        """Toggle PID active state."""
        self._pid_active = not self._pid_active
        if self._pid_active:
            self.pid_start_stop_btn.setText("Stop Control")
            self._on_apply_clicked()  # sync params before starting
            self.pid_enabled_changed.emit(True)
        else:
            self.pid_start_stop_btn.setText("Start Control")
            self.pid_enabled_changed.emit(False)

    def set_connected(self, connected):
        """Update UI state based on connection status."""
        self.setEnabled(connected)

    def get_syringe_diameter(self):
        """Get the currently selected syringe diameter."""
        return self.syringe_combo.currentData()

    def is_pid_enabled(self):
        """Check if PID mode is active."""
        return self._pid_mode

    def get_pid_params(self):
        """Get current PID parameters."""
        return {
            'kp': self.kp_spin.value(),
            'ki': self.ki_spin.value(),
            'kd': self.kd_spin.value(),
            'setpoint': self.setpoint_spin.value(),
            'max_rate': self.max_rate_spin.value(),
            'sample_time': self.sample_time_spin.value()
        }
