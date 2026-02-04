"""
Live plotting widget for dual pump monitoring.
"""

from collections import deque
from datetime import datetime

from PyQt5.QtWidgets import QWidget, QVBoxLayout
from PyQt5.QtCore import QTimer

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import matplotlib.dates as mdates


class LivePlotWidget(QWidget):
    """Widget displaying two side-by-side plots for vacuum and fluid pumps."""

    def __init__(self, max_points=500, parent=None):
        super().__init__(parent)
        self.max_points = max_points

        # Data storage for vacuum pump
        self.vacuum_timestamps = deque(maxlen=max_points)
        self.vacuum_pressures = deque(maxlen=max_points)
        self.vacuum_rates = deque(maxlen=max_points)
        self.vacuum_setpoints = deque(maxlen=max_points)  # Time series of setpoints (None when disabled)
        self.vacuum_setpoint = None  # Current setpoint value

        # Data storage for fluid pump
        self.fluid_timestamps = deque(maxlen=max_points)
        self.fluid_pressures = deque(maxlen=max_points)
        self.fluid_rates = deque(maxlen=max_points)
        self.fluid_setpoints = deque(maxlen=max_points)  # Time series of setpoints (None when disabled)
        self.fluid_setpoint = None  # Current setpoint value

        # Setup matplotlib figure
        self.figure = Figure(figsize=(12, 5))
        self.canvas = FigureCanvas(self.figure)

        # Create two subplots side by side
        self.ax_vacuum = self.figure.add_subplot(121)
        self.ax_fluid = self.figure.add_subplot(122)

        # Create twin axes for flow rates
        self.ax_vacuum_rate = self.ax_vacuum.twinx()
        self.ax_fluid_rate = self.ax_fluid.twinx()

        # Configure plots
        self._setup_axes()

        # Layout
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self.canvas)

        # Update timer (100ms interval)
        self.update_timer = QTimer(self)
        self.update_timer.timeout.connect(self.update_plots)
        self.update_timer.start(100)

    def _setup_axes(self):
        """Configure the axes appearance."""
        # Vacuum pump plot - pressure on left (blue), flow rate on right (red)
        self.ax_vacuum.set_title('Vacuum Pump')
        self.ax_vacuum.set_xlabel('Time (s)')
        self.ax_vacuum.set_ylabel('Pressure', color='blue')
        self.ax_vacuum.tick_params(axis='y', labelcolor='blue')
        self.ax_vacuum.yaxis.set_label_position('left')

        self.ax_vacuum_rate.set_ylabel('Flow Rate', color='red')
        self.ax_vacuum_rate.tick_params(axis='y', labelcolor='red')
        self.ax_vacuum_rate.yaxis.set_label_position('right')
        self.ax_vacuum_rate.yaxis.tick_right()

        # Fluid pump plot - pressure on left (blue), flow rate on right (red)
        self.ax_fluid.set_title('Fluid Pump')
        self.ax_fluid.set_xlabel('Time (s)')
        self.ax_fluid.set_ylabel('Pressure', color='blue')
        self.ax_fluid.tick_params(axis='y', labelcolor='blue')
        self.ax_fluid.yaxis.set_label_position('left')

        self.ax_fluid_rate.set_ylabel('Flow Rate', color='red')
        self.ax_fluid_rate.tick_params(axis='y', labelcolor='red')
        self.ax_fluid_rate.yaxis.set_label_position('right')
        self.ax_fluid_rate.yaxis.tick_right()

        self.figure.tight_layout()

    def add_vacuum_data(self, pressure, rate, timestamp=None, setpoint=None):
        """Add a data point for the vacuum pump."""
        if timestamp is None:
            timestamp = datetime.now()
        self.vacuum_timestamps.append(timestamp)
        self.vacuum_pressures.append(pressure)
        self.vacuum_rates.append(rate)
        self.vacuum_setpoints.append(setpoint)

    def add_fluid_data(self, pressure, rate, timestamp=None, setpoint=None):
        """Add a data point for the fluid pump."""
        if timestamp is None:
            timestamp = datetime.now()
        self.fluid_timestamps.append(timestamp)
        self.fluid_pressures.append(pressure)
        self.fluid_rates.append(rate)
        self.fluid_setpoints.append(setpoint)

    def set_vacuum_setpoint(self, setpoint):
        """Set the vacuum pump setpoint (None to disable)."""
        self.vacuum_setpoint = setpoint

    def set_fluid_setpoint(self, setpoint):
        """Set the fluid pump setpoint (None to disable)."""
        self.fluid_setpoint = setpoint

    def _plot_setpoint_segments(self, ax, times, setpoints, label):
        """Plot setpoint as connected segments, with gaps where setpoint is None.

        This avoids interpolating lines across periods when pressure control was disabled.
        """
        if not times or not setpoints:
            return

        # Find contiguous segments where setpoint is not None
        segments_times = []
        segments_values = []
        current_times = []
        current_values = []
        label_added = False

        for t, sp in zip(times, setpoints):
            if sp is not None:
                current_times.append(t)
                current_values.append(sp)
            else:
                # End of a segment
                if current_times:
                    segments_times.append(current_times)
                    segments_values.append(current_values)
                    current_times = []
                    current_values = []

        # Don't forget the last segment
        if current_times:
            segments_times.append(current_times)
            segments_values.append(current_values)

        # Plot each segment
        for seg_times, seg_values in zip(segments_times, segments_values):
            if label_added:
                ax.plot(seg_times, seg_values, 'g:', linewidth=2)
            else:
                ax.plot(seg_times, seg_values, 'g:', linewidth=2, label=label)
                label_added = True

    def update_plots(self):
        """Redraw both plots with current data."""
        # Clear axes
        self.ax_vacuum.clear()
        self.ax_vacuum_rate.clear()
        self.ax_fluid.clear()
        self.ax_fluid_rate.clear()

        # Re-setup axes labels
        self._setup_axes()

        # Plot vacuum data
        if len(self.vacuum_timestamps) > 0:
            times = list(self.vacuum_timestamps)
            pressures = list(self.vacuum_pressures)
            rates = list(self.vacuum_rates)
            setpoints = list(self.vacuum_setpoints)

            # Pressure line (blue, left axis)
            self.ax_vacuum.plot(times, pressures, 'b-', linewidth=1.5, label='Pressure')

            # Flow rate line (red, right axis)
            self.ax_vacuum_rate.plot(times, rates, 'r-', linewidth=1.5, label='Flow Rate')

            # Setpoint line (green dotted) - plot segments where setpoint is not None
            self._plot_setpoint_segments(self.ax_vacuum, times, setpoints, 'Setpoint')

            # Format x-axis
            self.ax_vacuum.xaxis.set_major_formatter(mdates.DateFormatter('%H:%M:%S'))
            for label in self.ax_vacuum.xaxis.get_ticklabels():
                label.set_rotation(45)
                label.set_ha('right')

            # Add legend
            lines1, labels1 = self.ax_vacuum.get_legend_handles_labels()
            lines2, labels2 = self.ax_vacuum_rate.get_legend_handles_labels()
            self.ax_vacuum.legend(lines1 + lines2, labels1 + labels2, loc='upper left', fontsize='small')

        # Plot fluid data
        if len(self.fluid_timestamps) > 0:
            times = list(self.fluid_timestamps)
            pressures = list(self.fluid_pressures)
            rates = list(self.fluid_rates)
            setpoints = list(self.fluid_setpoints)

            # Pressure line (blue, left axis)
            self.ax_fluid.plot(times, pressures, 'b-', linewidth=1.5, label='Pressure')

            # Flow rate line (red, right axis)
            self.ax_fluid_rate.plot(times, rates, 'r-', linewidth=1.5, label='Flow Rate')

            # Setpoint line (green dotted) - plot segments where setpoint is not None
            self._plot_setpoint_segments(self.ax_fluid, times, setpoints, 'Setpoint')

            # Format x-axis
            self.ax_fluid.xaxis.set_major_formatter(mdates.DateFormatter('%H:%M:%S'))
            for label in self.ax_fluid.xaxis.get_ticklabels():
                label.set_rotation(45)
                label.set_ha('right')

            # Add legend
            lines1, labels1 = self.ax_fluid.get_legend_handles_labels()
            lines2, labels2 = self.ax_fluid_rate.get_legend_handles_labels()
            self.ax_fluid.legend(lines1 + lines2, labels1 + labels2, loc='upper left', fontsize='small')

        # Add grid
        self.ax_vacuum.grid(True, alpha=0.3)
        self.ax_fluid.grid(True, alpha=0.3)

        self.figure.tight_layout()
        self.canvas.draw()

    def clear_data(self):
        """Clear all data from both plots."""
        self.vacuum_timestamps.clear()
        self.vacuum_pressures.clear()
        self.vacuum_rates.clear()
        self.vacuum_setpoints.clear()
        self.fluid_timestamps.clear()
        self.fluid_pressures.clear()
        self.fluid_rates.clear()
        self.fluid_setpoints.clear()
        self.vacuum_setpoint = None
        self.fluid_setpoint = None
