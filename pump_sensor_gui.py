#!/usr/bin/env python3
"""
Dual Pump and Sensor Control GUI Application

A PyQt5-based GUI for controlling two syringe pumps (vacuum and fluid) with
PID feedback from pressure sensors.

Usage:
    python pump_sensor_gui.py
"""

import sys
from PyQt5.QtWidgets import QApplication
from gui.main_window import MainWindow


def main():
    app = QApplication(sys.argv)
    app.setApplicationName("Dual Pump and Sensor Control")

    window = MainWindow()
    window.show()

    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
