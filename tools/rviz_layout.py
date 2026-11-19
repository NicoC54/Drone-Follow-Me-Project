#!/usr/bin/env python3
"""Generates the 'QMainWindow State' of follow_me.rviz: dock layout (RViz does not store panel
sizes anywhere else). Left column 600 px wide: camera HUD on top (16:9), Displays below.

    QT_QPA_PLATFORM=offscreen python3 tools/rviz_layout.py      # prints the hex string
RViz dock widgets are named after the panel / display ("Displays", "Camera HUD").
"""
import sys

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QApplication, QDockWidget, QMainWindow, QWidget

WIDTH, HEIGHT, COLUMN, CAMERA_H = 1400, 900, 600, 360

app = QApplication(sys.argv)
win = QMainWindow()
win.setCentralWidget(QWidget())
win.resize(WIDTH, HEIGHT)
docks = {}
for name in ("Camera HUD", "Displays"):
    d = QDockWidget(name, win)
    d.setObjectName(name)
    d.setWidget(QWidget())
    win.addDockWidget(Qt.LeftDockWidgetArea, d)
    docks[name] = d
win.splitDockWidget(docks["Camera HUD"], docks["Displays"], Qt.Vertical)
win.show()
app.processEvents()
win.resizeDocks([docks["Camera HUD"]], [COLUMN], Qt.Horizontal)
win.resizeDocks([docks["Camera HUD"], docks["Displays"]], [CAMERA_H, HEIGHT - CAMERA_H - 120], Qt.Vertical)
app.processEvents()
print(bytes(win.saveState().toHex()).decode())
