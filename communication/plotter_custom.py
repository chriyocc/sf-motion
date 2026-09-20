import contextlib
import io
import json
import struct
import sys
import time
import traceback
from collections import deque
from pathlib import Path

import numpy as np
import pyqtgraph as pg
import queue
import serial
import serial.tools.list_ports
from PyQt5 import QtCore, QtWidgets

from MotorProtocol import MotorProtocol


MODE_LABELS = {
    0: "Current",
    1: "Speed",
    2: "Position",
    3: "Encoder Calibration",
    4: "Audio",
    5: "Voltage",
    6: "Disabled",
}

CONTROL_MODES = {
    "Disabled": 6,
    "Current": 0,
    "Speed": 1,
    "Position": 2,
}

PLOT_PRESETS = {
    "Current": ["Is_ref", "id", "iq"],
    "Speed": ["rpm_ref", "actual_rpm"],
    "Position": ["pos_ref", "actual_angle"],
    "Encoder": ["m_angle_rad", "m_angle_rad_comp", "e_rad"],
    "Voltage": ["v_bus", "vd", "vq", "va", "vb", "vc"],
}

ORIGINAL_PID_VALUES = {
    "pid_id.kp": 0.02,
    "pid_id.ki": 12.0,
    "pid_id.deadband": 0.0,
    "pid_iq.kp": 0.02,
    "pid_iq.ki": 12.0,
    "pid_iq.deadband": 0.0,
    "pid_speed.kp": 0.01,
    "pid_speed.ki": 0.1,
    "pid_speed.out_max": 10.0,
    "pid_speed.deadband": 0.01,
    "pid_position.kp": 4.1,
    "pid_position.ki": 0.0,
    "pid_position.kd": 0.21,
    "pid_position.out_max": 3.0,
    "pid_position.deadband": 0.01,
    "pid_position.d_fc": 100.0,
    "field_weakening.kp": 0.1,
    "field_weakening.ki": 1.0,
    "field_weakening.out_min": -3.5,
    "field_weakening_enable": False,
    "mtpa_enable": False,
}


class SerialPortDialog(QtWidgets.QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Select Serial Port")
        self.setModal(True)
        self.setFixedWidth(420)

        layout = QtWidgets.QVBoxLayout(self)
        layout.addWidget(QtWidgets.QLabel("Select Serial Port:"))

        self.port_combo = QtWidgets.QComboBox()
        self.port_combo.setPlaceholderText("Select port...")
        layout.addWidget(self.port_combo)

        refresh_btn = QtWidgets.QPushButton("Refresh Ports")
        refresh_btn.clicked.connect(self.refresh_ports)
        layout.addWidget(refresh_btn)

        layout.addWidget(QtWidgets.QLabel("Baudrate:"))
        self.baud_combo = QtWidgets.QComboBox()
        self.baud_combo.addItems(
            ["9600", "19200", "38400", "57600", "115200", "230400", "460800"]
        )
        self.baud_combo.setCurrentText("115200")
        layout.addWidget(self.baud_combo)

        button_box = QtWidgets.QDialogButtonBox(
            QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel
        )
        button_box.accepted.connect(self.accept)
        button_box.rejected.connect(self.reject)
        layout.addWidget(button_box)

        self.refresh_ports()
        if self.port_combo.count() == 1:
            self.port_combo.setCurrentIndex(0)

    def refresh_ports(self):
        self.port_combo.clear()
        ports = serial.tools.list_ports.comports()

        if not ports:
            self.port_combo.addItem("No ports found", None)
            self.port_combo.setEnabled(False)
        else:
            self.port_combo.setEnabled(True)
            for port in ports:
                description = f"{port.device} - {port.description}"
                self.port_combo.addItem(description, port.device)
            self.port_combo.setCurrentIndex(0)

    def get_selected_port(self):
        if self.port_combo.currentIndex() >= 0:
            return self.port_combo.currentData()
        return None

    def get_baudrate(self):
        return int(self.baud_combo.currentText())


class DataAcquisitionThread(QtCore.QThread):
    data_received = QtCore.pyqtSignal(list)
    connection_lost = QtCore.pyqtSignal()

    def __init__(self, serial_conn=None, parent=None):
        super().__init__(parent)
        self.running = True
        self.serial_conn = serial_conn
        self.data_queue = queue.Queue(maxsize=10000)
        self.raw_buffer = bytearray()
        self.response_queue = queue.Queue()
        self.expected_response_size = None
        self.error_count = 0
        self.max_errors = 10

    def run(self):
        while self.running:
            try:
                if self.serial_conn and self.serial_conn.is_open:
                    raw_data = self.serial_conn.read(self.serial_conn.in_waiting)
                    if raw_data:
                        self.raw_buffer.extend(raw_data)
                        self.error_count = 0

                self.parse_buffer()
                QtCore.QThread.msleep(1)

            except serial.SerialException as e:
                self.error_count += 1
                print(f"Serial error: {e}")
                if self.error_count >= self.max_errors:
                    print("Too many serial errors, emitting connection_lost")
                    self.connection_lost.emit()
                    break
                QtCore.QThread.msleep(100)
            except Exception as e:
                print(f"Error di thread akuisisi: {e}")
                QtCore.QThread.msleep(10)

    def expect_response(self, size):
        self.expected_response_size = size

    def parse_buffer(self):
        while True:
            if len(self.raw_buffer) < 3:
                break

            header = struct.unpack("<H", self.raw_buffer[:2])[0]

            if header == 0xABCD:
                num_channel = self.raw_buffer[2]
                frame_size = 3 + num_channel * 4
                if len(self.raw_buffer) < frame_size:
                    break

                values = []
                offset = 3
                for _ in range(num_channel):
                    value = struct.unpack("<f", self.raw_buffer[offset : offset + 4])[0]
                    values.append(value)
                    offset += 4

                self.data_received.emit(values)
                self.raw_buffer = self.raw_buffer[frame_size:]

            elif header == 0xA55A:
                if self.expected_response_size is None:
                    break

                frame_size = 2 + self.expected_response_size
                if len(self.raw_buffer) < frame_size:
                    break

                payload = bytes(self.raw_buffer[2:frame_size])
                self.response_queue.put(payload)
                self.raw_buffer = self.raw_buffer[frame_size:]
                self.expected_response_size = None

            else:
                self.raw_buffer.pop(0)

    def stop(self):
        self.running = False
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
        self.wait()


class MotorSequenceThread(QtCore.QThread):
    def __init__(self, motor, mode, sequence, parent=None):
        super().__init__(parent)
        self.motor = motor
        self.mode = mode
        self.sequence = sequence
        self.running = True

    def run(self):
        m = self.motor
        m.set_foc_motor_mode(self.mode)

        for name in list(m.plotter_channels):
            m.plotter_remove_line(name)

        if self.mode == 0:
            for name in ["Is_ref", "id", "iq"]:
                m.plotter_add_line(name)
        elif self.mode == 1:
            for name in ["rpm_ref", "actual_rpm"]:
                m.plotter_add_line(name)
        elif self.mode == 2:
            for name in ["pos_ref", "actual_angle"]:
                m.plotter_add_line(name)

        for setpoint, delay in self.sequence:
            if not self.running:
                break
            if self.mode == 0:
                m.set_foc_current_set_point(setpoint)
            elif self.mode == 1:
                m.set_foc_speed_set_point(setpoint)
            elif self.mode == 2:
                m.set_foc_position_set_point(setpoint)
            self.msleep(int(delay * 1000))

    def stop(self):
        self.running = False


class LivePlotter(QtWidgets.QMainWindow):
    def __init__(self, max_points=1000, port=None, baudrate=115200):
        super().__init__()

        self.max_points = max_points
        self.serial_conn = None
        self.acq_thread = None
        self.motor = None
        self.is_connected = False
        self.current_port = None
        self.current_baudrate = None
        self.current_motor_mode = None
        self.parameters_loaded = False
        self.plot_host = "page"
        self.float_plot_window = None
        self.active_control_mode = 6
        self.control_mode_setpoints = {
            0: 0.0,
            1: 0.0,
            2: 0.0,
            6: 0.0,
        }
        self._syncing_plot_checks = False

        self.time_buffer = deque(maxlen=max_points)
        self.data_buffers = []
        self.counter = 0
        self.channels = {}
        self.used_colors = set()
        self.color_index = 0
        self.available_colors = [
            (218, 54, 51),
            (42, 101, 218),
            (26, 137, 23),
            (223, 126, 24),
            (120, 80, 200),
            (0, 145, 160),
            (205, 65, 120),
            (95, 120, 45),
            (0, 90, 120),
            (170, 80, 30),
            (90, 90, 90),
            (30, 30, 120),
        ]

        self.motor_sequence_thread = None
        self.console_namespace = {
            "plotter": self,
            "thread": self.acq_thread,
            "motor": self.motor,
            "np": np,
            "pg": pg,
        }

        self.plotter_dict = self.load_plotter_dict()

        self.setup_ui()
        self.setup_console_completion()
        self.populate_plot_channels()
        self.update_connection_ui()

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(10)

        self.fps_counter = 0
        self.fps_result = 0
        self.fps_timer = QtCore.QTimer()
        self.fps_timer.timeout.connect(self.update_fps)
        self.fps_timer.start(1000)

        if port:
            self.connect_serial(port, baudrate)
        else:
            QtCore.QTimer.singleShot(100, self.show_connection_dialog)

        self.log("LivePlotter initialized")

    def load_plotter_dict(self):
        config_path = Path(__file__).with_name("motor_commands.json")
        try:
            with open(config_path, "r") as f:
                config = json.load(f)
            return config.get("plotter_dict", {})
        except Exception as exc:
            print(f"Failed to load plotter dictionary: {exc}")
            return {}

    def setup_ui(self):
        self.setWindowTitle("sf-Motion VESC-Lite")
        self.setGeometry(100, 100, 1320, 760)
        self.setMinimumSize(980, 620)

        self.setStyleSheet(
            """
            QMainWindow, QWidget {
                background: #f4f5f7;
                color: #20242a;
                font-size: 10pt;
            }
            QToolBar {
                background: #20242a;
                spacing: 6px;
                padding: 6px;
                border: 0;
            }
            QToolButton {
                background: #f4f5f7;
                color: #20242a;
                border: 1px solid #c8cdd5;
                border-radius: 4px;
                padding: 6px 10px;
            }
            QToolButton:disabled {
                background: #555b64;
                color: #b8bdc5;
            }
            QPushButton {
                background: #ffffff;
                border: 1px solid #c8cdd5;
                border-radius: 4px;
                padding: 6px 10px;
            }
            QPushButton:hover {
                border-color: #6a7a90;
            }
            QPushButton:disabled {
                background: #e2e5e9;
                color: #8a929e;
            }
            QPushButton:checked {
                background: #265dd8;
                border-color: #1d48aa;
                color: #ffffff;
                font-weight: 600;
            }
            QPushButton#DangerButton, QToolButton#DangerButton {
                background: #a93632;
                border-color: #8d2b28;
                color: #ffffff;
                font-weight: 600;
            }
            QPushButton#ModeButton:checked {
                background: #1f7a55;
                border-color: #176142;
                color: #ffffff;
            }
            QPushButton#PresetButton:checked {
                background: #344052;
                border-color: #20242a;
                color: #ffffff;
            }
            QFrame#RealtimeStrip, QFrame#TerminalPanel {
                background: #ffffff;
                border-top: 1px solid #d6dae1;
            }
            QLabel#RealtimeName {
                color: #697282;
                font-size: 8pt;
            }
            QLabel#RealtimeValue {
                color: #20242a;
                font-weight: 700;
                font-size: 10pt;
            }
            QPushButton#PrimaryButton {
                background: #265dd8;
                border-color: #1d48aa;
                color: #ffffff;
                font-weight: 600;
            }
            QListWidget {
                background: #20242a;
                color: #dce1e8;
                border: 0;
                outline: 0;
                padding: 8px;
            }
            QListWidget::item {
                padding: 10px 12px;
                border-radius: 4px;
                margin: 2px 0;
            }
            QListWidget::item:selected {
                background: #344052;
                color: #ffffff;
            }
            QGroupBox {
                background: #ffffff;
                border: 1px solid #d6dae1;
                border-radius: 6px;
                margin-top: 16px;
                padding: 12px;
                font-weight: 600;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px;
            }
            QLineEdit, QDoubleSpinBox, QSpinBox, QComboBox, QPlainTextEdit {
                background: #ffffff;
                border: 1px solid #c8cdd5;
                border-radius: 4px;
                padding: 4px;
            }
            QLabel#MetricValue {
                font-size: 16pt;
                font-weight: 700;
            }
            QLabel#SubtleText {
                color: #697282;
            }
            """
        )

        self.setup_toolbar()

        central_widget = QtWidgets.QWidget()
        self.setCentralWidget(central_widget)
        outer_layout = QtWidgets.QVBoxLayout(central_widget)
        outer_layout.setContentsMargins(0, 0, 0, 0)
        outer_layout.setSpacing(0)

        root_layout = QtWidgets.QHBoxLayout()
        root_layout.setContentsMargins(0, 0, 0, 0)
        root_layout.setSpacing(0)
        outer_layout.addLayout(root_layout, 1)

        self.nav_list = QtWidgets.QListWidget()
        self.nav_list.setFixedWidth(180)
        self.nav_list.addItems(
            ["Dashboard", "Control", "Realtime Plot", "Commissioning", "Parameters", "Console"]
        )
        self.nav_list.currentRowChanged.connect(self.show_page)
        root_layout.addWidget(self.nav_list)

        self.pages = QtWidgets.QStackedWidget()
        root_layout.addWidget(self.pages, 1)

        self.side_plot_container = QtWidgets.QWidget()
        self.side_plot_container.setMinimumWidth(360)
        self.side_plot_container.setVisible(False)
        self.side_plot_layout = QtWidgets.QVBoxLayout(self.side_plot_container)
        self.side_plot_layout.setContentsMargins(8, 8, 8, 8)
        root_layout.addWidget(self.side_plot_container)

        self.dashboard_page = self.create_dashboard_page()
        self.control_page = self.create_control_page()
        self.plot_page = self.create_plot_page()
        self.commissioning_page = self.create_commissioning_page()
        self.parameters_page = self.create_parameters_page()
        self.console_page = self.create_console_page()

        for page in [
            self.dashboard_page,
            self.control_page,
            self.plot_page,
            self.commissioning_page,
            self.parameters_page,
            self.console_page,
        ]:
            self.pages.addWidget(page)

        self.nav_list.setCurrentRow(0)

        self.status_label = QtWidgets.QLabel("Not Connected")
        self.mode_status_label = QtWidgets.QLabel("Mode: --")
        self.sample_status_label = QtWidgets.QLabel("Samples: 0 | FPS: 0")
        self.statusBar().addWidget(self.status_label, 1)
        self.statusBar().addPermanentWidget(self.mode_status_label)
        self.statusBar().addPermanentWidget(self.sample_status_label)

        self.attach_plot_to_page()

        outer_layout.addWidget(self.create_realtime_strip())
        outer_layout.addWidget(self.create_terminal_panel())

    def setup_toolbar(self):
        toolbar = QtWidgets.QToolBar("Main")
        toolbar.setMovable(False)
        self.addToolBar(toolbar)

        self.connect_action = QtWidgets.QAction("Connect", self)
        self.connect_action.triggered.connect(self.show_connection_dialog)
        toolbar.addAction(self.connect_action)

        self.disconnect_action = QtWidgets.QAction("Disconnect", self)
        self.disconnect_action.triggered.connect(self.disconnect_serial)
        toolbar.addAction(self.disconnect_action)

        toolbar.addSeparator()

        self.disable_action = QtWidgets.QAction("Disable Motor", self)
        self.disable_action.triggered.connect(self.disable_motor)
        toolbar.addAction(self.disable_action)

        self.save_config_action = QtWidgets.QAction("Save Config", self)
        self.save_config_action.triggered.connect(self.save_config)
        toolbar.addAction(self.save_config_action)

        toolbar.addSeparator()

        self.clear_console_action = QtWidgets.QAction("Clear Console", self)
        self.clear_console_action.triggered.connect(lambda: self.console_output.clear())
        toolbar.addAction(self.clear_console_action)

        toolbar.addSeparator()

        self.side_plot_action = QtWidgets.QAction("Dock Plot Side", self)
        self.side_plot_action.setCheckable(True)
        self.side_plot_action.triggered.connect(self.toggle_side_plot)
        toolbar.addAction(self.side_plot_action)

        self.float_plot_action = QtWidgets.QAction("Float Plot", self)
        self.float_plot_action.setCheckable(True)
        self.float_plot_action.triggered.connect(self.toggle_float_plot)
        toolbar.addAction(self.float_plot_action)

    def page_widget(self):
        page = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(page)
        layout.setContentsMargins(18, 18, 18, 18)
        layout.setSpacing(14)
        return page, layout

    def create_dashboard_page(self):
        page, layout = self.page_widget()

        title = QtWidgets.QLabel("sf-Motion Control")
        title.setObjectName("MetricValue")
        layout.addWidget(title)

        metrics = QtWidgets.QHBoxLayout()
        self.connection_metric = self.metric_box("Connection", "Disconnected")
        self.mode_metric = self.metric_box("Motor Mode", "--")
        self.plot_metric = self.metric_box("Plot Channels", "0")
        metrics.addWidget(self.connection_metric["box"])
        metrics.addWidget(self.mode_metric["box"])
        metrics.addWidget(self.plot_metric["box"])
        layout.addLayout(metrics)

        quick_group = QtWidgets.QGroupBox("Quick Actions")
        quick_layout = QtWidgets.QHBoxLayout(quick_group)
        disable_btn = QtWidgets.QPushButton("Disable Motor")
        disable_btn.setObjectName("DangerButton")
        disable_btn.clicked.connect(self.disable_motor)
        quick_layout.addWidget(disable_btn)

        control_btn = QtWidgets.QPushButton("Open Control")
        control_btn.clicked.connect(lambda: self.nav_list.setCurrentRow(1))
        quick_layout.addWidget(control_btn)

        plot_btn = QtWidgets.QPushButton("Open Realtime Plot")
        plot_btn.clicked.connect(lambda: self.nav_list.setCurrentRow(2))
        quick_layout.addWidget(plot_btn)
        quick_layout.addStretch()
        layout.addWidget(quick_group)

        note = QtWidgets.QLabel(
            "Use the disable control before changing wiring, closing the app, or changing bench setup. "
            "Disconnect now attempts disable automatically while the serial link is still available."
        )
        note.setWordWrap(True)
        note.setObjectName("SubtleText")
        layout.addWidget(note)
        layout.addStretch()
        return page

    def metric_box(self, label, value):
        box = QtWidgets.QGroupBox(label)
        layout = QtWidgets.QVBoxLayout(box)
        value_label = QtWidgets.QLabel(value)
        value_label.setObjectName("MetricValue")
        layout.addWidget(value_label)
        return {"box": box, "value": value_label}

    def create_control_page(self):
        page, layout = self.page_widget()

        group = QtWidgets.QGroupBox("Motor Control")
        group_layout = QtWidgets.QGridLayout(group)

        self.mode_buttons = {}
        self.mode_button_group = QtWidgets.QButtonGroup(self)
        self.mode_button_group.setExclusive(True)
        for col, (label, mode) in enumerate(CONTROL_MODES.items()):
            button = QtWidgets.QPushButton(label)
            button.setObjectName("ModeButton")
            button.setCheckable(True)
            if mode == 6:
                button.setChecked(True)
            self.mode_button_group.addButton(button, mode)
            self.mode_buttons[mode] = button
            group_layout.addWidget(button, 0, col)
        self.mode_button_group.buttonClicked[int].connect(self.on_control_mode_changed)

        self.setpoint_spin = QtWidgets.QDoubleSpinBox()
        self.setpoint_spin.setRange(-1000000.0, 1000000.0)
        self.setpoint_spin.setDecimals(4)
        self.setpoint_spin.setSingleStep(0.05)
        self.setpoint_spin.setKeyboardTracking(False)
        self.setpoint_spin.valueChanged.connect(self.on_setpoint_value_changed)
        group_layout.addWidget(QtWidgets.QLabel("Setpoint"), 1, 0)
        group_layout.addWidget(self.setpoint_spin, 1, 1, 1, 2)

        self.setpoint_unit_label = QtWidgets.QLabel("")
        group_layout.addWidget(self.setpoint_unit_label, 1, 3)

        self.apply_setpoint_button = QtWidgets.QPushButton("Apply Setpoint")
        self.apply_setpoint_button.setObjectName("PrimaryButton")
        self.apply_setpoint_button.clicked.connect(self.apply_setpoint)
        group_layout.addWidget(self.apply_setpoint_button, 2, 0, 1, 2)

        self.zero_setpoint_button = QtWidgets.QPushButton("Zero Setpoint")
        self.zero_setpoint_button.clicked.connect(self.zero_setpoint)
        group_layout.addWidget(self.zero_setpoint_button, 2, 2)

        self.control_disable_button = QtWidgets.QPushButton("Disable Motor")
        self.control_disable_button.setObjectName("DangerButton")
        self.control_disable_button.clicked.connect(self.disable_motor)
        group_layout.addWidget(self.control_disable_button, 2, 3)

        layout.addWidget(group)

        relative_group = QtWidgets.QGroupBox("Relative Position")
        relative_layout = QtWidgets.QGridLayout(relative_group)
        self.relative_position_spin = QtWidgets.QDoubleSpinBox()
        self.relative_position_spin.setRange(-360000.0, 360000.0)
        self.relative_position_spin.setDecimals(3)
        self.relative_position_spin.setSingleStep(5.0)
        self.relative_position_spin.setSuffix(" deg")
        relative_layout.addWidget(QtWidgets.QLabel("Move from latest actual_angle"), 0, 0)
        relative_layout.addWidget(self.relative_position_spin, 0, 1)
        move_relative_btn = QtWidgets.QPushButton("Move Relative")
        move_relative_btn.clicked.connect(self.move_relative_position)
        relative_layout.addWidget(move_relative_btn, 0, 2)
        layout.addWidget(relative_group)

        self.control_status = QtWidgets.QLabel("Disconnected")
        self.control_status.setWordWrap(True)
        self.control_status.setObjectName("SubtleText")
        layout.addWidget(self.control_status)
        layout.addStretch()

        self.on_control_mode_changed(6)
        return page

    def create_plot_page(self):
        page, layout = self.page_widget()

        splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        layout.addWidget(splitter, 1)

        self.plot_page_host = QtWidgets.QWidget()
        self.plot_page_host_layout = QtWidgets.QVBoxLayout(self.plot_page_host)
        self.plot_page_host_layout.setContentsMargins(0, 0, 0, 0)

        self.plot_panel = QtWidgets.QWidget()
        self.plot_panel_layout = QtWidgets.QVBoxLayout(self.plot_panel)
        self.plot_panel_layout.setContentsMargins(0, 0, 0, 0)

        self.plot_location_label = QtWidgets.QLabel("Realtime Plot")
        self.plot_location_label.setObjectName("SubtleText")
        self.plot_panel_layout.addWidget(self.plot_location_label)

        self.plot_widget = pg.PlotWidget()
        self.plot_widget.setBackground("w")
        self.plot_widget.setLabel("left", "Value")
        self.plot_widget.setLabel("bottom", "Sample")
        self.plot_widget.showGrid(x=True, y=True, alpha=0.3)
        self.plot_widget.addLegend()
        self.plot_panel_layout.addWidget(self.plot_widget, 1)

        control_layout = QtWidgets.QHBoxLayout()
        self.clear_button = QtWidgets.QPushButton("Clear")
        self.clear_button.clicked.connect(self.clear_data)
        control_layout.addWidget(self.clear_button)

        self.pause_button = QtWidgets.QPushButton("Pause")
        self.pause_button.clicked.connect(self.toggle_pause)
        self.paused = False
        control_layout.addWidget(self.pause_button)

        self.auto_range_check = QtWidgets.QCheckBox("Auto Range")
        self.auto_range_check.setChecked(True)
        control_layout.addWidget(self.auto_range_check)

        self.info_label = QtWidgets.QLabel("Samples: 0 | FPS: 0")
        control_layout.addStretch()
        control_layout.addWidget(self.info_label)
        self.plot_panel_layout.addLayout(control_layout)

        splitter.addWidget(self.plot_page_host)

        side_panel = QtWidgets.QWidget()
        side_layout = QtWidgets.QVBoxLayout(side_panel)
        side_layout.setContentsMargins(10, 0, 0, 0)

        presets_group = QtWidgets.QGroupBox("Presets")
        presets_layout = QtWidgets.QGridLayout(presets_group)
        self.preset_buttons = {}
        for idx, name in enumerate(PLOT_PRESETS):
            button = QtWidgets.QPushButton(name)
            button.setObjectName("PresetButton")
            button.setCheckable(True)
            button.clicked.connect(lambda checked=False, preset=name: self.apply_plot_preset(preset))
            presets_layout.addWidget(button, idx // 2, idx % 2)
            self.preset_buttons[name] = button
        side_layout.addWidget(presets_group)

        channels_group = QtWidgets.QGroupBox("Channels")
        channels_layout = QtWidgets.QVBoxLayout(channels_group)
        self.channel_scroll = QtWidgets.QScrollArea()
        self.channel_scroll.setWidgetResizable(True)
        self.channel_container = QtWidgets.QWidget()
        self.channel_layout = QtWidgets.QVBoxLayout(self.channel_container)
        self.channel_layout.setContentsMargins(4, 4, 4, 4)
        self.channel_scroll.setWidget(self.channel_container)
        channels_layout.addWidget(self.channel_scroll)
        side_layout.addWidget(channels_group, 1)

        self.remove_channels_button = QtWidgets.QPushButton("Remove All Channels")
        self.remove_channels_button.clicked.connect(self.remove_all_plot_channels)
        side_layout.addWidget(self.remove_channels_button)

        splitter.addWidget(side_panel)
        splitter.setSizes([880, 300])
        return page

    def create_commissioning_page(self):
        page, layout = self.page_widget()

        prep_group = QtWidgets.QGroupBox("Commissioning")
        prep_layout = QtWidgets.QGridLayout(prep_group)

        self.pole_pairs_spin = QtWidgets.QSpinBox()
        self.pole_pairs_spin.setRange(1, 255)
        self.pole_pairs_spin.setValue(7)
        prep_layout.addWidget(QtWidgets.QLabel("Pole pairs"), 0, 0)
        prep_layout.addWidget(self.pole_pairs_spin, 0, 1)

        set_sensored_btn = QtWidgets.QPushButton("Set Sensored FOC")
        set_sensored_btn.clicked.connect(lambda: self.safe_motor_call("set_foc_mode", 0))
        prep_layout.addWidget(set_sensored_btn, 1, 0)

        set_pole_btn = QtWidgets.QPushButton("Set Pole Pairs")
        set_pole_btn.clicked.connect(self.set_pole_pairs)
        prep_layout.addWidget(set_pole_btn, 1, 1)

        run_btn = QtWidgets.QPushButton("Run Self Commissioning")
        run_btn.setObjectName("PrimaryButton")
        run_btn.clicked.connect(self.run_self_commissioning)
        prep_layout.addWidget(run_btn, 2, 0)

        read_btn = QtWidgets.QPushButton("Read Motor Params")
        read_btn.clicked.connect(self.read_motor_params)
        prep_layout.addWidget(read_btn, 2, 1)

        save_btn = QtWidgets.QPushButton("Save Config")
        save_btn.clicked.connect(self.save_config)
        prep_layout.addWidget(save_btn, 3, 0)

        disable_btn = QtWidgets.QPushButton("Disable Motor")
        disable_btn.setObjectName("DangerButton")
        disable_btn.clicked.connect(self.disable_motor)
        prep_layout.addWidget(disable_btn, 3, 1)

        layout.addWidget(prep_group)

        warning = QtWidgets.QLabel(
            "Self commissioning starts resistance and inductance measurement, calculates current PI gains, "
            "and starts encoder calibration. The helper returns after starting encoder calibration; it does "
            "not verify calibration completion or save configuration."
        )
        warning.setWordWrap(True)
        warning.setObjectName("SubtleText")
        layout.addWidget(warning)

        self.commissioning_status = QtWidgets.QPlainTextEdit()
        self.commissioning_status.setReadOnly(True)
        self.commissioning_status.setMaximumHeight(180)
        layout.addWidget(self.commissioning_status)
        layout.addStretch()
        return page

    def create_parameters_page(self):
        page, layout = self.page_widget()

        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        container = QtWidgets.QWidget()
        self.parameters_layout = QtWidgets.QVBoxLayout(container)
        self.parameters_layout.setSpacing(12)
        scroll.setWidget(container)
        layout.addWidget(scroll)

        self.parameter_widgets = {}
        self.add_parameter_group(
            "Motor Identity",
            [
                ("pole_pairs", "Pole pairs", "int"),
                ("kv", "Kv", "float"),
                ("rs", "Rs", "float"),
                ("ld", "Ld", "float"),
                ("lq", "Lq", "float"),
                ("flux_linkage", "Flux linkage", "float"),
            ],
        )
        self.add_parameter_group(
            "Current PI",
            [
                ("pid_id.kp", "Id Kp", "float"),
                ("pid_id.ki", "Id Ki", "float"),
                ("pid_id.deadband", "Id deadband", "float"),
                ("pid_iq.kp", "Iq Kp", "float"),
                ("pid_iq.ki", "Iq Ki", "float"),
                ("pid_iq.deadband", "Iq deadband", "float"),
            ],
        )
        self.add_parameter_group(
            "Speed and Position PID",
            [
                ("pid_speed.kp", "Speed Kp", "float"),
                ("pid_speed.ki", "Speed Ki", "float"),
                ("pid_speed.out_max", "Speed out max", "float"),
                ("pid_speed.deadband", "Speed deadband", "float"),
                ("pid_position.kp", "Position Kp", "float"),
                ("pid_position.ki", "Position Ki", "float"),
                ("pid_position.kd", "Position Kd", "float"),
                ("pid_position.out_max", "Position out max", "float"),
                ("pid_position.deadband", "Position deadband", "float"),
                ("pid_position.d_fc", "Position D fc", "float"),
            ],
        )
        self.add_parameter_group(
            "Advanced",
            [
                ("field_weakening.kp", "FW Kp", "float"),
                ("field_weakening.ki", "FW Ki", "float"),
                ("field_weakening.out_min", "FW out min", "float"),
                ("field_weakening_enable", "Field weakening enabled", "bool"),
                ("mtpa_enable", "MTPA enabled", "bool"),
            ],
        )

        self.set_original_pid_values()

        buttons = QtWidgets.QHBoxLayout()
        read_all_btn = QtWidgets.QPushButton("Read All Parameters")
        read_all_btn.clicked.connect(self.read_all_parameters)
        buttons.addWidget(read_all_btn)

        restore_pid_btn = QtWidgets.QPushButton("Use Original PID Values")
        restore_pid_btn.clicked.connect(self.set_original_pid_values)
        buttons.addWidget(restore_pid_btn)

        apply_original_pid_btn = QtWidgets.QPushButton("Apply Original PID")
        apply_original_pid_btn.clicked.connect(self.apply_original_pid_values)
        buttons.addWidget(apply_original_pid_btn)

        apply_all_btn = QtWidgets.QPushButton("Apply All Parameters")
        apply_all_btn.setObjectName("PrimaryButton")
        apply_all_btn.clicked.connect(self.apply_all_parameters)
        buttons.addWidget(apply_all_btn)

        save_btn = QtWidgets.QPushButton("Save Config")
        save_btn.clicked.connect(self.save_config)
        buttons.addWidget(save_btn)
        buttons.addStretch()
        self.parameters_layout.addLayout(buttons)
        self.parameters_layout.addStretch()
        return page

    def create_realtime_strip(self):
        frame = QtWidgets.QFrame()
        frame.setObjectName("RealtimeStrip")
        layout = QtWidgets.QHBoxLayout(frame)
        layout.setContentsMargins(12, 6, 12, 6)
        layout.setSpacing(18)

        self.realtime_labels = {}
        for key, title, unit in [
            ("id", "ID", "A"),
            ("iq", "IQ", "A"),
            ("actual_rpm", "RPM", ""),
            ("actual_angle", "ANGLE", "deg"),
            ("v_bus", "VBUS", "V"),
        ]:
            box = QtWidgets.QWidget()
            box_layout = QtWidgets.QVBoxLayout(box)
            box_layout.setContentsMargins(0, 0, 0, 0)
            box_layout.setSpacing(0)
            name = QtWidgets.QLabel(title)
            name.setObjectName("RealtimeName")
            value = QtWidgets.QLabel(f"-- {unit}".rstrip())
            value.setObjectName("RealtimeValue")
            box_layout.addWidget(name)
            box_layout.addWidget(value)
            layout.addWidget(box)
            self.realtime_labels[key] = (value, unit)

        layout.addStretch()
        return frame

    def create_terminal_panel(self):
        frame = QtWidgets.QFrame()
        frame.setObjectName("TerminalPanel")
        layout = QtWidgets.QVBoxLayout(frame)
        layout.setContentsMargins(8, 6, 8, 8)
        layout.setSpacing(4)

        header = QtWidgets.QHBoxLayout()
        title = QtWidgets.QLabel("Terminal")
        title.setObjectName("SubtleText")
        header.addWidget(title)
        header.addStretch()

        clear_btn = QtWidgets.QPushButton("Clear")
        clear_btn.clicked.connect(lambda: self.console_output.clear())
        header.addWidget(clear_btn)
        layout.addLayout(header)

        self.console_output = QtWidgets.QPlainTextEdit()
        self.console_output.setReadOnly(True)
        self.console_output.setMaximumHeight(140)
        layout.addWidget(self.console_output)

        self.console_input = QtWidgets.QLineEdit()
        self.console_input.setPlaceholderText(">>>")
        self.console_input.returnPressed.connect(self.execute_command)
        layout.addWidget(self.console_input)
        return frame

    def add_parameter_group(self, title, fields):
        group = QtWidgets.QGroupBox(title)
        layout = QtWidgets.QGridLayout(group)
        for idx, (key, label, kind) in enumerate(fields):
            layout.addWidget(QtWidgets.QLabel(label), idx, 0)
            if kind == "bool":
                widget = QtWidgets.QCheckBox()
            elif kind == "int":
                widget = QtWidgets.QSpinBox()
                widget.setRange(-2147483648, 2147483647)
            else:
                widget = QtWidgets.QDoubleSpinBox()
                widget.setRange(-1000000000.0, 1000000000.0)
                widget.setDecimals(8)
                widget.setSingleStep(0.01)
                widget.setKeyboardTracking(False)
            layout.addWidget(widget, idx, 1)
            self.parameter_widgets[key] = widget
        self.parameters_layout.addWidget(group)

    def create_console_page(self):
        page, layout = self.page_widget()

        title = QtWidgets.QLabel("Terminal")
        title.setObjectName("MetricValue")
        layout.addWidget(title)

        message = QtWidgets.QLabel(
            "The interactive Python console is docked at the bottom of every page. "
            "Use this page when you want more space above it for command output."
        )
        message.setWordWrap(True)
        message.setObjectName("SubtleText")
        layout.addWidget(message)

        focus_btn = QtWidgets.QPushButton("Focus Terminal Input")
        focus_btn.clicked.connect(lambda: self.console_input.setFocus())
        layout.addWidget(focus_btn)
        layout.addStretch()
        return page

    def detach_plot_panel(self):
        parent = self.plot_panel.parentWidget()
        if parent is not None and parent.layout() is not None:
            parent.layout().removeWidget(self.plot_panel)
        self.plot_panel.setParent(None)

    def attach_plot_to_page(self):
        self.detach_plot_panel()
        self.plot_page_host_layout.addWidget(self.plot_panel)
        self.side_plot_container.setVisible(False)
        if self.float_plot_window is not None:
            self.float_plot_window.hide()
        self.plot_host = "page"
        self.plot_location_label.setText("Realtime Plot")
        self.side_plot_action.setChecked(False)
        self.float_plot_action.setChecked(False)

    def attach_plot_to_side(self):
        self.detach_plot_panel()
        self.side_plot_layout.addWidget(self.plot_panel)
        self.side_plot_container.setVisible(True)
        if self.float_plot_window is not None:
            self.float_plot_window.hide()
        self.plot_host = "side"
        self.plot_location_label.setText("Realtime Plot - Side Dock")
        self.side_plot_action.setChecked(True)
        self.float_plot_action.setChecked(False)

    def attach_plot_to_float(self):
        self.detach_plot_panel()
        if self.float_plot_window is None:
            self.float_plot_window = QtWidgets.QDialog(self)
            self.float_plot_window.setWindowTitle("sf-Motion Realtime Plot")
            self.float_plot_window.resize(900, 620)
            float_layout = QtWidgets.QVBoxLayout(self.float_plot_window)
            float_layout.setContentsMargins(8, 8, 8, 8)
            self.float_plot_window.finished.connect(self.on_float_plot_closed)
        self.float_plot_window.layout().addWidget(self.plot_panel)
        self.side_plot_container.setVisible(False)
        self.float_plot_window.show()
        self.float_plot_window.raise_()
        self.plot_host = "float"
        self.plot_location_label.setText("Realtime Plot - Floating Window")
        self.side_plot_action.setChecked(False)
        self.float_plot_action.setChecked(True)

    def toggle_side_plot(self, checked):
        if checked:
            self.attach_plot_to_side()
        else:
            self.attach_plot_to_page()

    def toggle_float_plot(self, checked):
        if checked:
            self.attach_plot_to_float()
        else:
            self.attach_plot_to_page()

    def on_float_plot_closed(self):
        if self.plot_host == "float":
            self.attach_plot_to_page()

    def show_page(self, index):
        self.pages.setCurrentIndex(index)

    def log(self, message):
        print(message)
        if hasattr(self, "console_output"):
            self.console_output.appendPlainText(str(message))

    def echo_command(self, command):
        if hasattr(self, "console_output"):
            self.console_output.appendPlainText(f">>> {command}")

    def setup_console_completion(self):
        words = self.build_completion()
        completer = QtWidgets.QCompleter(words, self)
        completer.setCompletionMode(QtWidgets.QCompleter.PopupCompletion)
        completer.setCaseSensitivity(QtCore.Qt.CaseInsensitive)
        completer.setFilterMode(QtCore.Qt.MatchContains)
        self.console_input.setCompleter(completer)

    def build_completion(self):
        words = []
        for name, obj in self.console_namespace.items():
            if obj is not None:
                words.append(name)
                for attr in dir(obj):
                    if not attr.startswith("_"):
                        words.append(f"{name}.{attr}")
        return sorted(words)

    def execute_command(self):
        cmd = self.console_input.text().strip()
        if not cmd:
            return

        self.console_output.appendPlainText(f">>> {cmd}")
        try:
            buffer = io.StringIO()
            with contextlib.redirect_stdout(buffer):
                try:
                    result = eval(cmd, globals(), self.console_namespace)
                    if result is not None:
                        print(result)
                except SyntaxError:
                    exec(cmd, globals(), self.console_namespace)
            out = buffer.getvalue()
            if out:
                self.console_output.appendPlainText(out.rstrip())
        except Exception:
            self.console_output.appendPlainText(traceback.format_exc())
        self.console_input.clear()

    def show_connection_dialog(self):
        dialog = SerialPortDialog(self)
        if dialog.exec_() == QtWidgets.QDialog.Accepted:
            port = dialog.get_selected_port()
            baudrate = dialog.get_baudrate()
            if port:
                self.connect_serial(port, baudrate)

    def connect_serial(self, port, baudrate):
        try:
            if self.is_connected:
                self.disconnect_serial()

            self.serial_conn = serial.Serial(port, baudrate, timeout=0.001)
            self.log(f"Connected to {port} at {baudrate} baud")

            self.acq_thread = DataAcquisitionThread(serial_conn=self.serial_conn)
            self.acq_thread.data_received.connect(self.on_data_received)
            self.acq_thread.connection_lost.connect(self.handle_connection_lost)
            self.acq_thread.start()

            self.motor = MotorProtocol(self.serial_conn, self.acq_thread)
            self.plotter_dict = self.motor.plotter_dict
            self.console_namespace["thread"] = self.acq_thread
            self.console_namespace["motor"] = self.motor
            self.setup_console_completion()

            self.is_connected = True
            self.current_port = port
            self.current_baudrate = baudrate

            self.populate_plot_channels()
            self.refresh_motor_mode()
            self.apply_default_plot_for_mode()
            self.update_connection_ui()
            self.log("Serial connection established successfully")

        except serial.SerialException as e:
            QtWidgets.QMessageBox.critical(
                self,
                "Connection Error",
                f"Failed to connect to {port}:\n{str(e)}",
            )
            self.status_label.setText(f"Connection failed: {str(e)}")
            self.log(f"Connection error: {e}")
            self.update_connection_ui()

    def disconnect_serial(self):
        if self.is_connected:
            self.try_disable_motor(reason="disconnect")

        if self.acq_thread:
            self.acq_thread.stop()
            self.acq_thread = None

        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
        self.serial_conn = None

        self.is_connected = False
        self.current_motor_mode = None
        self.console_namespace["thread"] = None
        self.console_namespace["motor"] = None
        self.setup_console_completion()
        self.update_connection_ui()
        self.log("Disconnected from serial port")

    def handle_connection_lost(self):
        QtWidgets.QMessageBox.warning(
            self,
            "Connection Lost",
            "Serial connection has been lost. Please reconnect.",
        )
        self.is_connected = False
        self.disconnect_serial()

    def require_motor(self):
        if not self.is_connected or self.motor is None:
            self.log("No motor connected")
            return None
        return self.motor

    def safe_motor_call(self, method_name, *args):
        motor = self.require_motor()
        if motor is None:
            return None
        try:
            method = getattr(motor, method_name)
            arg_text = ", ".join(repr(arg) for arg in args)
            self.echo_command(f"motor.{method_name}({arg_text})")
            result = method(*args)
            self.log(f"{method_name}{args} -> {result}")
            if method_name in ("set_foc_motor_mode", "get_foc_motor_mode"):
                self.refresh_motor_mode()
            return result
        except Exception:
            self.log(traceback.format_exc())
            return None

    def try_disable_motor(self, reason="manual"):
        if not self.motor or not self.serial_conn or not self.serial_conn.is_open:
            return False
        try:
            if self.current_motor_mode == 0:
                self.echo_command("motor.set_foc_current_set_point(0.0)")
                self.motor.set_foc_current_set_point(0.0)
                QtWidgets.QApplication.processEvents()
                time.sleep(0.05)
            elif self.current_motor_mode == 1:
                self.echo_command("motor.set_foc_speed_set_point(0.0)")
                self.motor.set_foc_speed_set_point(0.0)
                QtWidgets.QApplication.processEvents()
                time.sleep(0.05)
            self.echo_command("motor.set_foc_motor_mode(6)")
            result = self.motor.set_foc_motor_mode(6)
            self.current_motor_mode = 6
            self.log(f"Motor disabled during {reason}: {result}")
            self.update_mode_ui()
            return True
        except Exception as exc:
            self.log(f"Could not disable motor during {reason}: {exc}")
            return False

    def disable_motor(self):
        if self.try_disable_motor(reason="manual"):
            self.mode_buttons[6].setChecked(True)
            self.on_control_mode_changed(6)
        else:
            self.log("Disable motor command was not sent")

    def save_config(self):
        self.safe_motor_call("save_config")

    def refresh_motor_mode(self):
        motor = self.require_motor()
        if motor is None:
            return None
        try:
            response = motor.get_foc_motor_mode()
            self.current_motor_mode = response.get("mode")
            self.update_mode_ui()
            return self.current_motor_mode
        except Exception:
            self.log(traceback.format_exc())
            return None

    def update_mode_ui(self):
        label = MODE_LABELS.get(self.current_motor_mode, "--")
        self.mode_status_label.setText(f"Mode: {label}")
        self.mode_metric["value"].setText(label)
        if self.current_motor_mode in self.mode_buttons:
            self.mode_buttons[self.current_motor_mode].setChecked(True)
            self.on_control_mode_changed(self.current_motor_mode)

    def update_connection_ui(self):
        connected = self.is_connected
        self.connect_action.setEnabled(not connected)
        self.disconnect_action.setEnabled(connected)
        self.disable_action.setEnabled(connected)
        self.save_config_action.setEnabled(connected)

        controls = [
            self.apply_setpoint_button,
            self.zero_setpoint_button,
            self.control_disable_button,
            self.remove_channels_button,
        ]
        for widget in controls:
            widget.setEnabled(connected)

        for button in self.mode_buttons.values():
            button.setEnabled(connected)

        if connected:
            text = f"Connected to {self.current_port} at {self.current_baudrate} baud"
            self.status_label.setText(text)
            self.connection_metric["value"].setText("Connected")
            self.control_status.setText(text)
        else:
            self.status_label.setText("Disconnected")
            self.mode_status_label.setText("Mode: --")
            self.connection_metric["value"].setText("Disconnected")
            self.mode_metric["value"].setText("--")
            self.control_status.setText("Disconnected")

    def apply_default_plot_for_mode(self):
        if self.current_motor_mode == 0:
            self.apply_plot_preset("Current")
        elif self.current_motor_mode == 1:
            self.apply_plot_preset("Speed")
        elif self.current_motor_mode == 2:
            self.apply_plot_preset("Position")

    def on_control_mode_changed(self, mode):
        if hasattr(self, "setpoint_spin") and self.active_control_mode in self.control_mode_setpoints:
            self.control_mode_setpoints[self.active_control_mode] = self.setpoint_spin.value()

        self.active_control_mode = mode
        self.setpoint_spin.blockSignals(True)
        if mode == 0:
            self.setpoint_unit_label.setText("A")
            self.setpoint_spin.setRange(-100.0, 100.0)
            self.setpoint_spin.setDecimals(4)
            self.setpoint_spin.setSingleStep(0.01)
        elif mode == 1:
            self.setpoint_unit_label.setText("RPM")
            self.setpoint_spin.setRange(-100000.0, 100000.0)
            self.setpoint_spin.setDecimals(2)
            self.setpoint_spin.setSingleStep(10.0)
        elif mode == 2:
            self.setpoint_unit_label.setText("deg relative")
            self.setpoint_spin.setRange(-360000.0, 360000.0)
            self.setpoint_spin.setDecimals(3)
            self.setpoint_spin.setSingleStep(5.0)
        else:
            self.setpoint_unit_label.setText("")
            self.setpoint_spin.setRange(0.0, 0.0)
            self.setpoint_spin.setDecimals(1)
            self.setpoint_spin.setSingleStep(1.0)
        self.setpoint_spin.setValue(self.control_mode_setpoints.get(mode, 0.0))
        self.setpoint_spin.blockSignals(False)

    def on_setpoint_value_changed(self, value):
        self.control_mode_setpoints[self.active_control_mode] = value

    def selected_control_mode(self):
        checked = self.mode_button_group.checkedButton()
        if checked is None:
            return 6
        return self.mode_button_group.id(checked)

    def apply_setpoint(self):
        motor = self.require_motor()
        if motor is None:
            return

        mode = self.selected_control_mode()
        value = self.setpoint_spin.value()

        try:
            if mode == 6:
                self.disable_motor()
                return

            if self.current_motor_mode != mode:
                self.echo_command(f"motor.set_foc_motor_mode({mode})")
                result = motor.set_foc_motor_mode(mode)
                self.log(f"set_foc_motor_mode({mode}) -> {result}")
                self.current_motor_mode = mode

            if mode == 0:
                self.echo_command(f"motor.set_foc_current_set_point({value!r})")
                result = motor.set_foc_current_set_point(value)
            elif mode == 1:
                self.echo_command(f"motor.set_foc_speed_set_point({value!r})")
                result = motor.set_foc_speed_set_point(value)
            elif mode == 2:
                current = self.latest_channel_value("actual_angle")
                if current is None:
                    self.log(
                        "No actual_angle samples available. "
                        "Add the Position preset and wait for data before applying a relative move."
                    )
                    self.apply_plot_preset("Position")
                    return
                target = current + value
                self.echo_command("current = plotter.channels['actual_angle']['buffer'][-1]")
                self.echo_command(f"motor.set_foc_position_set_point(current + {value!r})")
                result = motor.set_foc_position_set_point(target)
            else:
                self.log("Selected control mode cannot accept setpoints")
                return

            if mode == 2:
                self.log(
                    f"Applied Position relative move {value} deg "
                    f"from {current} deg -> target {target} deg: {result}"
                )
            else:
                self.log(f"Applied {MODE_LABELS[mode]} setpoint {value}: {result}")
            self.update_mode_ui()
        except Exception:
            self.log(traceback.format_exc())

    def zero_setpoint(self):
        mode = self.selected_control_mode()
        self.setpoint_spin.setValue(0.0)
        if mode in (0, 1, 2):
            self.apply_setpoint()
        elif mode == 6:
            self.disable_motor()

    def move_relative_position(self):
        motor = self.require_motor()
        if motor is None:
            return

        angle = self.latest_channel_value("actual_angle")
        if angle is None:
            self.log("No actual_angle samples available. Add the Position preset and wait for data.")
            self.apply_plot_preset("Position")
            return

        delta = self.relative_position_spin.value()
        self.mode_buttons[2].setChecked(True)
        self.on_control_mode_changed(2)
        self.setpoint_spin.setValue(delta)
        self.apply_setpoint()

    def latest_channel_value(self, name):
        channel = self.channels.get(name)
        if not channel:
            return None
        buffer = channel.get("buffer")
        if not buffer:
            return None
        for value in reversed(buffer):
            if not np.isnan(value):
                return float(value)
        return None

    def update_realtime_values(self):
        if not hasattr(self, "realtime_labels"):
            return
        for name, (label, unit) in self.realtime_labels.items():
            value = self.latest_channel_value(name)
            if value is None:
                label.setText(f"-- {unit}".rstrip())
            else:
                label.setText(f"{value:.3f} {unit}".rstrip())

    def populate_plot_channels(self):
        if not hasattr(self, "channel_layout"):
            return

        while self.channel_layout.count():
            item = self.channel_layout.takeAt(0)
            widget = item.widget()
            if widget:
                widget.deleteLater()

        self.channel_checks = {}
        for name in sorted(self.plotter_dict):
            check = QtWidgets.QCheckBox(name)
            check.toggled.connect(lambda checked, channel=name: self.on_channel_toggled(channel, checked))
            self.channel_layout.addWidget(check)
            self.channel_checks[name] = check
        self.channel_layout.addStretch()
        self.sync_channel_checks()

    def on_channel_toggled(self, channel, checked):
        if self._syncing_plot_checks:
            return
        motor = self.require_motor()
        if motor is None:
            self.sync_channel_checks()
            return

        try:
            if checked:
                self.echo_command(f"motor.plotter_add_line({channel!r})")
                result = motor.plotter_add_line(channel)
            else:
                self.echo_command(f"motor.plotter_remove_line({channel!r})")
                result = motor.plotter_remove_line(channel)
            self.log(f"plot {channel} {'on' if checked else 'off'} -> {result}")
            self.sync_channel_checks()
        except Exception:
            self.log(traceback.format_exc())
            self.sync_channel_checks()

    def sync_channel_checks(self):
        if not hasattr(self, "channel_checks"):
            return
        self._syncing_plot_checks = True
        active = set(self.motor.plotter_channels if self.motor else [])
        for name, check in self.channel_checks.items():
            check.setChecked(name in active)
        self._syncing_plot_checks = False
        if hasattr(self, "plot_metric"):
            self.plot_metric["value"].setText(str(len(active)))

    def apply_plot_preset(self, preset_name):
        motor = self.require_motor()
        if motor is None:
            return

        names = [name for name in PLOT_PRESETS[preset_name] if name in self.plotter_dict]
        self.remove_all_plot_channels()
        for name in names:
            try:
                self.echo_command(f"motor.plotter_add_line({name!r})")
                motor.plotter_add_line(name)
            except Exception:
                self.log(traceback.format_exc())
        self.sync_channel_checks()
        for name, button in self.preset_buttons.items():
            button.setChecked(name == preset_name)
        self.log(f"Applied plot preset: {preset_name}")

    def remove_all_plot_channels(self):
        motor = self.require_motor()
        if motor is None:
            return

        for name in list(motor.plotter_channels):
            try:
                self.echo_command(f"motor.plotter_remove_line({name!r})")
                motor.plotter_remove_line(name)
            except Exception as exc:
                self.log(f"Failed to remove plot channel {name}: {exc}")
        if hasattr(self, "preset_buttons"):
            for button in self.preset_buttons.values():
                button.setChecked(False)
        self.sync_channel_checks()

    def get_next_color(self):
        for color in self.available_colors:
            if color not in self.used_colors:
                self.used_colors.add(color)
                return color

        color = self.available_colors[self.color_index % len(self.available_colors)]
        self.color_index += 1
        return color

    def release_color(self, color):
        if color in self.used_colors:
            self.used_colors.remove(color)

    def on_data_received(self, values):
        if self.paused or not self.is_connected:
            return

        self.counter += 1
        self.time_buffer.append(self.counter)

        for name in list(self.channels.keys()):
            if name not in self.motor.plotter_channels:
                ch = self.channels.pop(name)
                ch["line"].clear()
                self.plot_widget.removeItem(ch["line"])
                if "color" in ch:
                    self.release_color(ch["color"])
                if name in self.data_buffers:
                    self.data_buffers.remove(name)

        for name in self.motor.plotter_channels:
            if name not in self.channels:
                buffer = deque([np.nan] * (len(self.time_buffer) - 1), maxlen=self.max_points)
                self.data_buffers.append(buffer)

                color = self.get_next_color()
                pen = pg.mkPen(color=color, width=1.5)
                line = self.plot_widget.plot([], [], pen=pen, name=name)
                self.channels[name] = {
                    "line": line,
                    "buffer": buffer,
                    "color": color,
                }

        for idx, name in enumerate(self.motor.plotter_channels):
            if idx < len(values):
                self.channels[name]["buffer"].append(values[idx])
            else:
                self.channels[name]["buffer"].append(np.nan)

        self.status_label.setText(f"Received {len(values)} values")
        self.sync_channel_checks()
        self.update_realtime_values()

    def update_plot(self):
        if self.paused or not self.is_connected or self.motor is None:
            return

        x = np.asarray(self.time_buffer)
        for name in self.motor.plotter_channels:
            if name not in self.channels:
                continue
            ch = self.channels[name]
            y = np.asarray(ch["buffer"])
            n = min(len(x), len(y))
            if n == 0:
                continue
            ch["line"].setData(x[-n:], y[-n:])

        if len(self.time_buffer) > 0:
            x_min = max(0, self.counter - self.max_points)
            x_max = self.counter
            self.plot_widget.setXRange(x_min, x_max, padding=0.05)

        if self.auto_range_check.isChecked():
            self.auto_range()

        text = f"Samples: {len(self.time_buffer)} | FPS: {self.fps_result}"
        self.info_label.setText(text)
        self.sample_status_label.setText(text)
        self.fps_counter += 1

    def auto_range(self):
        if len(self.time_buffer) == 0 or self.motor is None:
            return

        all_values = []
        for name in self.motor.plotter_channels:
            if name in self.channels:
                all_values.extend(self.channels[name]["buffer"])

        if not all_values:
            return

        all_values = np.asarray(all_values, dtype=float)
        all_values = all_values[~np.isnan(all_values)]
        if all_values.size == 0:
            return

        y_min = np.min(all_values)
        y_max = np.max(all_values)
        padding = max(1, (y_max - y_min) * 0.1)
        self.plot_widget.setYRange(y_min - padding, y_max + padding)

    def update_fps(self):
        self.fps_result = self.fps_counter
        self.fps_counter = 0

    def clear_data(self):
        for name in self.channels:
            self.channels[name]["buffer"].clear()
        self.time_buffer.clear()
        self.counter = 0

        for name in self.channels:
            self.channels[name]["line"].setData([], [])

        self.update_realtime_values()
        self.log("Data cleared")

    def toggle_pause(self):
        self.paused = not self.paused
        self.pause_button.setText("Resume" if self.paused else "Pause")
        self.log(f"Plot {'paused' if self.paused else 'resumed'}")

    def set_pole_pairs(self):
        self.safe_motor_call("set_pole_pairs", self.pole_pairs_spin.value())

    def run_self_commissioning(self):
        motor = self.require_motor()
        if motor is None:
            return
        try:
            self.echo_command("motor.self_commissioning()")
            buffer = io.StringIO()
            with contextlib.redirect_stdout(buffer):
                motor.self_commissioning()
            out = buffer.getvalue().strip()
            if out:
                self.commissioning_status.appendPlainText(out)
                self.console_output.appendPlainText(out)
            self.log("Self commissioning command sequence started")
        except Exception:
            text = traceback.format_exc()
            self.commissioning_status.appendPlainText(text)
            self.log(text)

    def read_motor_params(self):
        motor = self.require_motor()
        if motor is None:
            return
        try:
            for command in [
                "motor.get_pole_pairs()",
                "motor.get_rs()",
                "motor.get_ld()",
                "motor.get_lq()",
                "motor.get_kv()",
                "motor.get_flux_linkage()",
            ]:
                self.echo_command(command)
            values = {
                "pole_pairs": motor.get_pole_pairs(),
                "rs": motor.get_rs(),
                "ld": motor.get_ld(),
                "lq": motor.get_lq(),
                "kv": motor.get_kv(),
                "flux_linkage": motor.get_flux_linkage(),
            }
            for key, value in values.items():
                self.commissioning_status.appendPlainText(f"{key}: {value}")
            self.read_all_parameters()
        except Exception:
            text = traceback.format_exc()
            self.commissioning_status.appendPlainText(text)
            self.log(text)

    def read_all_parameters(self):
        motor = self.require_motor()
        if motor is None:
            return
        try:
            for command in [
                "motor.get_pole_pairs()",
                "motor.get_kv()",
                "motor.get_rs()",
                "motor.get_ld()",
                "motor.get_lq()",
                "motor.get_flux_linkage()",
                "motor.get_pid_id()",
                "motor.get_pid_iq()",
                "motor.get_pid_speed()",
                "motor.get_pid_position()",
                "motor.get_field_weakening()",
                "motor.get_field_weakening_enable()",
                "motor.get_mtpa_enable()",
            ]:
                self.echo_command(command)
            self.set_parameter_value("pole_pairs", motor.get_pole_pairs()["value"])
            self.set_parameter_value("kv", motor.get_kv()["value"])
            self.set_parameter_value("rs", motor.get_rs()["value"])
            self.set_parameter_value("ld", motor.get_ld()["value"])
            self.set_parameter_value("lq", motor.get_lq()["value"])
            self.set_parameter_value("flux_linkage", motor.get_flux_linkage()["value"])

            self.set_parameter_group("pid_id", motor.get_pid_id())
            self.set_parameter_group("pid_iq", motor.get_pid_iq())
            self.set_parameter_group("pid_speed", motor.get_pid_speed())
            self.set_parameter_group("pid_position", motor.get_pid_position())
            self.set_parameter_group("field_weakening", motor.get_field_weakening())
            self.set_parameter_value(
                "field_weakening_enable",
                bool(motor.get_field_weakening_enable()["enable"]),
            )
            self.set_parameter_value("mtpa_enable", bool(motor.get_mtpa_enable()["enable"]))
            self.parameters_loaded = True
            self.log("Read all parameters")
        except Exception:
            self.log(traceback.format_exc())

    def set_original_pid_values(self):
        for key, value in ORIGINAL_PID_VALUES.items():
            self.set_parameter_value(key, value)
        self.log("Loaded original firmware PID values into the parameter fields")

    def set_parameter_group(self, prefix, values):
        for key, value in values.items():
            self.set_parameter_value(f"{prefix}.{key}", value)

    def set_parameter_value(self, key, value):
        widget = self.parameter_widgets[key]
        if isinstance(widget, QtWidgets.QCheckBox):
            widget.setChecked(bool(value))
        else:
            widget.setValue(value)

    def parameter_value(self, key):
        widget = self.parameter_widgets[key]
        if isinstance(widget, QtWidgets.QCheckBox):
            return 1 if widget.isChecked() else 0
        return widget.value()

    def apply_all_parameters(self):
        motor = self.require_motor()
        if motor is None:
            return
        if not self.parameters_loaded:
            self.log(
                "Apply All Parameters blocked. Press Read All Parameters first, "
                "or use Apply Original PID to write only the original PID values."
            )
            return
        try:
            self.echo_command(f"motor.set_pole_pairs({int(self.parameter_value('pole_pairs'))!r})")
            motor.set_pole_pairs(int(self.parameter_value("pole_pairs")))
            self.echo_command(f"motor.set_kv({self.parameter_value('kv')!r})")
            motor.set_kv(self.parameter_value("kv"))
            self.echo_command(f"motor.set_rs({self.parameter_value('rs')!r})")
            motor.set_rs(self.parameter_value("rs"))
            self.echo_command(f"motor.set_ld({self.parameter_value('ld')!r})")
            motor.set_ld(self.parameter_value("ld"))
            self.echo_command(f"motor.set_lq({self.parameter_value('lq')!r})")
            motor.set_lq(self.parameter_value("lq"))
            self.echo_command(f"motor.set_flux_linkage({self.parameter_value('flux_linkage')!r})")
            motor.set_flux_linkage(self.parameter_value("flux_linkage"))
            self.echo_command(
                "motor.set_pid_id("
                f"{self.parameter_value('pid_id.kp')!r}, "
                f"{self.parameter_value('pid_id.ki')!r}, "
                f"{self.parameter_value('pid_id.deadband')!r})"
            )
            motor.set_pid_id(
                self.parameter_value("pid_id.kp"),
                self.parameter_value("pid_id.ki"),
                self.parameter_value("pid_id.deadband"),
            )
            self.echo_command(
                "motor.set_pid_iq("
                f"{self.parameter_value('pid_iq.kp')!r}, "
                f"{self.parameter_value('pid_iq.ki')!r}, "
                f"{self.parameter_value('pid_iq.deadband')!r})"
            )
            motor.set_pid_iq(
                self.parameter_value("pid_iq.kp"),
                self.parameter_value("pid_iq.ki"),
                self.parameter_value("pid_iq.deadband"),
            )
            self.echo_command(
                "motor.set_pid_speed("
                f"{self.parameter_value('pid_speed.kp')!r}, "
                f"{self.parameter_value('pid_speed.ki')!r}, "
                f"{self.parameter_value('pid_speed.out_max')!r}, "
                f"{self.parameter_value('pid_speed.deadband')!r})"
            )
            motor.set_pid_speed(
                self.parameter_value("pid_speed.kp"),
                self.parameter_value("pid_speed.ki"),
                self.parameter_value("pid_speed.out_max"),
                self.parameter_value("pid_speed.deadband"),
            )
            self.echo_command(
                "motor.set_pid_position("
                f"{self.parameter_value('pid_position.kp')!r}, "
                f"{self.parameter_value('pid_position.ki')!r}, "
                f"{self.parameter_value('pid_position.kd')!r}, "
                f"{self.parameter_value('pid_position.out_max')!r}, "
                f"{self.parameter_value('pid_position.deadband')!r}, "
                f"{self.parameter_value('pid_position.d_fc')!r})"
            )
            motor.set_pid_position(
                self.parameter_value("pid_position.kp"),
                self.parameter_value("pid_position.ki"),
                self.parameter_value("pid_position.kd"),
                self.parameter_value("pid_position.out_max"),
                self.parameter_value("pid_position.deadband"),
                self.parameter_value("pid_position.d_fc"),
            )
            self.echo_command(
                "motor.set_field_weakening("
                f"{self.parameter_value('field_weakening.kp')!r}, "
                f"{self.parameter_value('field_weakening.ki')!r}, "
                f"{self.parameter_value('field_weakening.out_min')!r})"
            )
            motor.set_field_weakening(
                self.parameter_value("field_weakening.kp"),
                self.parameter_value("field_weakening.ki"),
                self.parameter_value("field_weakening.out_min"),
            )
            self.echo_command(
                f"motor.set_field_weakening_enable({self.parameter_value('field_weakening_enable')!r})"
            )
            motor.set_field_weakening_enable(self.parameter_value("field_weakening_enable"))
            self.echo_command(f"motor.set_mtpa_enable({self.parameter_value('mtpa_enable')!r})")
            motor.set_mtpa_enable(self.parameter_value("mtpa_enable"))
            self.log("Applied all parameters")
        except Exception:
            self.log(traceback.format_exc())

    def apply_original_pid_values(self):
        motor = self.require_motor()
        if motor is None:
            return
        try:
            self.set_original_pid_values()
            self.echo_command(
                "motor.set_pid_id("
                f"{ORIGINAL_PID_VALUES['pid_id.kp']!r}, "
                f"{ORIGINAL_PID_VALUES['pid_id.ki']!r}, "
                f"{ORIGINAL_PID_VALUES['pid_id.deadband']!r})"
            )
            motor.set_pid_id(
                ORIGINAL_PID_VALUES["pid_id.kp"],
                ORIGINAL_PID_VALUES["pid_id.ki"],
                ORIGINAL_PID_VALUES["pid_id.deadband"],
            )
            self.echo_command(
                "motor.set_pid_iq("
                f"{ORIGINAL_PID_VALUES['pid_iq.kp']!r}, "
                f"{ORIGINAL_PID_VALUES['pid_iq.ki']!r}, "
                f"{ORIGINAL_PID_VALUES['pid_iq.deadband']!r})"
            )
            motor.set_pid_iq(
                ORIGINAL_PID_VALUES["pid_iq.kp"],
                ORIGINAL_PID_VALUES["pid_iq.ki"],
                ORIGINAL_PID_VALUES["pid_iq.deadband"],
            )
            self.echo_command(
                "motor.set_pid_speed("
                f"{ORIGINAL_PID_VALUES['pid_speed.kp']!r}, "
                f"{ORIGINAL_PID_VALUES['pid_speed.ki']!r}, "
                f"{ORIGINAL_PID_VALUES['pid_speed.out_max']!r}, "
                f"{ORIGINAL_PID_VALUES['pid_speed.deadband']!r})"
            )
            motor.set_pid_speed(
                ORIGINAL_PID_VALUES["pid_speed.kp"],
                ORIGINAL_PID_VALUES["pid_speed.ki"],
                ORIGINAL_PID_VALUES["pid_speed.out_max"],
                ORIGINAL_PID_VALUES["pid_speed.deadband"],
            )
            self.echo_command(
                "motor.set_pid_position("
                f"{ORIGINAL_PID_VALUES['pid_position.kp']!r}, "
                f"{ORIGINAL_PID_VALUES['pid_position.ki']!r}, "
                f"{ORIGINAL_PID_VALUES['pid_position.kd']!r}, "
                f"{ORIGINAL_PID_VALUES['pid_position.out_max']!r}, "
                f"{ORIGINAL_PID_VALUES['pid_position.deadband']!r}, "
                f"{ORIGINAL_PID_VALUES['pid_position.d_fc']!r})"
            )
            motor.set_pid_position(
                ORIGINAL_PID_VALUES["pid_position.kp"],
                ORIGINAL_PID_VALUES["pid_position.ki"],
                ORIGINAL_PID_VALUES["pid_position.kd"],
                ORIGINAL_PID_VALUES["pid_position.out_max"],
                ORIGINAL_PID_VALUES["pid_position.deadband"],
                ORIGINAL_PID_VALUES["pid_position.d_fc"],
            )
            self.echo_command(
                "motor.set_field_weakening("
                f"{ORIGINAL_PID_VALUES['field_weakening.kp']!r}, "
                f"{ORIGINAL_PID_VALUES['field_weakening.ki']!r}, "
                f"{ORIGINAL_PID_VALUES['field_weakening.out_min']!r})"
            )
            motor.set_field_weakening(
                ORIGINAL_PID_VALUES["field_weakening.kp"],
                ORIGINAL_PID_VALUES["field_weakening.ki"],
                ORIGINAL_PID_VALUES["field_weakening.out_min"],
            )
            self.echo_command("motor.set_field_weakening_enable(0)")
            motor.set_field_weakening_enable(0)
            self.echo_command("motor.set_mtpa_enable(0)")
            motor.set_mtpa_enable(0)
            self.log("Applied original firmware PID values")
        except Exception:
            self.log(traceback.format_exc())

    def closeEvent(self, event):
        self.log("Closing application...")
        self.disconnect_serial()

        if hasattr(self, "timer"):
            self.timer.stop()
        if hasattr(self, "fps_timer"):
            self.fps_timer.stop()

        event.accept()

    def run_motor_sequence(self, mode, sequence):
        if not self.is_connected or self.motor is None:
            self.log("No motor connected")
            return

        if self.motor_sequence_thread is not None and self.motor_sequence_thread.isRunning():
            self.log("Motor sequence is already running")
            return

        self.motor_sequence_thread = MotorSequenceThread(
            motor=self.motor,
            mode=mode,
            sequence=sequence,
            parent=self,
        )
        self.motor_sequence_thread.finished.connect(self.on_motor_sequence_finished)
        self.motor_sequence_thread.start()

    def on_motor_sequence_finished(self):
        self.log("Motor sequence finished")
        self.motor_sequence_thread.deleteLater()
        self.motor_sequence_thread = None

    def run_motor_speed_demo(self):
        sequence = (
            [(speed, 0.02) for speed in range(0, 1001, 10)]
            + [(1000, 2)]
            + [(speed, 0.02) for speed in range(1000, -1, -10)]
            + [(0, 2)]
            + [(speed, 0.02) for speed in range(0, -1001, -10)]
            + [(-1000, 2)]
            + [(speed, 0.02) for speed in range(-1000, 1, 10)]
        )
        self.run_motor_sequence(1, sequence)

    def run_motor_position_demo(self):
        home = self.latest_channel_value("actual_angle")
        if home is None:
            self.log("No actual_angle samples available for position demo")
            return
        sequence = [
            (home, 1),
            (home + 90, 1),
            (home + 180, 1),
            (home + 360, 1),
            (home, 1),
            (home - 45, 1),
            (home - 180, 1),
            (home, 1),
        ]
        self.run_motor_sequence(2, sequence)

    def run_motor_current_demo(self):
        sequence = [
            (0, 1),
            (0.05, 2),
            (0, 1),
            (-0.05, 2),
            (0, 1),
        ]
        self.run_motor_sequence(0, sequence)
