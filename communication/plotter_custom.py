import contextlib
import io
import json
import math
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
from gui_theme import SPACE, stylesheet, style_plot, terminal_font, ui_font
from gui_tooltips import apply_action_tooltips, apply_window_tooltips, parameter_tip, tip, update_control_tooltips


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

class ParameterExportDialog(QtWidgets.QDialog):
    def __init__(self, sections, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Export Parameters")
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(16, 16, 16, 16)
        layout.setSpacing(12)
        layout.addWidget(QtWidgets.QLabel("Choose sections to export:"))
        self.section_checks = {}
        for title in sections:
            checkbox = QtWidgets.QCheckBox(title)
            checkbox.setToolTip(tip("Export section checkboxes"))
            checkbox.setChecked(True)
            checkbox.toggled.connect(self.update_export_button)
            self.section_checks[title] = checkbox
            layout.addWidget(checkbox)
        self.buttons = QtWidgets.QDialogButtonBox(
            QtWidgets.QDialogButtonBox.Save | QtWidgets.QDialogButtonBox.Cancel
        )
        self.buttons.button(QtWidgets.QDialogButtonBox.Save).setText("Export")
        self.buttons.accepted.connect(self.accept)
        self.buttons.rejected.connect(self.reject)
        layout.addWidget(self.buttons)

    def selected_sections(self):
        return [title for title, checkbox in self.section_checks.items() if checkbox.isChecked()]

    def update_export_button(self):
        if hasattr(self, "buttons"):
            self.buttons.button(QtWidgets.QDialogButtonBox.Save).setEnabled(
                bool(self.selected_sections())
            )


class SerialPortDialog(QtWidgets.QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Select Serial Port")
        self.setModal(True)
        self.setFixedWidth(420)

        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(16, 16, 16, 16)
        layout.setSpacing(8)
        port_label = QtWidgets.QLabel("Select Serial Port:")
        layout.addWidget(port_label)

        self.port_combo = QtWidgets.QComboBox()
        self.port_combo.setPlaceholderText("Select port...")
        port_label.setBuddy(self.port_combo)
        layout.addWidget(self.port_combo)

        refresh_btn = QtWidgets.QPushButton("Refresh Ports")
        refresh_btn.clicked.connect(self.refresh_ports)
        layout.addWidget(refresh_btn)

        baud_label = QtWidgets.QLabel("Baudrate:")
        layout.addWidget(baud_label)
        self.baud_combo = QtWidgets.QComboBox()
        baud_label.setBuddy(self.baud_combo)
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

        self.port_combo.setToolTip(tip("Serial port selector"))
        port_label.setToolTip(tip("Serial port selector"))
        self.baud_combo.setToolTip(tip("Baudrate"))
        baud_label.setToolTip(tip("Baudrate"))
        apply_action_tooltips(self)

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
    data_received = QtCore.pyqtSignal(int, float)
    connection_lost = QtCore.pyqtSignal()

    def __init__(self, serial_conn=None, parent=None):
        super().__init__(parent)
        self.running = True
        self.serial_conn = serial_conn
        self.data_queue = queue.Queue(maxsize=10000)
        self.raw_buffer = bytearray()
        self.response_queue = queue.Queue()
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

    def parse_buffer(self):
        while True:
            if len(self.raw_buffer) < 3:
                break

            if self.raw_buffer[:2] == b"\xA5\xA5":
                payload_length = self.raw_buffer[2]
                frame_size = 3 + payload_length
                if len(self.raw_buffer) < frame_size:
                    break

                payload = bytes(self.raw_buffer[3:frame_size])
                self.raw_buffer = self.raw_buffer[frame_size:]

                if len(payload) < 3:
                    continue

                com_type, node_id, address = payload[:3]
                data = payload[3:]
                if com_type == 0x00:
                    self.response_queue.put((node_id, address, data))
                elif com_type == 0x05 and len(data) == 4:
                    self.data_received.emit(address, struct.unpack("<f", data)[0])
                elif com_type == 0x06:
                    self.response_queue.put((node_id, address, data))

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

        self.setFont(ui_font())
        self.setStyleSheet(stylesheet())

        self.setup_toolbar()

        central_widget = QtWidgets.QWidget()
        self.setCentralWidget(central_widget)
        outer_layout = QtWidgets.QVBoxLayout(central_widget)
        outer_layout.setContentsMargins(0, 0, 0, 0)
        outer_layout.setSpacing(0)

        self.terminal_splitter = QtWidgets.QSplitter(QtCore.Qt.Vertical)
        self.terminal_splitter.setObjectName("TerminalSplitter")
        self.terminal_splitter.setAccessibleName("Workspace and terminal height")
        self.terminal_splitter.setChildrenCollapsible(False)
        self.terminal_splitter.setHandleWidth(10)
        outer_layout.addWidget(self.terminal_splitter)

        workspace = QtWidgets.QWidget()
        workspace_layout = QtWidgets.QVBoxLayout(workspace)
        workspace_layout.setContentsMargins(0, 0, 0, 0)
        workspace_layout.setSpacing(0)
        root_container = QtWidgets.QWidget()
        root_layout = QtWidgets.QHBoxLayout(root_container)
        root_layout.setContentsMargins(0, 0, 0, 0)
        root_layout.setSpacing(0)
        workspace_layout.addWidget(root_container, 1)
        self.terminal_splitter.addWidget(workspace)

        navigation = QtWidgets.QFrame()
        navigation.setObjectName("Navigation")
        navigation.setFixedWidth(184)
        navigation_layout = QtWidgets.QVBoxLayout(navigation)
        navigation_layout.setContentsMargins(8, 16, 8, 8)
        navigation_layout.setSpacing(8)
        brand = QtWidgets.QLabel("sf-Motion")
        brand.setObjectName("Brand")
        navigation_layout.addWidget(brand)
        caption = QtWidgets.QLabel("MOTOR WORKSPACE")
        caption.setObjectName("BrandCaption")
        navigation_layout.addWidget(caption)
        self.nav_list = QtWidgets.QListWidget()
        self.nav_list.setObjectName("NavigationList")
        self.nav_list.setAccessibleName("Workspace pages")
        self.nav_list.setVerticalScrollMode(QtWidgets.QAbstractItemView.ScrollPerPixel)
        self.nav_list.addItems(
            ["Dashboard", "Control", "Realtime Plot", "Commissioning", "Parameters", "Console"]
        )
        self.nav_list.currentRowChanged.connect(self.show_page)
        self.nav_list.currentRowChanged.connect(
            lambda row: self.nav_list.scrollToItem(self.nav_list.item(row))
        )
        navigation_layout.addWidget(self.nav_list, 1)
        root_layout.addWidget(navigation)

        self.pages = QtWidgets.QStackedWidget()
        root_layout.addWidget(self.pages, 1)

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

        workspace_layout.addWidget(self.create_realtime_strip())
        self.terminal_panel = self.create_terminal_panel()
        self.terminal_splitter.addWidget(self.terminal_panel)
        self.terminal_splitter.setStretchFactor(0, 1)
        self.terminal_splitter.setStretchFactor(1, 0)
        self.terminal_splitter.setSizes([500, 160])
        self.terminal_splitter.handle(1).setToolTip("Drag up or down to resize the terminal")
        apply_window_tooltips(self)

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
        toolbar.widgetForAction(self.disable_action).setObjectName("DangerButton")

        self.save_config_action = QtWidgets.QAction("Save Config", self)
        self.save_config_action.triggered.connect(self.save_config)
        toolbar.addAction(self.save_config_action)

        toolbar.addSeparator()

        self.clear_console_action = QtWidgets.QAction("Clear Console", self)
        self.clear_console_action.triggered.connect(lambda: self.console_output.clear())
        toolbar.addAction(self.clear_console_action)

        toolbar.addSeparator()

        self.float_plot_action = QtWidgets.QAction("Float Plot", self)
        self.float_plot_action.setCheckable(True)
        self.float_plot_action.triggered.connect(self.toggle_float_plot)
        toolbar.addAction(self.float_plot_action)

    def page_widget(self, title, description, scrollable=False):
        page = QtWidgets.QWidget()
        page.setObjectName("Page")
        outer = QtWidgets.QVBoxLayout(page)
        outer.setContentsMargins(0, 0, 0, 0)
        if scrollable:
            scroll = QtWidgets.QScrollArea()
            scroll.setWidgetResizable(True)
            scroll.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
            content = QtWidgets.QWidget()
            content.setObjectName("PageContent")
            layout = QtWidgets.QVBoxLayout(content)
            scroll.setWidget(content)
            outer.addWidget(scroll)
        else:
            layout = outer
        layout.setContentsMargins(SPACE["page"], SPACE["section"], SPACE["page"], SPACE["section"])
        layout.setSpacing(SPACE["medium"])
        heading = QtWidgets.QLabel(title)
        heading.setObjectName("PageTitle")
        layout.addWidget(heading)
        subtitle = QtWidgets.QLabel(description)
        subtitle.setWordWrap(True)
        subtitle.setObjectName("SubtleText")
        layout.addWidget(subtitle)
        return page, layout

    def create_dashboard_page(self):
        page, layout = self.page_widget(
            "Dashboard", "Connection, motor state, and shortcuts for your bench session.", True
        )

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
        page, layout = self.page_widget(
            'Control', 'Set motor mode and setpoints, or command a relative position move.', True
        )

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
        field_label = QtWidgets.QLabel("Setpoint")
        field_label.setBuddy(self.setpoint_spin)
        group_layout.addWidget(field_label, 1, 0)
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
        field_label = QtWidgets.QLabel("Move from latest actual_angle")
        field_label.setBuddy(self.relative_position_spin)
        relative_layout.addWidget(field_label, 0, 0)
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
        page, layout = self.page_widget(
            'Realtime Plot', 'Inspect live signals. Choose a preset or select individual channels.', True
        )

        self.plot_page_host = QtWidgets.QWidget()
        self.plot_page_host_layout = QtWidgets.QVBoxLayout(self.plot_page_host)
        self.plot_page_host_layout.setContentsMargins(0, 0, 0, 0)

        layout.addWidget(self.plot_page_host, 1)
        self.plot_workspace = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        self.plot_workspace.setMinimumHeight(300)

        self.plot_panel = QtWidgets.QWidget()
        self.plot_panel_layout = QtWidgets.QVBoxLayout(self.plot_panel)
        self.plot_panel_layout.setContentsMargins(0, 0, 0, 0)

        self.plot_location_label = QtWidgets.QLabel("Realtime Plot")
        self.plot_location_label.setObjectName("SubtleText")
        self.plot_panel_layout.addWidget(self.plot_location_label)

        self.plot_widget = pg.PlotWidget()
        self.plot_widget.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Ignored)
        self.plot_widget.setBackground("w")
        self.plot_widget.setLabel("left", "Value")
        self.plot_widget.setLabel("bottom", "Sample")
        self.plot_widget.showGrid(x=True, y=True, alpha=0.3)
        self.plot_widget.addLegend()
        style_plot(self.plot_widget)
        self.plot_panel_layout.addWidget(self.plot_widget, 1)

        control_layout = QtWidgets.QGridLayout()
        self.clear_button = QtWidgets.QPushButton("Clear")
        self.clear_button.clicked.connect(self.clear_data)
        control_layout.addWidget(self.clear_button, 0, 0)

        self.pause_button = QtWidgets.QPushButton("Pause")
        self.pause_button.clicked.connect(self.toggle_pause)
        self.paused = False
        control_layout.addWidget(self.pause_button, 0, 1)

        self.auto_range_check = QtWidgets.QCheckBox("Auto Range")
        self.auto_range_check.setChecked(True)
        control_layout.addWidget(self.auto_range_check, 0, 2)

        self.info_label = QtWidgets.QLabel("Samples: 0 | FPS: 0")
        control_layout.setColumnStretch(2, 1)
        control_layout.addWidget(self.info_label, 1, 0, 1, 3, QtCore.Qt.AlignRight)
        self.plot_panel_layout.addLayout(control_layout)

        self.plot_workspace.addWidget(self.plot_panel)

        side_panel = QtWidgets.QWidget()
        side_layout = QtWidgets.QVBoxLayout(side_panel)
        side_layout.setContentsMargins(SPACE["medium"], 0, 0, 0)
        side_panel.setMinimumWidth(224)

        presets_group = QtWidgets.QGroupBox("Presets")
        presets_group.setSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Fixed)
        presets_layout = QtWidgets.QGridLayout(presets_group)
        presets_layout.setSizeConstraint(QtWidgets.QLayout.SetMinimumSize)
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

        self.plot_workspace.addWidget(side_panel)
        self.plot_workspace.setSizes([880, 300])
        return page

    def create_commissioning_page(self):
        page, layout = self.page_widget(
            'Commissioning', 'Prepare the motor and run the existing measurement and calibration helpers.', True
        )

        prep_group = QtWidgets.QGroupBox("Commissioning")
        prep_layout = QtWidgets.QGridLayout(prep_group)

        self.pole_pairs_spin = QtWidgets.QSpinBox()
        self.pole_pairs_spin.setRange(1, 255)
        self.pole_pairs_spin.setValue(7)
        field_label = QtWidgets.QLabel("Pole pairs")
        field_label.setBuddy(self.pole_pairs_spin)
        prep_layout.addWidget(field_label, 0, 0)
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
        prep_layout.addWidget(read_btn, 3, 0)

        preparation = QtWidgets.QLabel(
            "Before commissioning: connect motor phases and DC power, secure the motor, "
            "and leave the shaft clear. Wait for calibration movement to finish before saving."
        )
        preparation.setWordWrap(True)
        preparation.setObjectName("SubtleText")
        prep_layout.addWidget(preparation, 2, 1)

        save_btn = QtWidgets.QPushButton("Save Config")
        save_btn.clicked.connect(self.save_config)
        prep_layout.addWidget(save_btn, 3, 1)

        disable_btn = QtWidgets.QPushButton("Disable Motor")
        disable_btn.setObjectName("DangerButton")
        disable_btn.clicked.connect(self.disable_motor)
        prep_layout.addWidget(disable_btn, 4, 0, 1, 2)

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
        page, layout = self.page_widget(
            'Parameters', 'Read, review, and apply motor settings. Save Config persists applied values.', False
        )

        scroll = QtWidgets.QScrollArea()
        scroll.setObjectName("ParameterScroll")
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        container = QtWidgets.QWidget()
        self.parameters_layout = QtWidgets.QVBoxLayout(container)
        self.parameters_layout.setSpacing(SPACE["section"])
        self.parameters_layout.setContentsMargins(0, 0, SPACE["small"], 0)
        scroll.setWidget(container)
        layout.addWidget(scroll, 1)

        self.parameter_widgets = {}
        self.parameter_sections = {}
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

        self.parameter_actions = QtWidgets.QFrame()
        self.parameter_actions.setObjectName("ParameterActions")
        buttons = QtWidgets.QGridLayout(self.parameter_actions)
        buttons.setContentsMargins(12, 8, 12, 8)
        buttons.setSpacing(8)
        read_all_btn = QtWidgets.QPushButton("Read All Parameters")
        read_all_btn.clicked.connect(self.read_all_parameters)
        buttons.addWidget(read_all_btn, 0, 0)

        export_btn = QtWidgets.QPushButton("Export Parameters")
        export_btn.clicked.connect(self.export_parameters)
        buttons.addWidget(export_btn, 0, 2)

        import_btn = QtWidgets.QPushButton("Import Parameters")
        import_btn.clicked.connect(self.import_parameters)
        buttons.addWidget(import_btn, 0, 1)

        apply_all_btn = QtWidgets.QPushButton("Apply All Parameters")
        apply_all_btn.setObjectName("PrimaryButton")
        apply_all_btn.clicked.connect(self.apply_all_parameters)
        buttons.addWidget(apply_all_btn, 1, 0)

        save_btn = QtWidgets.QPushButton("Save Config")
        save_btn.clicked.connect(self.save_config)
        buttons.addWidget(save_btn, 1, 1)
        layout.addWidget(self.parameter_actions)
        self.parameters_layout.addStretch()
        QtWidgets.QWidget.setTabOrder(read_all_btn, import_btn)
        QtWidgets.QWidget.setTabOrder(import_btn, export_btn)
        QtWidgets.QWidget.setTabOrder(export_btn, apply_all_btn)
        QtWidgets.QWidget.setTabOrder(apply_all_btn, save_btn)
        return page

    def create_realtime_strip(self):
        frame = QtWidgets.QFrame()
        frame.setObjectName("RealtimeStrip")
        layout = QtWidgets.QHBoxLayout(frame)
        layout.setContentsMargins(16, 8, 16, 8)
        layout.setSpacing(24)

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
        layout.setContentsMargins(16, 8, 16, 8)
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
        self.console_output.setObjectName("TerminalOutput")
        self.console_output.setFont(terminal_font())
        self.console_output.setAccessibleName("Terminal output")
        self.console_output.setReadOnly(True)
        self.terminal_scroll_timer = QtCore.QTimer(self)
        self.terminal_scroll_timer.setSingleShot(True)
        self.terminal_scroll_timer.timeout.connect(self.scroll_terminal_to_bottom)
        self.console_output.verticalScrollBar().rangeChanged.connect(
            lambda _minimum, _maximum: self.scroll_terminal_to_bottom()
        )
        self.console_output.setMinimumHeight(48)
        layout.addWidget(self.console_output, 1)

        self.console_input = QtWidgets.QLineEdit()
        self.console_input.setObjectName("TerminalInput")
        self.console_input.setFont(terminal_font())
        self.console_input.setAccessibleName("Python command")
        self.console_input.setPlaceholderText(">>>")
        self.console_input.returnPressed.connect(self.execute_command)
        layout.addWidget(self.console_input)
        return frame

    def add_parameter_group(self, title, fields):
        self.parameter_sections[title] = [key for key, _, _ in fields]
        group = QtWidgets.QGroupBox(title)
        if title == "Current PI":
            group.setToolTip(tip("Current PI group"))
        layout = QtWidgets.QGridLayout(group)
        layout.setColumnStretch(0, 1)
        layout.setColumnStretch(1, 1)
        layout.setVerticalSpacing(8)
        for idx, (key, label, kind) in enumerate(fields):
            field_label = QtWidgets.QLabel(label)
            layout.addWidget(field_label, idx, 0)
            if kind == "bool":
                widget = QtWidgets.QCheckBox()
            elif kind == "int":
                widget = QtWidgets.QSpinBox()
                widget.setRange(-2147483648, 2147483647)
            else:
                widget = QtWidgets.QDoubleSpinBox()
                widget.setRange(-1000000000.0, 1000000000.0)
                widget.setDecimals(8)
                if key.startswith(("pid_", "field_weakening.")):
                    if key.endswith(".out_min"):
                        widget.setMaximum(0.0)
                    else:
                        widget.setMinimum(0.00000001 if key.endswith(".d_fc") else 0.0)
                widget.setSingleStep(0.01)
                widget.setKeyboardTracking(False)
            field_label.setBuddy(widget)
            field_label.setToolTip(parameter_tip(key))
            widget.setToolTip(parameter_tip(key))
            widget.setAccessibleName(label)
            layout.addWidget(widget, idx, 1)
            self.parameter_widgets[key] = widget
        self.parameters_layout.addWidget(group)

    def create_console_page(self):
        page, layout = self.page_widget(
            'Console', 'Run Python commands in the persistent terminal below.', True
        )

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

    def detach_plot_workspace(self):
        parent = self.plot_workspace.parentWidget()
        if parent is not None and parent.layout() is not None:
            parent.layout().removeWidget(self.plot_workspace)
        self.plot_workspace.setParent(None)

    def attach_plot_to_page(self):
        self.detach_plot_workspace()
        self.plot_page_host_layout.addWidget(self.plot_workspace)
        if self.float_plot_window is not None:
            self.float_plot_window.hide()
        self.plot_host = "page"
        self.plot_location_label.setText("Realtime Plot")
        self.float_plot_action.setChecked(False)

    def attach_plot_to_float(self):
        self.detach_plot_workspace()
        if self.float_plot_window is None:
            self.float_plot_window = QtWidgets.QDialog(self)
            self.float_plot_window.setWindowTitle("sf-Motion Realtime Plot")
            self.float_plot_window.resize(1200, 620)
            float_layout = QtWidgets.QVBoxLayout(self.float_plot_window)
            float_layout.setContentsMargins(8, 8, 8, 8)
            self.float_plot_window.finished.connect(self.on_float_plot_closed)
        self.float_plot_window.layout().addWidget(self.plot_workspace)
        self.float_plot_window.show()
        self.float_plot_window.raise_()
        self.plot_host = "float"
        self.plot_location_label.setText("Realtime Plot - Floating Window")
        self.float_plot_action.setChecked(True)

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

    def scroll_terminal_to_bottom(self):
        scrollbar = self.console_output.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())

    def append_terminal_output(self, text):
        self.console_output.appendPlainText(str(text))
        self.scroll_terminal_to_bottom()
        self.terminal_scroll_timer.start(0)

    def log(self, message):
        print(message)
        if hasattr(self, "console_output"):
            self.append_terminal_output(str(message))

    def echo_command(self, command):
        if hasattr(self, "console_output"):
            self.append_terminal_output(f">>> {command}")

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

        self.append_terminal_output(f">>> {cmd}")
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
                self.append_terminal_output(out.rstrip())
        except Exception:
            self.append_terminal_output(traceback.format_exc())
        self.console_input.clear()

    def show_connection_dialog(self):
        dialog = SerialPortDialog(self)
        if dialog.exec_() == QtWidgets.QDialog.Accepted:
            port = dialog.get_selected_port()
            baudrate = dialog.get_baudrate()
            if port:
                self.connect_serial(port, baudrate)

    def connect_serial(self, port, baudrate):
        self.parameters_loaded = False
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
            self.motor.reset_plotter_streams()
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

        except Exception as e:
            self.is_connected = False
            self.disconnect_serial()
            QtWidgets.QMessageBox.critical(
                self,
                "Connection Error",
                f"Failed to connect to {port}:\n{str(e)}",
            )
            self.status_label.setText(f"Connection failed: {str(e)}")
            self.log(f"Connection error: {e}")
            self.update_connection_ui()

    def disconnect_serial(self):
        self.parameters_loaded = False
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
        # A failed preliminary zero must never prevent the disable attempt.
        try:
            if self.current_motor_mode == 0:
                self.motor.set_foc_current_set_point(0.0)
            elif self.current_motor_mode == 1:
                self.motor.set_foc_speed_set_point(0.0)
        except Exception as exc:
            self.log(f"Zero setpoint failed; attempting disable anyway: {exc}")
        try:
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
            self.log("Motor disable was not confirmed")

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
        update_control_tooltips(self, mode)

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

            if mode == 2:
                current = motor.get_actual_angle()
                if not math.isfinite(current):
                    raise ValueError("Invalid position feedback; relative move cancelled")

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
                target = current + value
                self.echo_command(f"current = {current!r}  # fresh board feedback")
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
            check.setToolTip(tip("Channel checkbox"))
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
        for name in list(self.channels):
            if name not in active:
                channel = self.channels.pop(name)
                self.plot_widget.removeItem(channel["line"])
                self.release_color(channel["color"])
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
        failed_channels = []
        for name in names:
            try:
                self.echo_command(f"motor.plotter_add_line({name!r})")
                motor.plotter_add_line(name)
            except Exception:
                failed_channels.append(name)
                self.log(traceback.format_exc())
        self.sync_channel_checks()
        for name, button in self.preset_buttons.items():
            button.setChecked(name == preset_name)
        if failed_channels:
            self.log(f"Plot preset {preset_name} incomplete; failed channels: {', '.join(failed_channels)}")
        else:
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

    def on_data_received(self, address, value):
        if self.paused or not self.is_connected:
            return

        sample_name = self.motor.channel_name(address)
        if sample_name is None or sample_name not in self.motor.plotter_channels:
            return

        self.counter += 1
        self.time_buffer.append(self.counter)

        if sample_name not in self.channels:
            color = self.get_next_color()
            pen = pg.mkPen(color=color, width=1.5)
            line = self.plot_widget.plot([], [], pen=pen, name=sample_name)
            self.channels[sample_name] = {
                "line": line,
                "buffer": deque(maxlen=self.max_points),
                "time": deque(maxlen=self.max_points),
                "color": color,
            }

        # Stream frames contain one channel, so only advance that channel's history.
        channel = self.channels[sample_name]
        channel["buffer"].append(value)
        channel["time"].append(self.counter)

        self.status_label.setText(f"Received {sample_name}")
        self.sync_channel_checks()
        self.update_realtime_values()

    def update_plot(self):
        if self.paused or not self.is_connected or self.motor is None:
            return

        for name in self.motor.plotter_channels:
            if name not in self.channels:
                continue
            ch = self.channels[name]
            x = np.asarray(ch["time"])
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
            self.channels[name]["time"].clear()
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
                self.append_terminal_output(out)
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
        self.parameters_loaded = False
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

    def parameter_document(self, sections):
        values = {}
        for section in sections:
            for key in self.parameter_sections[section]:
                widget = self.parameter_widgets[key]
                values[key] = (
                    widget.isChecked() if isinstance(widget, QtWidgets.QCheckBox)
                    else self.parameter_value(key)
                )
        return {"version": 1, "parameters": values}

    def validate_parameter_document(self, document):
        if not isinstance(document, dict):
            raise ValueError("Expected a JSON object.")
        if type(document.get("version")) is not int or document["version"] != 1:
            raise ValueError("Unsupported parameter file version; expected version 1.")
        values = document.get("parameters")
        if not isinstance(values, dict) or not values:
            raise ValueError("The parameters mapping must contain at least one parameter.")
        for key, value in values.items():
            if key not in self.parameter_widgets:
                raise ValueError(f"Unknown parameter: {key}")
            widget = self.parameter_widgets[key]
            if isinstance(widget, QtWidgets.QCheckBox):
                if type(value) is not bool:
                    raise ValueError(f"{key} must be a boolean.")
            else:
                if isinstance(widget, QtWidgets.QSpinBox):
                    valid_type = type(value) is int
                else:
                    valid_type = type(value) in (int, float)
                if not valid_type:
                    raise ValueError(f"{key} has an invalid numeric type.")
                # Check bounds first, so arbitrarily large JSON integers never overflow
                # during conversion to a Qt or Python floating-point value.
                if not widget.minimum() <= value <= widget.maximum() or not math.isfinite(value):
                    raise ValueError(
                        f"{key} must be finite and between "
                        f"{widget.minimum()} and {widget.maximum()}."
                    )
        return values

    def load_parameter_document(self, document):
        values = self.validate_parameter_document(document)
        for key, value in values.items():
            self.set_parameter_value(key, value)
        if set(values) == set(self.parameter_widgets):
            self.parameters_loaded = True

    def report_parameter_file_error(self, action, error):
        message = f"Could not {action} parameters: {error}"
        self.log(message)
        QtWidgets.QMessageBox.warning(self, "Parameter File Error", message)

    def export_parameters(self):
        dialog = ParameterExportDialog(self.parameter_sections, self)
        if dialog.exec_() != QtWidgets.QDialog.Accepted:
            return
        sections = dialog.selected_sections()
        if not sections:
            return
        filename, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "Export Parameters", "parameters.json", "JSON files (*.json)"
        )
        if not filename:
            return
        try:
            document = self.parameter_document(sections)
            self.validate_parameter_document(document)
            Path(filename).write_text(
                json.dumps(document, indent=2, allow_nan=False) + "\n", encoding="utf-8"
            )
            self.log(f"Exported {len(document['parameters'])} parameters to {filename}")
        except (OSError, ValueError) as error:
            self.report_parameter_file_error("export", error)

    def import_parameters(self):
        filename, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "Import Parameters", "", "JSON files (*.json)"
        )
        if not filename:
            return
        try:
            document = json.loads(Path(filename).read_text(encoding="utf-8"))
            self.load_parameter_document(document)
            self.log(f"Imported parameters from {filename}. Review fields before applying.")
        except (OSError, ValueError) as error:
            self.report_parameter_file_error("import", error)

    def apply_all_parameters(self):
        motor = self.require_motor()
        if motor is None:
            return
        if not self.parameters_loaded:
            self.log(
                "Apply All Parameters blocked. Read all parameters or import a complete "
                "parameter file first."
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
            self.parameters_loaded = False
            self.log("Parameters may be partially applied; read all parameters before retrying")
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
        motor = self.require_motor()
        if motor is None:
            return
        try:
            home = motor.get_actual_angle()
            if not math.isfinite(home):
                raise ValueError("Invalid position feedback")
        except Exception as exc:
            self.log(f"Position demo cancelled: {exc}")
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
