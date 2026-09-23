import os
from pathlib import Path
import sys
import unittest
from unittest.mock import Mock, patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from PyQt5 import QtCore, QtWidgets
import serial
from plotter import LivePlotter


class BasicConsoleTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def setUp(self):
        with patch.object(QtCore.QTimer, "singleShot"):
            self.gui = LivePlotter(max_points=4)
        self.gui.timer.stop()
        self.gui.fps_timer.stop()

    def tearDown(self):
        self.gui.close()

    def command(self, command):
        self.gui.console_input.setText(command)
        self.gui.execute_command()
        return self.gui.console_output.toPlainText()

    def connect(self):
        port = Mock(is_open=True)
        reader = Mock()

        def response(**kwargs):
            frame = port.write.call_args.args[0]
            return frame[4], b"\x06" if frame[3] == 1 else b"\x00"

        reader.response_queue.get.side_effect = response
        with patch("plotter.serial.Serial", return_value=port), \
             patch("plotter.DataAcquisitionThread", return_value=reader):
            self.gui.connect_serial("mock-port", 115200)
        return port, reader

    def test_disconnected_command_explains_how_to_connect(self):
        output = self.command("motor.set_foc_motor_mode(6)")
        self.assertIn("Click Connect", output)
        self.assertNotIn("Traceback", output)

    def test_legacy_command_sends_disable_and_reconnect_restores_console(self):
        for _ in range(2):
            port, reader = self.connect()
            self.assertTrue(self.gui.is_connected)
            self.assertIs(self.gui.console_namespace["motor"], self.gui.motor)
            output = self.command("motor.set_foc_motor_mode(6)")
            self.assertNotIn("Traceback", output)
            self.assertEqual(port.write.call_args.args[0], b"\xa5\xa5\x03\x02\x08\x06")
            self.gui.disconnect_serial()
            reader.stop.assert_called_once()
            self.assertIsNone(self.gui.motor)
            self.assertIsNone(self.gui.serial_conn)
            self.assertIn("Click Connect", self.command("motor.set_foc_motor_mode(6)"))

    def test_failed_serial_open_is_reported_in_console(self):
        with patch("plotter.serial.Serial", side_effect=serial.SerialException("Port busy")), \
             patch.object(QtWidgets.QMessageBox, "critical"):
            self.gui.connect_serial("mock-port", 115200)
        self.assertFalse(self.gui.is_connected)
        self.assertIn("Port busy", self.gui.console_output.toPlainText())

    def test_protocol_initialization_failure_cleans_up_connection(self):
        port = Mock(is_open=True)
        reader = Mock()
        with patch("plotter.serial.Serial", return_value=port), \
             patch("plotter.DataAcquisitionThread", return_value=reader), \
             patch("plotter.SFMotion", side_effect=ValueError("Invalid registers")), \
             patch.object(QtWidgets.QMessageBox, "critical"):
            self.gui.connect_serial("mock-port", 115200)
        reader.stop.assert_called_once()
        port.close.assert_called_once()
        self.assertFalse(self.gui.is_connected)
        self.assertIsNone(self.gui.motor)
        self.assertIn("Invalid registers", self.gui.console_output.toPlainText())


if __name__ == "__main__":
    unittest.main()
