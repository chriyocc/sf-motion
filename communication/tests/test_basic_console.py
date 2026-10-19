import os
from pathlib import Path
import sys
import struct
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
            if frame[3] == 1:
                register = self.gui.motor.registers[frame[5]]
                value = 6 if register["name"] == "motor_mode" else frame[5]
                return frame[4], frame[5], struct.pack(register["format"], value)
            return frame[4], frame[5], b"\x00"

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
            self.assertEqual(port.write.call_args.args[0], b"\xa5\xa5\x04\x02\x00\x0e\x06")
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
             patch("plotter.MotorProtocol", side_effect=ValueError("Invalid registers")), \
             patch.object(QtWidgets.QMessageBox, "critical"):
            self.gui.connect_serial("mock-port", 115200)
        reader.stop.assert_called_once()
        port.close.assert_called_once()
        self.assertFalse(self.gui.is_connected)
        self.assertIsNone(self.gui.motor)
        self.assertIn("Invalid registers", self.gui.console_output.toPlainText())

    def test_multiline_pid_reads_display_both_results(self):
        port, _ = self.connect()
        port.write.reset_mock()
        output = self.command("motor.get_pid_id()\nmotor.get_pid_iq()")
        self.assertNotIn("Traceback", output)
        self.assertIn("{'kp': 21.0, 'ki': 22.0, 'deadband': 23.0}", output)
        self.assertIn("{'kp': 24.0, 'ki': 25.0, 'deadband': 26.0}", output)
        self.assertEqual(port.write.call_count, 6)

    def test_save_config_and_raw_api_are_both_available(self):
        port, _ = self.connect()
        output = self.command("motor.save_config()")
        self.assertNotIn("Traceback", output)
        self.assertEqual(port.write.call_args.args[0], b"\xa5\xa5\x03\x02\x00\x08")
        output = self.command("motor.get_id_ki()")
        self.assertNotIn("Traceback", output)
        self.assertIn("22.0", output)
        completions = self.gui.build_completion()
        for name in ("get_pid_id", "get_pid_iq", "save_config", "get_id_ki"):
            self.assertIn(f"motor.{name}", completions)

    def test_multiline_assignment_and_expressions_preserve_namespace(self):
        output = self.command("value = 3\nvalue + 1\nvalue + 2")
        self.assertNotIn("Traceback", output)
        self.assertIn("4\n5", output)
        self.assertEqual(self.gui.console_namespace["value"], 3)


if __name__ == "__main__":
    unittest.main()
