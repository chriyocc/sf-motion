import os
from pathlib import Path
import queue
import struct
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from PyQt5 import QtCore, QtWidgets
from MotorProtocol import MotorProtocol
from plotter_custom import LivePlotter


class ProtocolRegressionTests(unittest.TestCase):
    def setUp(self):
        self.serial = Mock()
        self.responses = Mock()
        self.motor = MotorProtocol(self.serial, SimpleNamespace(response_queue=self.responses))
        self.protocol = self.motor.protocol

    def address(self, name):
        return next(a for a, r in self.protocol.registers.items() if r["name"] == name)

    def test_timeout_quarantines_late_reply_even_for_same_register(self):
        self.responses.get.side_effect = queue.Empty
        with self.assertRaises(TimeoutError):
            self.protocol.get_speed_kp()
        self.responses.get.side_effect = None
        self.responses.get.return_value = (self.address("speed_kp"), struct.pack("<f", 123))
        with self.assertRaisesRegex(RuntimeError, "reconnect"):
            self.protocol.get_speed_kp()
        self.assertEqual(self.serial.write.call_count, 1)
        self.assertEqual(self.responses.get.call_count, 1)

    def test_disable_is_sent_after_timeout_but_not_reported_confirmed(self):
        self.responses.get.side_effect = queue.Empty
        with self.assertRaises(TimeoutError):
            self.motor.set_foc_speed_set_point(0)
        with self.assertRaisesRegex(RuntimeError, "Disable sent but unconfirmed"):
            self.motor.set_foc_motor_mode(6)
        self.assertEqual(self.serial.write.call_count, 2)
        self.assertEqual(self.serial.write.call_args.args[0],
                         b"\xa5\xa5\x03\x02" + bytes([self.address("motor_mode"), 6]))

    def test_mismatched_reply_blocks_subsequent_writes(self):
        self.responses.get.return_value = (self.address("speed_ki"), struct.pack("<f", 1))
        with self.assertRaisesRegex(RuntimeError, "Unexpected response"):
            self.protocol.get_speed_kp()
        with self.assertRaisesRegex(RuntimeError, "reconnect"):
            self.protocol.set_speed_kp(1)
        self.assertEqual(self.serial.write.call_count, 1)

    def test_invalid_group_is_rejected_before_any_write(self):
        for invalid in (-2, float("nan"), float("inf")):
            with self.subTest(invalid=invalid), self.assertRaises(ValueError):
                self.motor.set_pid_speed(1, 2, invalid, 0)
        with self.assertRaises(ValueError):
            self.motor.set_pid_position(1, 2, 3, 4, 0, 0)
        with self.assertRaises(ValueError):
            self.motor.set_field_weakening(1, 2, 3)
        self.serial.write.assert_not_called()

    def test_mid_group_failure_reports_partial_update_and_stops(self):
        self.responses.get.side_effect = [(self.address("speed_kp"), b"\x00"), queue.Empty]
        with self.assertRaisesRegex(RuntimeError, "partially applied"):
            self.motor.set_pid_speed(1, 2, 3, 0)
        self.assertEqual(self.serial.write.call_count, 2)

    def test_device_rejection_is_not_success_or_transport_desync(self):
        self.responses.get.return_value = (self.address("speed_kp"), b"\xff")
        with self.assertRaisesRegex(RuntimeError, "Device returned error: -1"):
            self.protocol.set_speed_kp(1)
        self.responses.get.return_value = (self.address("speed_kp"), b"\x00")
        self.assertTrue(self.protocol.set_speed_kp(2))


class GuiControlRegressionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def setUp(self):
        with patch.object(QtCore.QTimer, "singleShot"):
            self.gui = LivePlotter(max_points=4)
        self.gui.timer.stop()
        self.gui.fps_timer.stop()
        self.gui.motor = Mock()
        self.gui.serial_conn = Mock(is_open=True)
        self.gui.is_connected = True
        self.gui.update_mode_ui = Mock()

    def tearDown(self):
        self.gui.is_connected = False
        self.gui.close()

    def test_disable_survives_zero_timeout_in_both_modes(self):
        for mode, setter in ((0, "set_foc_current_set_point"), (1, "set_foc_speed_set_point")):
            with self.subTest(mode=mode):
                self.gui.motor.reset_mock()
                self.gui.current_motor_mode = mode
                getattr(self.gui.motor, setter).side_effect = TimeoutError()
                self.assertTrue(self.gui.try_disable_motor())
                self.gui.motor.set_foc_motor_mode.assert_called_once_with(6)

    def test_disconnect_invalidates_apply_all(self):
        self.gui.parameters_loaded = True
        self.gui.disconnect_serial()
        self.assertFalse(self.gui.parameters_loaded)
        self.gui.is_connected = True
        self.gui.motor.reset_mock()
        self.gui.apply_all_parameters()
        self.assertEqual(self.gui.motor.mock_calls, [])

    def test_failed_refresh_invalidates_previous_read(self):
        self.gui.parameters_loaded = True
        self.gui.motor.get_pole_pairs.side_effect = TimeoutError()
        self.gui.read_all_parameters()
        self.assertFalse(self.gui.parameters_loaded)

    def test_failed_apply_requires_fresh_read(self):
        self.gui.parameters_loaded = True
        self.gui.motor.set_kv.side_effect = TimeoutError()
        self.gui.apply_all_parameters()
        self.assertFalse(self.gui.parameters_loaded)
        self.gui.motor.set_pid_id.assert_not_called()

    def test_reconnect_invalidates_previous_read(self):
        self.gui.is_connected = False
        self.gui.parameters_loaded = True
        self.gui.populate_plot_channels = Mock()
        self.gui.refresh_motor_mode = Mock()
        self.gui.apply_default_plot_for_mode = Mock()
        self.gui.update_connection_ui = Mock()
        self.gui.setup_console_completion = Mock()
        with patch("plotter_custom.serial.Serial", return_value=self.gui.serial_conn), \
             patch("plotter_custom.DataAcquisitionThread"), \
             patch("plotter_custom.MotorProtocol", return_value=self.gui.motor):
            self.gui.connect_serial("mock-port", 115200)
        self.assertTrue(self.gui.is_connected)
        self.assertFalse(self.gui.parameters_loaded)

    def test_relative_move_reads_board_even_when_plot_is_paused(self):
        self.gui.paused = True
        self.gui.current_motor_mode = 2
        self.gui.selected_control_mode = Mock(return_value=2)
        self.gui.latest_channel_value = Mock(return_value=100)
        self.gui.motor.get_actual_angle.return_value = 140
        self.gui.setpoint_spin.setRange(-360000, 360000)
        self.gui.setpoint_spin.setValue(10)
        self.gui.apply_setpoint()
        self.gui.motor.set_foc_position_set_point.assert_called_once_with(150)
        self.gui.latest_channel_value.assert_not_called()

    def test_failed_position_read_does_not_enable_or_move(self):
        self.gui.current_motor_mode = 6
        self.gui.selected_control_mode = Mock(return_value=2)
        self.gui.motor.get_actual_angle.side_effect = TimeoutError()
        self.gui.apply_setpoint()
        self.gui.motor.set_foc_motor_mode.assert_not_called()
        self.gui.motor.set_foc_position_set_point.assert_not_called()

    def test_pid_widgets_enforce_domain(self):
        for key in ("pid_speed.out_max", "pid_id.kp", "pid_position.deadband"):
            widget = self.gui.parameter_widgets[key]
            widget.setValue(-2)
            self.assertGreaterEqual(widget.value(), 0)
        self.assertGreater(self.gui.parameter_widgets["pid_position.d_fc"].minimum(), 0)
        self.assertEqual(self.gui.parameter_widgets["field_weakening.out_min"].maximum(), 0)


if __name__ == "__main__":
    unittest.main()
