import os
from pathlib import Path
import struct
import sys
import unittest
from unittest.mock import patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np
from PyQt5 import QtCore, QtWidgets

from MotorProtocol import MotorProtocol
from plotter_custom import DataAcquisitionThread, LivePlotter


class CustomPlotStreamingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def setUp(self):
        with patch.object(QtCore.QTimer, "singleShot"):
            self.plotter = LivePlotter(max_points=4)
        self.plotter.timer.stop()
        self.plotter.fps_timer.stop()
        self.plotter.motor = MotorProtocol()
        self.plotter.motor.plotter_channels = ["rpm_ref", "actual_rpm"]
        self.plotter.is_connected = True
        self.reader = DataAcquisitionThread()
        self.reader.data_received.connect(self.plotter.on_data_received)

    def tearDown(self):
        self.plotter.is_connected = False
        self.plotter.close()

    def feed_sample(self, name, value):
        address = self.plotter.motor.plotter_dict[name]
        frame = b"\xA5\xA5\x06\x05" + bytes([address]) + struct.pack("<f", value)
        for fragment in (frame[:4], frame[4:]):
            self.reader.raw_buffer.extend(fragment)
            self.reader.parse_buffer()

    def test_interleaved_frames_produce_continuous_curves(self):
        for index in range(6):
            self.feed_sample("rpm_ref", 100.0)
            self.feed_sample("actual_rpm", float(index))
        self.plotter.update_plot()

        for name, expected_x, expected_y in (
            ("rpm_ref", [5, 7, 9, 11], [100] * 4),
            ("actual_rpm", [6, 8, 10, 12], [2, 3, 4, 5]),
        ):
            x, y = self.plotter.channels[name]["line"].getData()
            np.testing.assert_array_equal(x, expected_x)
            np.testing.assert_array_equal(y, expected_y)
            self.assertTrue(np.isfinite(y).all())
        self.assertEqual(self.plotter.latest_channel_value("actual_rpm"), 5.0)

    def test_clear_pause_and_channel_removal(self):
        self.feed_sample("actual_rpm", 10.0)
        self.plotter.paused = True
        self.feed_sample("actual_rpm", 20.0)
        self.assertEqual(self.plotter.latest_channel_value("actual_rpm"), 10.0)
        self.plotter.paused = False
        self.plotter.clear_data()
        self.assertFalse(self.plotter.channels["actual_rpm"]["time"])
        self.feed_sample("actual_rpm", 30.0)
        self.plotter.update_plot()
        x, y = self.plotter.channels["actual_rpm"]["line"].getData()
        np.testing.assert_array_equal(x, [1])
        np.testing.assert_array_equal(y, [30])

        self.plotter.motor.plotter_channels.clear()
        self.plotter.sync_channel_checks()
        self.assertFalse(self.plotter.channels)
        self.assertFalse(self.plotter.used_colors)
        self.feed_sample("actual_rpm", 40.0)
        self.assertFalse(self.plotter.channels)


if __name__ == "__main__":
    unittest.main()
