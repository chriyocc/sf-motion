import json
import os
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import Mock, patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from PyQt5 import QtCore, QtGui, QtWidgets
from MotorProtocol import MotorProtocol
from plotter_custom import LivePlotter, ParameterExportDialog


class CustomGuiFeatureTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        # Windows' offscreen Qt platform needs its font registered explicitly.
        font_path = Path("C:/Windows/Fonts/segoeui.ttf")
        if font_path.exists():
            QtGui.QFontDatabase.addApplicationFont(str(font_path))

    def setUp(self):
        with patch.object(QtCore.QTimer, "singleShot"):
            self.gui = LivePlotter(max_points=4)
        self.gui.timer.stop()
        self.gui.fps_timer.stop()

    def tearDown(self):
        self.gui.is_connected = False
        self.gui.close()

    def document(self):
        return self.gui.parameter_document(self.gui.parameter_sections)

    def test_complete_round_trip_and_apply_requires_explicit_action(self):
        self.gui.set_parameter_value("pid_position.kp", 2.5)
        self.gui.set_parameter_value("field_weakening_enable", True)
        document = json.loads(json.dumps(self.document()))
        self.gui.set_parameter_value("pid_position.kp", 0)
        self.gui.is_connected = True
        self.gui.motor = Mock()
        self.gui.load_parameter_document(document)
        self.assertEqual(self.gui.parameter_value("pid_position.kp"), 2.5)
        self.assertEqual(self.gui.parameter_value("field_weakening_enable"), 1)
        self.assertTrue(self.gui.parameters_loaded)
        self.gui.motor.assert_not_called()
        self.assertEqual(self.gui.motor.mock_calls, [])
        self.gui.apply_all_parameters()
        self.gui.motor.set_pid_position.assert_called_once()
        self.gui.motor.save_config.assert_not_called()

    def test_partial_import_preserves_other_fields_and_readiness(self):
        document = {"version": 1, "parameters": {"pid_speed.kp": 1.25}}
        self.gui.set_parameter_value("kv", 300)
        for ready in (False, True):
            self.gui.parameters_loaded = ready
            self.gui.load_parameter_document(document)
            self.assertEqual(self.gui.parameter_value("pid_speed.kp"), 1.25)
            self.assertEqual(self.gui.parameter_value("kv"), 300)
            self.assertEqual(self.gui.parameters_loaded, ready)
        self.gui.parameters_loaded = False
        self.gui.motor = Mock()
        self.gui.is_connected = True
        self.gui.apply_all_parameters()
        self.assertEqual(self.gui.motor.mock_calls, [])

    def test_invalid_import_is_atomic(self):
        before = self.document()
        invalid_entries = [
            ("unknown", 1), ("pole_pairs", 1.5), ("pole_pairs", True),
            ("kv", "100"), ("kv", float("nan")), ("kv", float("inf")),
            ("kv", 10**400), ("pid_id.kp", -1), ("pid_position.d_fc", 0),
            ("field_weakening.out_min", 1), ("mtpa_enable", 1),
        ]
        for key, value in invalid_entries:
            with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                self.gui.load_parameter_document({
                    "version": 1, "parameters": {"rs": 2, key: value}
                })
            self.assertEqual(self.document(), before)
            self.assertFalse(self.gui.parameters_loaded)
        for document in ([], {}, {"version": True, "parameters": {"kv": 1}},
                         {"version": 2, "parameters": {"kv": 1}},
                         {"version": 1, "parameters": {}}):
            with self.subTest(document=document), self.assertRaises(ValueError):
                self.gui.load_parameter_document(document)

    def test_export_section_selection_defaults_and_empty_selection(self):
        dialog = ParameterExportDialog(self.gui.parameter_sections, self.gui)
        self.assertEqual(dialog.selected_sections(), list(self.gui.parameter_sections))
        for checkbox in dialog.section_checks.values():
            checkbox.setChecked(False)
        self.assertFalse(dialog.buttons.button(QtWidgets.QDialogButtonBox.Save).isEnabled())
        dialog.section_checks["Advanced"].setChecked(True)
        self.assertTrue(dialog.buttons.button(QtWidgets.QDialogButtonBox.Save).isEnabled())
        self.assertEqual(set(self.gui.parameter_document(dialog.selected_sections())["parameters"]),
                         set(self.gui.parameter_sections["Advanced"]))

    def test_file_export_import_and_cancel(self):
        with tempfile.TemporaryDirectory() as directory:
            path = str(Path(directory) / "parameters.json")
            with patch("plotter_custom.ParameterExportDialog") as dialog, \
                 patch.object(QtWidgets.QFileDialog, "getSaveFileName", return_value=(path, "")):
                dialog.return_value.exec_.return_value = QtWidgets.QDialog.Accepted
                dialog.return_value.selected_sections.return_value = ["Motor Identity"]
                self.gui.set_parameter_value("kv", 250)
                self.gui.export_parameters()
            self.assertEqual(set(json.loads(Path(path).read_text())["parameters"]),
                             set(self.gui.parameter_sections["Motor Identity"]))
            self.gui.set_parameter_value("kv", 0)
            with patch.object(QtWidgets.QFileDialog, "getOpenFileName", return_value=(path, "")):
                self.gui.import_parameters()
            self.assertEqual(self.gui.parameter_value("kv"), 250)
            before = self.document()
            with patch.object(QtWidgets.QFileDialog, "getOpenFileName", return_value=("", "")):
                self.gui.import_parameters()
            self.assertEqual(self.document(), before)
            with patch("plotter_custom.ParameterExportDialog") as dialog, \
                 patch.object(QtWidgets.QFileDialog, "getSaveFileName") as save:
                dialog.return_value.exec_.return_value = QtWidgets.QDialog.Rejected
                self.gui.export_parameters()
                save.assert_not_called()
            with patch("plotter_custom.ParameterExportDialog") as dialog, \
                 patch.object(QtWidgets.QFileDialog, "getSaveFileName", return_value=("", "")):
                dialog.return_value.exec_.return_value = QtWidgets.QDialog.Accepted
                self.gui.export_parameters()
            self.assertEqual(self.document(), before)

    def test_file_errors_are_reported_without_modifying_fields(self):
        before = self.document()
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "bad.json"
            for content in ("not JSON", '{"version":1,"parameters":{"kv":-1e99}}'):
                path.write_text(content)
                with patch.object(QtWidgets.QFileDialog, "getOpenFileName", return_value=(str(path), "")), \
                     patch.object(QtWidgets.QMessageBox, "warning") as warning:
                    self.gui.import_parameters()
                    warning.assert_called_once()
                self.assertEqual(self.document(), before)
            with patch.object(QtWidgets.QFileDialog, "getOpenFileName", return_value=(str(path / "missing"), "")), \
                 patch.object(QtWidgets.QMessageBox, "warning") as warning:
                self.gui.import_parameters()
                warning.assert_called_once()

    def test_float_moves_entire_workspace_and_preserves_stream(self):
        self.gui.show()
        self.gui.show_page(2)
        self.gui.motor = MotorProtocol()
        self.gui.motor.plotter_channels = ["actual_rpm"]
        self.gui.is_connected = True
        self.gui.on_data_received(self.gui.motor.plotter_dict["actual_rpm"], 42)
        channel = self.gui.channels["actual_rpm"]
        original_buttons = self.gui.preset_buttons.copy()
        for _ in range(3):
            self.gui.attach_plot_to_float()
            self.app.processEvents()
            self.assertTrue(self.gui.float_plot_window.isAncestorOf(self.gui.channel_scroll))
            self.assertTrue(self.gui.channel_scroll.isVisible())
            self.assertTrue(self.gui.float_plot_window.isAncestorOf(self.gui.preset_buttons["Speed"]))
            self.gui.float_plot_window.close()
            self.app.processEvents()
            self.assertEqual(self.gui.plot_host, "page")
            self.assertTrue(self.gui.plot_page_host.isAncestorOf(self.gui.channel_scroll))
            self.assertTrue(self.gui.channel_scroll.isVisible())
            self.assertFalse(self.gui.float_plot_action.isChecked())
        self.gui.toggle_float_plot(True)
        self.gui.toggle_float_plot(False)
        self.assertEqual(self.gui.plot_host, "page")
        self.assertIs(self.gui.channels["actual_rpm"], channel)
        self.assertEqual(list(channel["buffer"]), [42])
        self.assertEqual(self.gui.preset_buttons, original_buttons)
        action_texts = [action.text() for action in self.gui.findChildren(QtWidgets.QAction)]
        self.assertNotIn("Dock Plot Side", action_texts)

    def test_terminal_follows_new_output_without_stealing_focus(self):
        self.gui.show()
        self.gui.console_input.setFocus()
        self.gui.log("\n".join(str(i) for i in range(100)))
        self.app.processEvents()
        scrollbar = self.gui.console_output.verticalScrollBar()
        self.assertGreater(scrollbar.maximum(), 0)
        for append in (lambda: self.gui.log("new log\nsecond line"),
                       lambda: self.gui.echo_command("motor.get_kv()"),
                       lambda: self.gui.append_terminal_output("Traceback\nerror")):
            scrollbar.setValue(0)
            append()
            self.app.processEvents()
            self.assertEqual(scrollbar.value(), scrollbar.maximum())
            self.assertTrue(self.gui.console_input.hasFocus())
        self.gui.console_input.setText("1 / 0")
        self.gui.execute_command()
        self.app.processEvents()
        self.assertIn("ZeroDivisionError", self.gui.console_output.toPlainText())
        self.assertEqual(scrollbar.value(), scrollbar.maximum())

    def test_parameter_actions_remain_visible_outside_scrolling_form(self):
        self.gui.resize(980, 620)
        self.gui.show()
        self.gui.nav_list.setCurrentRow(4)
        self.app.processEvents()
        scroll = self.gui.parameters_page.findChild(QtWidgets.QScrollArea, "ParameterScroll")
        buttons = self.gui.parameter_actions.findChildren(QtWidgets.QPushButton)
        self.assertEqual(len(buttons), 5)
        self.assertFalse(scroll.isAncestorOf(self.gui.parameter_actions))
        self.assertEqual(self.gui.size(), QtCore.QSize(980, 620))
        for position in (0, scroll.verticalScrollBar().maximum()):
            scroll.verticalScrollBar().setValue(position)
            self.app.processEvents()
            for button in buttons:
                self.assertTrue(button.isVisible())
                self.assertGreaterEqual(button.width(), button.sizeHint().width())
                rectangle = QtCore.QRect(button.mapTo(self.gui, QtCore.QPoint()), button.size())
                self.assertTrue(self.gui.rect().contains(rectangle))
        for widget in self.gui.parameter_widgets.values():
            self.assertTrue(widget.accessibleName())

    def test_all_pages_fit_minimum_width_and_scrolled_controls_are_reachable(self):
        self.gui.show()
        self.gui.resize(980, 620)
        for index in range(6):
            self.gui.nav_list.setCurrentRow(index)
            self.app.processEvents()
            page = self.gui.pages.currentWidget()
            for scroll in page.findChildren(QtWidgets.QScrollArea):
                self.assertEqual(scroll.horizontalScrollBar().maximum(), 0, f"page {index}")
                for button in scroll.widget().findChildren(QtWidgets.QPushButton):
                    scroll.ensureWidgetVisible(button)
                    self.app.processEvents()
                    center = button.mapTo(scroll.viewport(), button.rect().center())
                    self.assertTrue(scroll.viewport().rect().contains(center), button.text())

    def test_terminal_height_changes_without_losing_output_or_input(self):
        self.gui.resize(1100, 900)
        self.gui.show()
        self.gui.log("\n".join(f"line {i}" for i in range(80)))
        self.gui.console_input.setText("motor.get_kv()")
        self.app.processEvents()
        before = self.gui.console_output.height()
        text = self.gui.console_output.toPlainText()
        splitter = self.gui.terminal_splitter
        splitter.moveSplitter(splitter.sizes()[0] - 180, 1)
        self.app.processEvents()
        self.assertGreater(self.gui.console_output.height(), before + 100)
        self.assertEqual(self.gui.console_output.toPlainText(), text)
        self.assertEqual(self.gui.console_input.text(), "motor.get_kv()")
        self.gui.nav_list.setCurrentRow(4)
        self.app.processEvents()
        self.assertGreater(self.gui.console_output.height(), before + 100)
        splitter.moveSplitter(splitter.sizes()[0] + 180, 1)
        self.app.processEvents()
        self.assertLess(self.gui.console_output.height(), before + 30)
        self.assertTrue(self.gui.console_input.isVisible())
        self.assertFalse(splitter.childrenCollapsible())


if __name__ == "__main__":
    unittest.main()
