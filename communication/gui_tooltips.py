"""User-facing help text for the custom GUI; no motor behavior lives here."""

from html import escape

from PyQt5 import QtWidgets


TIPS = {
    'Serial port selector': 'Select the COM port connected to your SF-Motion board.',
    'Refresh Ports': 'Scan again for connected serial devices.',
    'Baudrate': 'Serial communication speed. Use 115200 unless your device requires another setting.',
    'Connect': 'Open communication with the board. Connection alone does not confirm motor power or calibration.',
    'Disconnect': 'Close communication with the board. Disable the motor first.',
    'Mode status': 'Last motor mode known to the app. Selecting a control option does not activate it until you apply a setpoint.',
    'Current mode': 'Regulate motor current in amperes. Current controls torque; it does not limit speed.',
    'Speed mode': 'Regulate motor speed in RPM.',
    'Position mode': 'Move by an offset in degrees from the current position, then actively hold the target.',
    'Disabled mode selector': 'Select Disabled, then click Apply Setpoint to disable the drive.',
    'Setpoint — Current': 'Requested current in amperes. The sign selects torque direction.',
    'Setpoint — Speed': 'Requested speed in RPM. The sign selects rotation direction.',
    'Setpoint — Position': 'Relative movement in degrees. Each Apply reads the current position and adds this offset.',
    'Apply Setpoint': 'Activate the selected mode and send the entered target. The motor may move immediately.',
    'Zero Setpoint — Current': 'Send zero current. The drive remains enabled.',
    'Zero Setpoint — Speed': 'Request zero speed. The drive remains enabled.',
    'Zero Setpoint — Position': 'Hold the freshly read current position. This does not reset the encoder or establish home.',
    'Relative Position field': 'Enter the movement offset in degrees. Positive and negative values request opposite directions.',
    'Move Relative': 'Read the current position, add this offset, and command a position move.',
    'Disable Motor — everywhere': 'Disable motor drive and release active holding torque. A moving shaft may continue to coast.',
    'Pole pairs': 'Number of magnetic pole pairs, not total poles. For example, 14 poles means 7 pole pairs.',
    'Set Pole Pairs': 'Write the entered pole-pair count to the board. Save Config makes it persistent.',
    'Set Sensored FOC': 'Use encoder feedback for field-oriented motor control.',
    'Run Self Commissioning': 'Measure motor resistance and inductance, calculate current PI gains, and start encoder calibration. The shaft will move.',
    'Read Motor Params': 'Read and display the board’s motor parameters for review.',
    'Save Config — everywhere': 'Store the current board configuration for future power cycles. Apply edited fields first and disable the motor before saving.',
    'Read All Parameters': 'Load current board settings into the form, replacing the displayed values.',
    'Apply All Parameters': 'Write all displayed settings to the board. Read all settings or import a complete file first. Save Config makes changes persistent.',
    'Export Parameters': 'Export the selected sections’ displayed values to a JSON file, including unapplied edits.',
    'Import Parameters': 'Load settings from a JSON file into the form. Review them, then Apply All Parameters to update the board.',
    'Export section checkboxes': 'Choose which parameter sections to include in the exported file.',
    'Kv': 'Motor speed constant in RPM per volt. Use the value specified for your motor.',
    'Rs': 'Motor resistance parameter in ohms. Use validated commissioning results.',
    'Ld': 'd-axis inductance in henries. Convert millihenries to henries before entering.',
    'Lq': 'q-axis inductance in henries. Convert millihenries to henries before entering.',
    'Flux linkage': 'Motor magnetic flux-linkage parameter. Use a validated value for this motor.',
    'Current PI group': 'Id regulates d-axis current; Iq regulates torque-producing q-axis current.',
    'Kp — all controllers': 'Proportional gain: response to the present error. Excessive gain can cause oscillation.',
    'Ki — all controllers': 'Integral gain: corrects persistent error over time. Excessive gain can cause overshoot.',
    'Position Kd': 'Derivative gain: adds damping based on how quickly position error changes.',
    'Id / Iq deadband': 'Current error band in amperes used by the controller’s deadband logic.',
    'Speed deadband': 'Speed error band in RPM used by the controller’s deadband logic.',
    'Position deadband': 'Position error band in degrees used by the controller’s deadband logic.',
    'Speed out max': 'Maximum magnitude of the speed controller’s current command, in amperes.',
    'Position out max': 'Maximum magnitude of the position controller’s speed command, in RPM.',
    'Position D fc': 'Derivative filter cutoff in Hz. Lower values smooth more noise but add delay.',
    'Field weakening enabled': 'Enable advanced field weakening for higher-speed operation. Requires validated motor parameters and limits.',
    'FW Kp / Ki': 'Gains for the field-weakening controller. Advanced tuning settings.',
    'FW out min': 'Lower output bound for the field-weakening controller. Must be zero or negative.',
    'MTPA enabled': 'Enable maximum torque per ampere control. Requires suitable motor parameters.',
    'Plot presets': 'Replace the current channel selection with signals for this task.',
    'Channel checkbox': 'Subscribe to or remove this live signal. The board supports up to 10 simultaneous channels.',
    'Remove All Channels': 'Stop all currently selected plot subscriptions. This does not stop the motor.',
    'Clear — plot': 'Clear displayed sample history while keeping channel subscriptions active.',
    'Pause / Resume': 'Pause or resume plot updates. This does not stop the motor.',
    'Auto Range': 'Automatically scale the plot to fit signal values. Channels with different units share the same axis.',
    'Sample axis': 'Horizontal position represents sample count, not elapsed time.',
    'Samples / FPS': 'Sample and plot refresh statistics. FPS is the display refresh rate, not the motor control frequency.',
    'Terminal input': 'Enter a Python command and press Enter. Commands can change live motor settings.',
    'Clear — terminal': 'Clear terminal output. This does not reset the board or its settings.',
    'Terminal resize handle': 'Drag up or down to resize the terminal.',
}


def tip(key):
    # Rich text lets Qt wrap long descriptions instead of making a wide single line.
    return "<qt>" + escape(TIPS[key]) + "</qt>"


def apply_action_tooltips(parent):
    aliases = {"Disable Motor": "Disable Motor — everywhere", "Save Config": "Save Config — everywhere",
               "Clear Console": "Clear — terminal"}
    for control in parent.findChildren(QtWidgets.QPushButton) + parent.findChildren(QtWidgets.QAction):
        key = aliases.get(control.text(), control.text())
        if key in TIPS:
            control.setToolTip(tip(key))


def parameter_tip(key):
    direct = {
        "pole_pairs": "Pole pairs", "kv": "Kv", "rs": "Rs", "ld": "Ld", "lq": "Lq",
        "flux_linkage": "Flux linkage", "pid_id.deadband": "Id / Iq deadband",
        "pid_iq.deadband": "Id / Iq deadband", "pid_speed.deadband": "Speed deadband",
        "pid_position.deadband": "Position deadband", "pid_speed.out_max": "Speed out max",
        "pid_position.out_max": "Position out max", "pid_position.kd": "Position Kd",
        "pid_position.d_fc": "Position D fc", "field_weakening.out_min": "FW out min",
        "field_weakening_enable": "Field weakening enabled", "mtpa_enable": "MTPA enabled",
    }
    if key in direct:
        return tip(direct[key])
    if key.startswith("field_weakening."):
        return tip("FW Kp / Ki")
    return tip("Kp — all controllers" if key.endswith(".kp") else "Ki — all controllers")


def update_control_tooltips(gui, mode):
    names = {0: "Current", 1: "Speed", 2: "Position"}
    name = names.get(mode)
    gui.setpoint_spin.setToolTip(tip("Setpoint — " + name) if name else tip("Disabled mode selector"))
    gui.zero_setpoint_button.setToolTip(
        tip("Zero Setpoint — " + name) if name else tip("Disabled mode selector")
    )


def apply_window_tooltips(gui):
    apply_action_tooltips(gui)
    for mode, button in gui.mode_buttons.items():
        key = {0: "Current mode", 1: "Speed mode", 2: "Position mode", 6: "Disabled mode selector"}[mode]
        button.setToolTip(tip(key))
    gui.mode_status_label.setToolTip(tip("Mode status"))
    gui.mode_metric["box"].setToolTip(tip("Mode status"))
    gui.mode_metric["value"].setToolTip(tip("Mode status"))
    gui.relative_position_spin.setToolTip(tip("Relative Position field"))
    gui.pole_pairs_spin.setToolTip(tip("Pole pairs"))
    for button in gui.preset_buttons.values():
        button.setToolTip(tip("Plot presets"))
    gui.clear_button.setToolTip(tip("Clear — plot"))
    gui.pause_button.setToolTip(tip("Pause / Resume"))
    gui.auto_range_check.setToolTip(tip("Auto Range"))
    gui.plot_widget.getAxis("bottom").setToolTip(tip("Sample axis"))
    gui.info_label.setToolTip(tip("Samples / FPS"))
    gui.sample_status_label.setToolTip(tip("Samples / FPS"))
    gui.plot_metric["box"].setToolTip(tip("Channel checkbox"))
    gui.console_input.setToolTip(tip("Terminal input"))
    gui.terminal_splitter.handle(1).setToolTip(tip("Terminal resize handle"))
    for button in gui.terminal_panel.findChildren(QtWidgets.QPushButton):
        if button.text() == "Clear":
            button.setToolTip(tip("Clear — terminal"))
    update_control_tooltips(gui, gui.active_control_mode)
