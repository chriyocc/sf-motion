# SF-Motion Motor and Plotter User Guide

Checked against this workspace's Python API and firmware on 2026-09-20.
This guide assumes the firmware flashed on your board matches this project.
Examples use your 7-pole-pair motor and sensored FOC. Current and speed examples
are small starting commands, not universal safe limits for every motor or load.

## Contents

1. [Everyday quick start](#1-everyday-quick-start)
2. [Launch and connect](#2-launch-and-connect)
3. [First-time commissioning](#3-first-time-commissioning)
4. [Modes and units](#4-modes-and-units)
5. [Current control](#5-current-control)
6. [Speed control](#6-speed-control)
7. [Position control](#7-position-control)
8. [Encoder readings](#8-encoder-readings)
9. [Plot controls](#9-plot-controls)
10. [PID settings](#10-pid-settings)
11. [Save, reopen, and reset](#11-save-reopen-and-reset)
12. [Troubleshooting](#12-troubleshooting)
13. [Complete command reference](#13-complete-command-reference)
14. [HSI clock notes](#14-hsi-clock-notes)
15. [Implementation references](#15-implementation-references)

## 1. Everyday quick start

Use this after successful commissioning and saving the configuration.
Enter Python commands in the app's bottom console, one line at a time.
Do not type the `>>>` prompt. Use normal underscores, without backslashes.

Connect the app, then start from disabled sensored operation:

```python
motor.set_foc_motor_mode(6)
motor.set_foc_mode(0)
motor.get_pole_pairs()
```

Choose ONE control mode:

| Task | Select mode first | Then send target |
| --- | --- | --- |
| Current/torque | `motor.set_foc_motor_mode(0)` | `motor.set_foc_current_set_point(0.05)` |
| Speed | `motor.set_foc_motor_mode(1)` | `motor.set_foc_speed_set_point(50)` |
| Position | `motor.set_foc_motor_mode(2)` | Follow the relative-angle example in section 7 |
| Disable drive | `motor.set_foc_motor_mode(6)` | No target needed |

The firmware ignores a current, speed, or position setpoint when its matching
motor mode is not active. It can still reply with `{'err_code': 0}`.

Before disconnecting or closing the app:

```python
motor.set_foc_motor_mode(6)
```

Disable removes commanded drive; it is not a controlled braking command or a
guarantee that a moving shaft stops instantly. The current app does not send
disable when you close it or disconnect USB. Keep the DC supply cutoff accessible.

## 2. Launch and connect

Run this in PowerShell from the project folder if `.venv` already exists:

```powershell
.\.venv\Scripts\python.exe communication\motor.py
```

For a fresh Python environment, run these once:

```powershell
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install numpy pyserial PyQt5 pyqtgraph keyboard
.\.venv\Scripts\python.exe communication\motor.py
```

1. Connect USB-C with a data-capable cable.
2. Click **Connect**, select the board's COM port, and use the default 115200 baud.
3. Check communication in the app console:

```python
motor.get_foc_motor_mode()
motor.get_foc_mode()
motor.get_pole_pairs()
```

USB communication alone does not establish that the motor power stage is powered.
For motor movement and commissioning, connect the proper DC bus supply and all
three motor phases. Use the voltage appropriate to your board and motor, with a
conservative supply current limit. Secure the motor and leave its shaft clear.
Disconnect power before changing phase wiring.

Response meanings:

| Response | Meaning |
| --- | --- |
| `{'err_code': 0}` | Handler reported success; not proof of motion, valid calibration, or correct mode |
| `{'err_code': 255}` | Firmware's signed `-1` error decoded as unsigned by Python |
| `{'value': 2.5}` | A measured value or setting; units depend on the command |
| `{'mode': 2}` | A mode value; use the correct mode table below |
| Timeout/traceback | Communication did not complete; do not assume the command had no effect |

## 3. First-time commissioning

Do this for a new motor/encoder setup, changed phase wiring or magnet alignment,
lost calibration, or a previous attempt made with disconnected phases.
You do not need to repeat it just because you reopen the app.

### Prepare

Connect the phases, mount the encoder magnet correctly, power the DC bus, and
allow the motor to rotate freely during encoder calibration.

```python
motor.set_foc_motor_mode(6)
motor.set_foc_mode(0)
motor.set_pole_pairs(7)
motor.get_pole_pairs()
```

Seven pole pairs means 14 magnetic poles. Do not use 7 for a different motor
unless that is its actual pole-pair count.

### Run

```python
motor.self_commissioning()
```

This Python helper starts resistance, d-axis inductance, and q-axis inductance
measurements with one-second delays, prints their values, calculates current PI
gains using a 500 Hz bandwidth, and starts encoder calibration.

**The helper returns after starting encoder calibration. It does not wait for
encoder calibration to finish and does not save the configuration.** Allow the
complete calibration movement to finish before changing modes or saving. There
is no exposed calibration-complete query in the current command API. A stopped
shaft alone is not proof of successful calibration if it stalled or never moved.

The project README requires CCW rotation during encoder calibration. Confirm the
viewing convention and encoder direction for your assembly. If phase order must
change, switch off power before swapping two phases, then recommission.

### Inspect, then save

After calibration has completed:

```python
motor.get_motor_param()
motor.get_pid_id()
motor.get_pid_iq()
motor.set_foc_motor_mode(6)
motor.save_config()
```

The earlier readings around `Rs = 594` and `Rs = 1945` obtained with disconnected
motor wires were not valid motor measurements. Do not reuse their derived PI
gains. Resistance and inductance should be finite, physically plausible for your
motor, and reasonably repeatable. The helper does not reject implausible values.

The estimator uses voltage/current for resistance and SI equations for inductance
(ohms and henries). The bandwidth helper divides the reported Rs/Ld/Lq by two
before calculating gains; do not assume an independently measured phase value
has the same convention without checking the measurement method.

## 4. Modes and units

These are TWO different settings:

### Motor control mode: what the motor regulates

```python
motor.set_foc_motor_mode(2)
motor.get_foc_motor_mode()
```

| Number | Mode | Normal use |
| --- | --- | --- |
| 0 | Torque/current | Command current in amperes |
| 1 | Speed | Command mechanical RPM |
| 2 | Position | Command accumulated angle in degrees |
| 3 | Encoder calibration enum | Use `start_calibrate_abs_encoder()` instead of selecting this directly |
| 4 | Audio | Specialized firmware functionality, not a normal motion test |
| 5 | Voltage/open-loop | Advanced direct voltage-vector testing |
| 6 | Disabled | Disable normal motor drive |

### FOC feedback mode: how rotor angle is obtained

```python
motor.set_foc_mode(0)
motor.get_foc_mode()
```

| Number | Mode |
| --- | --- |
| 0 | Sensored; use this for your encoder setup |
| 1 | Sensorless SMO/HFI |
| 2 | New sensorless SMO/HFI |
| 3 | Hybrid |

Stay with sensored mode for these instructions. Sensorless/hybrid operation needs
separate validation for the motor and operating range.

### Units you will see

| Signal | Units / meaning |
| --- | --- |
| `actual_angle`, `pos_ref` | Accumulated mechanical position in degrees; firmware applies configured gear ratio |
| `actual_rpm`, `rpm_ref` | Mechanical revolutions per minute |
| `m_angle_rad`, `m_angle_rad_comp` | Wrapped mechanical radians, approximately 0 to 6.283 |
| `e_rad` | FOC electrical angle in radians; source depends on operating mode |
| `Is_ref`, `id`, `iq`, phase currents | Amperes, subject to correct current-sense scaling |
| `v_bus` | DC bus voltage in volts, subject to correct sensing scale |

At gear ratio 1, 360 mechanical degrees is one shaft revolution. With seven pole
pairs, the electrical angle cycles seven times during one mechanical revolution.

## 5. Current control

Current mode controls torque-producing current. It does not regulate speed or
hold an angle. An unloaded motor can continue accelerating with nonzero current.
Supply current is not the same quantity as controlled motor phase/dq current.

```python
motor.set_foc_motor_mode(6)
motor.set_foc_mode(0)
motor.set_foc_motor_mode(0)
motor.set_foc_current_set_point(0.02)
motor.get_foc_current_set_point()
```

Observe the response before trying a slightly larger command:

```python
motor.set_foc_current_set_point(0.05)
```

Return to zero before testing the opposite sign:

```python
motor.set_foc_current_set_point(0)
motor.set_foc_current_set_point(-0.05)
```

Finish:

```python
motor.set_foc_current_set_point(0)
motor.set_foc_motor_mode(6)
```

Useful plots: `Is_ref`, `iq`, `id`. With field weakening and MTPA disabled,
`iq` should approximately track the signed current request and `id` should remain
near zero. Small commands may not overcome friction or sensor noise.
The speed PID output limit does not provide a general limit for direct current mode.

## 6. Speed control

```python
motor.set_foc_motor_mode(6)
motor.set_foc_mode(0)
motor.set_foc_motor_mode(1)
motor.set_foc_speed_set_point(50)
motor.get_foc_speed_set_point()
```

After verifying tracking, change speed:

```python
motor.set_foc_speed_set_point(100)
```

To reverse, first command zero and wait for the shaft to slow down:

```python
motor.set_foc_speed_set_point(0)
```

Then:

```python
motor.set_foc_speed_set_point(-50)
```

Finish:

```python
motor.set_foc_speed_set_point(0)
motor.set_foc_motor_mode(6)
```

Plot `rpm_ref` and `actual_rpm` together. Zero speed can actively resist rotation
while speed mode remains enabled; disabling removes that active drive.

## 7. Position control

### Move relative to the current position

Start stationary. Enter position mode and subscribe to the degree signals:

```python
motor.set_foc_motor_mode(6)
motor.set_foc_mode(0)
motor.set_foc_motor_mode(2)
motor.plotter_add_line('actual_angle')
motor.plotter_add_line('pos_ref')
```

Let fresh samples arrive before the next commands. Make sure the graph is not
paused. Read the current angle and move forward 30 degrees:

```python
current = plotter.channels['actual_angle']['buffer'][-1]
motor.set_foc_position_set_point(current + 30)
```

The motor should approach the target, stop, and actively hold it. To move another
30 degrees, read `current` again first. A variable does not update itself.

```python
current = plotter.channels['actual_angle']['buffer'][-1]
motor.set_foc_position_set_point(current - 30)
```

Check the commanded target:

```python
motor.get_foc_position_set_point()
```

Check the tracking error after fresh samples have arrived:

```python
motor.get_foc_position_set_point()['value'] - plotter.channels['actual_angle']['buffer'][-1]
```

The result is target minus actual angle in degrees. Plot samples have latency,
so use this for slow bench tests, not synchronized motion control.

### Why `current + 180` worked while `10` looked wrong

`set_foc_position_set_point()` is an ABSOLUTE accumulated-angle command.
If `actual_angle` is 130000 degrees:

| Command target | Requested travel |
| --- | --- |
| `130030` | Forward 30 degrees |
| `130180` | Forward 180 degrees |
| `10` | Backward 129990 degrees, roughly 361 turns |

The firmware does not choose the shortest route modulo 360 degrees.
Do not send `10` to mean "move forward 10 degrees".

### Use your own session zero

While stationary, capture a reference:

```python
home = plotter.channels['actual_angle']['buffer'][-1]
motor.set_foc_position_set_point(home + 30)
```

After arrival, try another offset, then return:

```python
motor.set_foc_position_set_point(home + 90)
```

```python
motor.set_foc_position_set_point(home)
```

`home` is only a Python variable. It does not zero the encoder, persist through
app restarts, or establish physical homing. Recapture it after a board reset.
There is no exposed zero-position command in the current JSON API.

Changing INTO position mode initializes its target to the current angle. Setting
mode 2 again while already in mode 2 does not reset the target.

Release holding torque when finished:

```python
motor.set_foc_motor_mode(6)
```

## 8. Encoder readings

With the drive disabled, turn the shaft by hand and repeat:

```python
motor.set_foc_motor_mode(6)
motor.get_foc_actual_e_rad()
```

This returns the encoder-derived electrical angle in radians, not accumulated
mechanical degrees. A changing value confirms that this reading responds to
rotation; it does not by itself prove correct phase alignment or calibration.

For mechanical-angle inspection:

```python
motor.plotter_add_line('m_angle_rad')
motor.plotter_add_line('m_angle_rad_comp')
motor.plotter_add_line('e_rad')
```

The jump between approximately 6.283 and 0 is normal angle wrapping. In open-loop
mode, the plotted `e_rad` is the commanded FOC angle; the direct getter above
specifically reads `encoder_e_angle_rad`.

## 9. Plot controls

### Add, remove, list, and clear

```python
motor.plotter_add_line('actual_angle')
motor.plotter_remove_line('actual_angle')
motor.plotter_channels
```

Remove every channel known to the current Python connection:

```python
for name in list(motor.plotter_channels): motor.plotter_remove_line(name)
```

There is no implemented `motor.plotter_remove_all_line()` helper in the current
MotorProtocol class, despite a reference to it in the demo code.

Clear sample history without removing subscriptions or changing motor position:

```python
plotter.clear_data()
```

Wait for new samples before using `buffer[-1]` after clearing.

### Reset the range

Enable ongoing automatic Y scaling:

```python
plotter.auto_range_check.setChecked(True)
plotter.auto_range()
```

For wrapped radians only:

```python
plotter.auto_range_check.setChecked(False)
plotter.plot_widget.setYRange(0, 6.5)
```

For a small current test:

```python
plotter.auto_range_check.setChecked(False)
plotter.plot_widget.setYRange(-0.2, 0.2)
```

For accumulated position around the current angle:

```python
current = plotter.channels['actual_angle']['buffer'][-1]
plotter.auto_range_check.setChecked(False)
plotter.plot_widget.setYRange(current - 60, current + 60)
```

Auto Range uses all subscribed channels and their retained history. Clear old
history when a previous extreme value dominates the scale. Do not mix large
accumulated degrees with radians or small currents on the same Y axis.
The live X axis follows the sample window automatically; it is sample count,
not a calibrated time axis.

### Pause and resume

Click **Pause** to freeze the graph and **Resume** to continue, or toggle:

```python
plotter.toggle_pause()
```

This toggles the current state. Pausing the plot does NOT stop the motor.
Use live, freshly updated samples for relative position commands.

### Recommended channel sets

Remove previous channels first, then add the desired group one line at a time.

| Test | Channels |
| --- | --- |
| Current | `Is_ref`, `iq`, `id` |
| Speed | `rpm_ref`, `actual_rpm` |
| Position | `pos_ref`, `actual_angle` |
| Encoder | `m_angle_rad`, `m_angle_rad_comp`, `e_rad` |
| Supply | `v_bus` |

Firmware supports at most **10 simultaneous channels**. Exceeding this can make
the Python channel list disagree with the device even when the reply is zero.

After reconnecting to a board that stayed powered, old device subscriptions may
remain while Python starts a new list. If labels and values seem mismatched,
disable the motor, remove all known dictionary entries, then add only your desired
channels and clear history:

```python
motor.set_foc_motor_mode(6)
for name in list(motor.plotter_dict): motor.plotter_remove_line(name)
motor.plotter_add_line('actual_angle')
motor.plotter_add_line('pos_ref')
plotter.clear_data()
plotter.auto_range_check.setChecked(True)
```

### All available channels

| Group | Names |
| --- | --- |
| Phase currents | `ia`, `ib`, `ic` |
| Stationary-frame currents | `i_alpha`, `i_beta` |
| Rotating-frame currents | `id`, `iq` |
| Phase voltage fields | `va`, `vb`, `vc` |
| Stationary-frame voltage fields | `v_alpha`, `v_beta` |
| Rotating-frame voltage fields | `vd`, `vq` |
| Electrical angle | `e_rad` |
| Speed | `actual_rpm`, `rpm_ref` |
| Position | `actual_angle`, `pos_ref` |
| Current reference | `Is_ref` |
| Mechanical angle | `m_angle_rad`, `m_angle_rad_comp` |
| Supply voltage | `v_bus` |

Voltage fields are controller variables; do not assume they are independently
measured motor-terminal voltages.

## 10. PID settings

Read existing settings before changing anything:

```python
motor.get_pid_id()
motor.get_pid_iq()
motor.get_pid_speed()
motor.get_pid_position()
```

The control cascade is position -> speed reference -> current reference -> drive.

| Setting | What it controls |
| --- | --- |
| Position `out_max` | Maximum magnitude of the speed reference, in RPM |
| Speed `out_max` | Maximum magnitude of its current reference, in A |
| Position `deadband` | Position error band, in degrees |
| Speed `deadband` | Speed error band, in RPM |
| Current `deadband` | Current error band, in A |
| Position `d_fc` | Derivative low-pass filter cutoff, in Hz |

The source default position output limit is only **3 RPM**, so a long position
move can be very slow. The source default speed-controller output limit is
**10 A**, which is not an appropriate assumed limit for every bench motor.
Read your saved values; they may differ from the defaults.

Example: preserve position gains but change the speed cap to 20 RPM after
confirming that speed is appropriate for your setup. Stop/disable first:

```python
motor.set_foc_motor_mode(6)
p = motor.get_pid_position()
motor.set_pid_position(p['kp'], p['ki'], p['kd'], 20.0, p['deadband'], p['d_fc'])
motor.get_pid_position()
```

Example: preserve speed gains but set its output cap to 0.2 A, only if that current
is appropriate for your motor. This may be too little to move a loaded motor:

```python
s = motor.get_pid_speed()
motor.set_pid_speed(s['kp'], s['ki'], 0.2, s['deadband'])
motor.get_pid_speed()
```

Change one parameter at a time and retest with small commands. Increasing a cap
does not fix wrong encoder alignment, phase order, or invalid current sensing.
Do not copy arbitrary gains to cure those problems. Save only validated settings.

The helper below recalculates and writes current PI gains; it is not a getter:

```python
motor.set_foc_bandwidth(500)
```

Use it only with valid Rs/Ld/Lq measurements and a deliberately chosen bandwidth.

## 11. Save, reopen, and reset

### Save validated configuration

```python
motor.set_foc_motor_mode(6)
motor.save_config()
```

The save path copies motor mode as well as motor parameters and controller
settings into storage. Saving while disabled avoids deliberately persisting an
active mode. Encoder calibration data lives in the stored configuration as well.
Saving is not required for each motion target or plot change.

Reopening the app does not require self-commissioning if the board still has a
valid saved configuration. A board reset reloads stored configuration. Reconnect,
disable, inspect settings, and select the mode you want to use. Python variables
such as `home`, plot history, and accumulated multi-turn tracking are not a
persistent physical homing system.

### Restore defaults only intentionally

```python
motor.set_foc_motor_mode(6)
motor.set_default_config()
```

This replaces configuration in RAM with defaults, including zero motor Rs/Ld/Lq
and cleared encoder calibration storage fields. It is not a routine reconnect or
graph-reset command. Recommission before driving, then save the validated result.
Do not use it when all you need is `plotter.clear_data()`.

## 12. Troubleshooting

| Symptom | Check / action |
| --- | --- |
| Setpoint returns zero but nothing happens | Read `get_foc_motor_mode()`; select the matching mode BEFORE sending the target |
| Position slowly spins for a long time | Compare accumulated `actual_angle` with target; use a fresh `current + offset` |
| Position moves and stops, then resists movement | Expected position holding behavior |
| Position move is extremely slow | Read position PID `out_max`; default is 3 RPM |
| A 10-degree command barely moves | Check target error, current limit, friction, tuning, and plot scale before increasing commands |
| Angle looks flat near 130000 | Use a narrow Y range around current angle; do not mix degree and radian signals |
| Electrical angle jumps between 0 and 6.283 | Normal wrapped angle behavior |
| Encoder getter changes but graph does not | Resume plot, verify subscriptions/labels, and reset the range |
| `KeyError: 'actual_angle'` | Add that channel and allow fresh data to arrive |
| Empty-buffer `IndexError` | Wait for samples after adding/clearing; verify connection and Resume state |
| Plot values make no sense after reconnect | Remove all dictionary channels and re-add the desired group, as in section 9 |
| Current mode keeps accelerating | It regulates current, not speed; command zero and disable |
| Motor buzzes, jerks, or draws excessive current | Disable; inspect phase wiring, encoder alignment, commissioning, and current sensing |
| Rs/Ld/Lq are huge or change wildly | Check powered DC bus, all phases, measurement scaling, and connections; do not save invalid gains |
| No motion on USB alone | Verify the separate motor DC supply and power stage |
| USB disconnects or commands time out | Stop through the supply if needed; check power/noise, cable, clock source, and reconnect |
| Closing app leaves motor running | App disconnect is not motor disable; disable before closing |

The bundled demo methods are not recommended with this revision:
`MotorSequenceThread` calls the missing `plotter_remove_all_line()` helper;
`run_motor_current_demo()` selects speed mode; and the position demo uses absolute
targets around zero. Use the explicit manual sequences in this guide.
Avoid long loops with `time.sleep()` in the GUI console: they block the GUI thread
and prevent timely interaction with its controls.

## 13. Complete command reference

These are signatures, not a sequence to execute. Names such as `value`, `kp`, or
`idx` are placeholders. Use positional arguments in the order shown.
All commands below are called on `motor`.

### Configuration and mode commands

| Signature | Purpose |
| --- | --- |
| `set_default_config()` | Replace runtime configuration with defaults |
| `save_config()` | Persist current configuration; disable first |
| `set_foc_mode(mode)` / `get_foc_mode()` | Select/read feedback strategy |
| `set_foc_motor_mode(mode)` / `get_foc_motor_mode()` | Select/read control mode |
| `set_pole_pairs(value)` / `get_pole_pairs()` | Motor pole-pair count |
| `set_kv(value)` / `get_kv()` | Motor KV parameter |
| `set_rs(value)` / `get_rs()` | Resistance parameter |
| `set_ld(value)` / `get_ld()` | d-axis inductance parameter |
| `set_lq(value)` / `get_lq()` | q-axis inductance parameter |
| `set_flux_linkage(value)` / `get_flux_linkage()` | Flux-linkage parameter |

Manual electrical-parameter setters are advanced overrides. Do not enter guessed
values or interchange millihenries and henries.

### Controllers

| Signature | Purpose |
| --- | --- |
| `set_pid_id(kp, ki, deadband)` / `get_pid_id()` | d-axis current PI |
| `set_pid_iq(kp, ki, deadband)` / `get_pid_iq()` | q-axis current PI |
| `set_pid_speed(kp, ki, out_max, deadband)` / `get_pid_speed()` | Speed PI; output cap in A |
| `set_pid_position(kp, ki, kd, out_max, deadband, d_fc)` / `get_pid_position()` | Position PID; output cap in RPM |
| `set_field_weakening(kp, ki, out_min)` / `get_field_weakening()` | Advanced field-weakening controller |
| `set_field_weakening_enable(enable)` / `get_field_weakening_enable()` | 0 disables, 1 enables field weakening |
| `set_mtpa_enable(enable)` / `get_mtpa_enable()` | 0 disables, 1 enables maximum-torque-per-ampere logic |

Field weakening and MTPA default to disabled. Leave them disabled for the basic
tests; they require validated motor parameters and operating limits.

### Targets and feedback

| Signature | Purpose |
| --- | --- |
| `set_foc_current_set_point(value)` / `get_foc_current_set_point()` | Current target in A; setter requires motor mode 0 |
| `set_foc_speed_set_point(value)` / `get_foc_speed_set_point()` | Speed target in RPM; setter requires motor mode 1 |
| `set_foc_position_set_point(value)` / `get_foc_position_set_point()` | Absolute accumulated-degree target; setter requires motor mode 2 |
| `get_foc_actual_e_rad()` | Encoder electrical angle in radians |
| `set_svpwm(vd, vq, e_rad)` / `get_svpwm()` | Direct voltage-vector fields; setter requires motor mode 5 |

Setpoint getters read the target, not actual motion. Use plot channels for actual
speed, accumulated position, and current. There are no corresponding direct
actual-speed/actual-position getters in this JSON API.
`set_svpwm()` specifies voltage and electrical angle, not RPM. Direct voltage can
produce large current at standstill; it is not needed for normal closed-loop use.

### Measurement and plotting

| Signature | Purpose |
| --- | --- |
| `start_measure_motor_Rs()` | Start resistance measurement |
| `start_measure_motor_Ld()` | Start d-axis inductance measurement |
| `start_measure_motor_Lq()` | Start q-axis inductance measurement |
| `start_calibrate_abs_encoder()` | Start encoder calibration motion |
| `get_abs_encoder_error_comp(idx)` | Read encoder correction table entry; use an index within the firmware LUT size |
| `plotter_add_line('name')` | Subscribe to a channel |
| `plotter_remove_line('name')` | Unsubscribe from a channel |

Measurement start commands return before the physical process finishes. Do not
launch overlapping measurements. Use the commissioning workflow unless diagnosing
the individual stages deliberately.

### Python convenience helpers

| Signature | Purpose |
| --- | --- |
| `self_commissioning()` | Measure parameters, calculate current gains, start encoder calibration |
| `get_motor_param()` | Print pole pairs, Rs, Ld, Lq |
| `set_foc_bandwidth(bw=100)` | Recalculate and write current PI gains; bandwidth in Hz |

Discover available names and argument metadata in the console:

```python
sorted(motor.commands)
motor.commands['set_pid_position']
sorted(motor.plotter_dict)
```

## 14. HSI clock notes

Your current `SystemClock_Config()` explicitly selects HSI and PLL settings
M=8, N=168, P=2, Q=7. From nominal 16 MHz HSI, these produce nominal 168 MHz system
clock and 48 MHz USB clock. The configuration is intentional, not simply an HSE
12 MHz setup running unchanged on HSI.

Nominal frequency does not guarantee clock accuracy. HSI accuracy varies with
device and operating conditions; the PLL carries that reference error into its
outputs. That can affect communication timing and real-time speed estimates.
An encoder position reading does not simply acquire the same percentage error
as the clock, because it is derived from measured angle.

For dependable USB operation, use a correctly specified external clock solution
and matching firmware configuration. Bench success with HSI does not establish
operation across temperature and supply variation. Check CAN timing margins too
before using the board on a network.

Reference: [ST RM0090 reference manual, clock-control and USB sections](https://www.st.com/resource/en/reference_manual/dm00031020-stm32f405-407-415-417-437-455-469-application-note-stmicroelectronics.pdf).

## 15. Implementation references

- [Command names, arguments, and plot dictionary](communication/motor_commands.json)
- [Python protocol and commissioning helper](communication/MotorProtocol.py)
- [Plot controls and connection behavior](communication/plotter.py)
- [FOC modes and data structures](lib/FOC/FOC_utils.h)
- [Setpoint guards, mode transitions, and control cascade](lib/FOC/FOC_utils.c)
- [Firmware command handlers and plotted fields](lib/COM/com.c)
- [Saved configuration and defaults](lib/STORAGE/storage.c)
- [Measurement and encoder-calibration implementation](lib/SELF_COMMISSIONING/self_commissioning.c)
- [Board initialization and HSI clock configuration](src/Core/Src/main.c)

This guide was checked against source; its motion examples were not executed on
your physical hardware during documentation generation.
