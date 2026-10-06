# TEAM: QArm Low-Level Control - Part 3: Wrist Current Mode (Hardware)
# TEAM: Physical hardware current control and asymmetry analysis. The manipulator is elevated safely above the base,
# and the wrist joint is operated in closed-loop Current Mode (j3_mode=254) with amplitudes from ±10° to ±40°.
# Students observe how internal wiring harness tension and mechanical spring bias create asymmetric current demands.

# region: Python level imports
import os
import sys
import time
import signal
from threading import Thread
import numpy as np

from PyQt6.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QLabel, QPushButton, QRadioButton, QButtonGroup, QGroupBox,
    QCheckBox
)
from PyQt6.QtCore import Qt, QTimer

# Ensure modified_library is discoverable from any directory
PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..'))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from modified_library.scope import Scope, XYScope, MultiScope, TYAxis
from modified_library.qarm import QArm
# endregion


# region: Scope constants
# TEAM: Define any constants needed for using the quanser scope here. Leave blank if not using the scope.
scopeFps = 30
scopeBufferSize = 500
scopeTimeWindow = 10.0

# Fixed Y-axis limits prevent misleading micro-scale auto-zooming around near-zero values
wristPosLim = [-1.0, 1.0]         # Wrist position limits (rad) [approx ±57°]
currentLim = [-0.065, 0.025]      # Motor current limits (A) [-50 mA to +15 mA asymmetric operating range]
wristSpeedLim = [-1.5, 1.5]       # Wrist angular speed limits (rad/s)
pwmLim = [-0.6, 0.6]              # PWM duty cycle limits (-0.6 to 0.6)
tempLim = [15.0, 50.0]            # Motor temperature limits (°C)
# endregion


# region: Scope setup or Probe/Observer setup (pick one)
# TEAM: Setup any scopes or probes/observers needed for your experiment here. Leave blank if not using the scope.
multiScope = MultiScope(
    rows=2,
    cols=3,
    title='QArm Hardware Low-Level Control - Part 3: Wrist Current Mode & Bias',
    fps=scopeFps,
    bufferSize=scopeBufferSize
)

# Axis 0: Wrist Commanded vs Measured Position
multiScope.addAxis(row=0, col=0, timeWindow=scopeTimeWindow, yLabel='Wrist Pos (rad)', yLim=wristPosLim)
multiScope.axes[0].attachSignal(name='wristCmd')
multiScope.axes[0].attachSignal(name='wristMeas')

# Axis 1: Wrist Commanded vs Measured Current (Directly reveals cable harness asymmetry!)
multiScope.addAxis(row=0, col=1, timeWindow=scopeTimeWindow, yLabel='Current (A)', yLim=currentLim)
multiScope.axes[1].attachSignal(name='currentCmd')
multiScope.axes[1].attachSignal(name='currentMeas')

# Axis 2: Wrist Angular Speed
multiScope.addAxis(row=0, col=2, timeWindow=scopeTimeWindow, yLabel='Speed (rad/s)', yLim=wristSpeedLim)
multiScope.axes[2].attachSignal(name='wristSpeed')

# Axis 3: PWM Duty Cycle (Shows driver response)
multiScope.addAxis(row=1, col=0, timeWindow=scopeTimeWindow, yLabel='PWM Duty Cycle', yLim=pwmLim)
multiScope.axes[3].attachSignal(name='wristPWM')

# Axis 4: Current Tracking Error (Cmd - Meas)
multiScope.addAxis(row=1, col=1, timeWindow=scopeTimeWindow, yLabel='Current Error (A)', yLim=currentLim)
multiScope.axes[4].attachSignal(name='currentError')

# Axis 5: Motor Temperature
multiScope.addAxis(row=1, col=2, timeWindow=scopeTimeWindow, yLabel='Temp (°C)', yLim=tempLim)
multiScope.axes[5].attachSignal(name='wristTemp')
# endregion


# region: Experiment constants
# TEAM: Define any constants needed for your experiment here. This is ideally where students change values and see the result based on the lab

hardwareMode = 1                  # 1 for Physical Hardware
readMode = 0                      # 0 for Immediate I/O, 1 for Task-based I/O
jointMode = [0, 0, 0, 254]        # Hybrid Mode: Joints 0-2 Position Mode, Joint 3 Current Mode (254)
scopeSampleInterval = 0.02        # Sampling rate for scope data collection (50 Hz)
defaultLedColor = [0, 1, 0]       # Base LED color (Green)

# Safe elevated pose for Base, Shoulder, Elbow
ELEVATED_POSE = np.array([0.0, -1.5, 1.2, 0.0], dtype=np.float64)
PULSE_PERIOD = 10.0               # Seconds per full cycle (1/10 Hz = 5s positive, 5s negative, Simulink block 1094)
CMD_SLEW_RATE = np.pi / 4.0       # Rate Limiter on position command (45 deg/s = 0.785 rad/s, block 1048)
CURRENT_SLEW_RATE = 1.0           # Slew Rate Limiter on current command (1.0 A/s, block 1180)
I_MIN = -0.050                    # Dynamic saturation lower clamp (-0.050 A = -50 mA against cable spring, block 1159)
I_MAX = 0.015                     # Dynamic saturation upper clamp (+0.015 A = +15 mA aided by cable spring, block 1158)
# endregion


# region: Main Loop
# TEAM: Implement the main code for your experiment here. Use comments to guide students through the steps needed to complete the lab.

# 1. Kill switch setup for clean exit on Ctrl+C
KILL_THREAD = False
def sig_handler(*args):
    global KILL_THREAD
    KILL_THREAD = True
signal.signal(signal.SIGINT, sig_handler)

# 2. Hardware Initialization in Hybrid Position/Current Mode
myArm = QArm(hardware=hardwareMode, readMode=readMode, jointMode=jointMode)

# Controller internal states
commanded_wrist_pos = 0.0
applied_wrist_current = 0.0
last_current_cmd = 0.0
integral_error = 0.0
last_pos_err = 0.0
speed_f1 = 0.0
filtered_speed = 0.0
cmd_start_time = None
last_control_time = time.time()

desired_LED_color = list(defaultLedColor)
myArm.read_write_current(
    currentCMD=0.0,
    phiCMD=ELEVATED_POSE.tolist(),
    baseLED=desired_LED_color
)

# 3. Background Data Collection Thread
def scope_loop():
    """Samples hardware values and pushes them to the MultiScope plots."""
    t0 = time.time()

    while not KILL_THREAD:
        t = time.time() - t0
        meas_phi = np.array(myArm.measJointPosition)[:4]
        meas_phidot = np.array(myArm.measJointSpeed)[:4]
        meas_current = np.array(myArm.measJointCurrent)[:4]
        meas_pwm = np.array(myArm.measJointPWM)[:4]
        meas_temp = np.array(myArm.measJointTemperature)[:4]

        curr_err = applied_wrist_current - meas_current[3]

        # Axis 0: Wrist Position Cmd vs Meas
        multiScope.axes[0].sample(t, [commanded_wrist_pos, meas_phi[3]])
        # Axis 1: Commanded vs Measured Wrist Current
        multiScope.axes[1].sample(t, [applied_wrist_current, meas_current[3]])
        # Axis 2: Wrist Angular Speed
        multiScope.axes[2].sample(t, meas_phidot[3])
        # Axis 3: Wrist PWM
        multiScope.axes[3].sample(t, meas_pwm[3])
        # Axis 4: Current Error
        multiScope.axes[4].sample(t, curr_err)
        # Axis 5: Wrist Temperature
        multiScope.axes[5].sample(t, meas_temp[3])

        time.sleep(scopeSampleInterval)

thread = Thread(target=scope_loop, daemon=True)
thread.start()

# 4. PyQt6 Custom Control Panel
app = QApplication.instance()
if app is None:
    app = QApplication([])

control_window = QWidget()
control_window.setWindowTitle("QArm Hardware - Part 3: Wrist Current Mode & Bias")
control_window.resize(540, 560)

main_layout = QVBoxLayout()
control_window.setLayout(main_layout)

title_label = QLabel(
    "<b>QArm Low-Level Control - Part 3: Wrist Current Mode</b><br>"
    "<small>Study wrist motor current and observe cable harness asymmetry across amplitudes</small>"
)
title_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
main_layout.addWidget(title_label)

# Switches Group
switch_group = QGroupBox("Manual Controller Switches")
switch_layout = QHBoxLayout()
switch_group.setLayout(switch_layout)

chk_enable_controller = QCheckBox("Enable Current Controller")
chk_enable_controller.setChecked(False)

chk_enable_cmd = QCheckBox("Enable Square Wave Cmd (10s Period)")
chk_enable_cmd.setChecked(False)

switch_layout.addWidget(chk_enable_controller)
switch_layout.addWidget(chk_enable_cmd)
main_layout.addWidget(switch_group)

# Amplitude Selection Group (Step 11 of Lab Manual)
amp_group = QGroupBox("Pulse Amplitude Selection (Step 11)")
amp_layout = QVBoxLayout()
amp_group.setLayout(amp_layout)

rb_10 = QRadioButton("Amplitude: ±10° (0.175 rad)")
rb_20 = QRadioButton("Amplitude: ±20° (0.349 rad)")
rb_30 = QRadioButton("Amplitude: ±30° (0.524 rad)")
rb_40 = QRadioButton("Amplitude: ±40° (0.698 rad)")

rb_10.setChecked(True)

grp_amp = QButtonGroup()
grp_amp.addButton(rb_10, 10)
grp_amp.addButton(rb_20, 20)
grp_amp.addButton(rb_30, 30)
grp_amp.addButton(rb_40, 40)

amp_layout.addWidget(rb_10)
amp_layout.addWidget(rb_20)
amp_layout.addWidget(rb_30)
amp_layout.addWidget(rb_40)
main_layout.addWidget(amp_group)

# Info Card explaining cable harness asymmetry & saturation
info_box = QGroupBox("Physics Note: Cable Harness Asymmetry & Saturation Limits")
info_layout = QVBoxLayout()
info_box.setLayout(info_layout)
info_text = QLabel(
    "• <b>Positive Motion (+):</b> Assisted by internal cable spring tension.<br>"
    "&nbsp;&nbsp;Current clamped to <b>+0.015 A (+15 mA)</b> | Gains: Kp=0.010, Ki=0.030, Kd=0.007<br>"
    "• <b>Negative Motion (-):</b> Opposed by internal cable spring tension.<br>"
    "&nbsp;&nbsp;Current clamped to <b>-0.050 A (-50 mA)</b> | Gains: Kp=0.007, Ki=0.090, Kd=0.012<br>"
    "• <b>Integrator Anti-Windup:</b> Resets to 0 on zero-crossings & inside ±0.005 rad deadband."
)
info_text.setStyleSheet("color: #444; font-size: 11px;")
info_layout.addWidget(info_text)
main_layout.addWidget(info_box)

# Live Status telemetry display
status_label = QLabel("Status: Controller Disabled | Commanded: 0.000 rad | Current: 0.0 mA")
status_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
status_label.setStyleSheet("font-family: monospace; font-weight: bold; padding: 4px; background: #eef2f7; border-radius: 4px;")
main_layout.addWidget(status_label)

# Emergency Stop / Disable Button
btn_stop = QPushButton("Emergency Stop / Zero Current")
btn_stop.setStyleSheet("background-color: #d9534f; color: white; font-weight: bold; padding: 6px;")
def emergency_stop():
    chk_enable_controller.setChecked(False)
    chk_enable_cmd.setChecked(False)
btn_stop.clicked.connect(emergency_stop)
main_layout.addWidget(btn_stop)

control_window.show()

# 5. Periodic Control Loop via QTimer (30 Hz)
start_time = time.time()

def control_step():
    global commanded_wrist_pos, applied_wrist_current, last_current_cmd
    global integral_error, last_pos_err, speed_f1, filtered_speed
    global cmd_start_time, last_control_time

    now = time.time()
    dt = max(min(now - last_control_time, 0.1), 1e-4)
    last_control_time = now

    amp_deg = grp_amp.checkedId()
    amp_rad = np.radians(amp_deg)

    if chk_enable_cmd.isChecked():
        if cmd_start_time is None:
            cmd_start_time = now
        t_cmd = now - cmd_start_time
        phase = (t_cmd % PULSE_PERIOD) / PULSE_PERIOD
        sign = 1.0 if phase < 0.5 else -1.0
        target_step = sign * amp_rad
    else:
        cmd_start_time = None
        target_step = 0.0

    # Rate-limited position command (45 deg/s = 0.785 rad/s, Simulink block 1048)
    max_step = CMD_SLEW_RATE * dt
    commanded_wrist_pos = float(np.clip(target_step, commanded_wrist_pos - max_step, commanded_wrist_pos + max_step))

    meas_phi = np.array(myArm.measJointPosition)[:4]
    meas_phidot = np.array(myArm.measJointSpeed)[:4]

    wrist_meas = meas_phi[3]
    raw_speed = meas_phidot[3]

    # 2nd-order critically damped low-pass filter at 5 Hz (Simulink block 1011)
    # Omega_n = 2 * pi * 5 = 31.416 rad/s, zeta = 1.0
    alpha = float(1.0 - np.exp(-2.0 * np.pi * 5.0 * dt))
    speed_f1 += alpha * (raw_speed - speed_f1)
    filtered_speed += alpha * (speed_f1 - filtered_speed)

    pos_err = commanded_wrist_pos - wrist_meas

    if chk_enable_controller.isChecked():
        # Deadband check (0.005 rad ~ 0.29 deg, Simulink block 1085)
        if abs(pos_err) < 0.005:
            effective_err = 0.0
            # Reset integrator inside deadband or near zero to prevent limit-cycle hunting
            integral_error = 0.0
        else:
            effective_err = pos_err

        # Simulink block 1045: ExternalReset is triggered on sign reversal (zero-crossing)
        if (pos_err * last_pos_err < 0.0):
            integral_error = 0.0
        else:
            integral_error += effective_err * dt
            integral_error = float(np.clip(integral_error, -0.5, 0.5))

        # Asymmetric PID gains (Simulink Switch block 1150 driven by Sign(commanded_wrist_pos)):
        # When commanding positive wrist angle (> 0): aided by internal cable spring
        # When commanding negative wrist angle (<= 0): opposed by internal cable spring
        if commanded_wrist_pos > 0.0:
            kp, ki, kd = 0.010, 0.030, 0.007
        else:
            kp, ki, kd = 0.007, 0.090, 0.012

        # Raw PID Current command (Amperes)
        # Note: Derivative term acts on filtered velocity (-kd * filtered_speed)
        i_raw = (kp * effective_err) + (ki * integral_error) - (kd * filtered_speed)

        # Slew rate limiter on current (Simulink Rate Limiter2 block 1180: 1.0 A/s)
        max_di = CURRENT_SLEW_RATE * dt
        i_slewed = float(np.clip(i_raw, last_current_cmd - max_di, last_current_cmd + max_di))
        last_current_cmd = i_slewed

        # Dynamic saturation clamp (Simulink Saturation Dynamic block 1184):
        # Clamped strictly between I_MIN (-0.050 A) and I_MAX (+0.015 A)
        i_saturated = float(np.clip(i_slewed, I_MIN, I_MAX))

        # Soft catastrophic safety limiter: if angle exceeds safe bounds, cut current towards limit
        if wrist_meas > 1.4 and i_saturated > 0.0:
            i_saturated = 0.0
        elif wrist_meas < -1.4 and i_saturated < 0.0:
            i_saturated = 0.0
        if abs(wrist_meas) > 1.6:
            i_saturated = 0.0

        applied_wrist_current = i_saturated
    else:
        applied_wrist_current = 0.0
        integral_error = 0.0
        last_current_cmd = 0.0

    last_pos_err = pos_err

    # Update GUI status label
    status_text = f"Cmd: {commanded_wrist_pos:+.3f} rad ({np.degrees(commanded_wrist_pos):+.1f}°) | Meas: {wrist_meas:+.3f} rad ({np.degrees(wrist_meas):+.1f}°) | Current: {applied_wrist_current * 1000.0:+.1f} mA"
    status_label.setText(status_text)

    # Send current command to wrist on analog channel 3; joints 0-2 hold elevated pose
    myArm.read_write_current(
        currentCMD=applied_wrist_current,
        phiCMD=ELEVATED_POSE.tolist(),
        gprCMD=0.0,
        baseLED=desired_LED_color
    )

    # Refresh MultiScope plots
    MultiScope.refreshAll()

# Clean shutdown handler
def close_event(event):
    global KILL_THREAD
    KILL_THREAD = True
    myArm.read_write_current(0.0, phiCMD=[0.0, 0.0, 0.0, 0.0], baseLED=[1, 0, 0])
    myArm.terminate()
    event.accept()

control_window.closeEvent = close_event

# 6. Start Qt Timer and Event Loop
timer = QTimer()
timer.timeout.connect(control_step)
timer.start(int(1000 / scopeFps))

app.exec()

# 7. Clean Shutdown Sequence
KILL_THREAD = True
myArm.read_write_current(0.0, phiCMD=[0.0, 0.0, 0.0, 0.0], baseLED=[1, 0, 0])
myArm.terminate()
print("\n[Hardware Part 3] Shut down gracefully.")
# endregion
