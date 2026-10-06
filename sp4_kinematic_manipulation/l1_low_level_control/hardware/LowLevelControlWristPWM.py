# TEAM: QArm Low-Level Control - Part 2: Hybrid Position/PWM Mode (Hardware)
# TEAM: Physical hardware PD tuning with actuator safety envelopes. Joints 0 to 2 are held in safe elevated
# Position Mode while the Wrist is driven in closed-loop PWM mode. Students tune Kp and Kd, configure PWM Saturation
# to constrain velocity under ±90 deg/s, and tune a Slew Rate Limiter to constrain acceleration under ±60 deg/s².

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
    QDoubleSpinBox, QCheckBox
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
scopeTimeWindow = 6.0

# Fixed Y-axis limits prevent misleading micro-scale auto-zooming around near-zero values
wristPosLim = [-1.5, 1.5]         # Wrist position limits (rad)
wristSpeedLim = [-2.5, 2.5]       # Wrist angular speed limits (rad/s) [±90 deg/s is ±1.57 rad/s]
wristAccelLim = [-5.0, 5.0]       # Wrist angular acceleration limits (rad/s²) [±60 deg/s² is ±1.05 rad/s²]
pwmLim = [-1.05, 1.05]            # PWM duty cycle limits (-1.0 to 1.0)
slewLim = [-8.0, 8.0]             # PWM slew rate limits (PWM/s) [cleanly displays limits around 2.0-5.0]
currentLim = [-0.6, 0.6]          # Motor current limits (A) [shows no-load wrist current dynamics clearly]
# endregion


# region: Scope setup or Probe/Observer setup (pick one)
# TEAM: Setup any scopes or probes/observers needed for your experiment here. Leave blank if not using the scope.
multiScope = MultiScope(
    rows=2,
    cols=3,
    title='QArm Hardware Low-Level Control - Part 2: Wrist PD & Safety Limits',
    fps=scopeFps,
    bufferSize=scopeBufferSize
)

# Axis 0: Wrist Commanded vs Measured Position
multiScope.addAxis(row=0, col=0, timeWindow=scopeTimeWindow, yLabel='Wrist Pos (rad)', yLim=wristPosLim)
multiScope.axes[0].attachSignal(name='wristCmd')
multiScope.axes[0].attachSignal(name='wristMeas')

# Axis 1: Wrist Angular Speed (Safety limit ±1.57 rad/s = ±90 deg/s)
multiScope.addAxis(row=0, col=1, timeWindow=scopeTimeWindow, yLabel='Speed (rad/s)', yLim=wristSpeedLim)
multiScope.axes[1].attachSignal(name='wristSpeed')

# Axis 2: Wrist Angular Acceleration (Safety limit ±1.05 rad/s² = ±60 deg/s²)
multiScope.addAxis(row=0, col=2, timeWindow=scopeTimeWindow, yLabel='Accel (rad/s²)', yLim=wristAccelLim)
multiScope.axes[2].attachSignal(name='wristAccel')

# Axis 3: PWM Duty Cycle (Shows Raw Saturated Step vs Slewed Output)
multiScope.addAxis(row=1, col=0, timeWindow=scopeTimeWindow, yLabel='PWM Duty Cycle', yLim=pwmLim)
multiScope.axes[3].attachSignal(name='rawPWM')
multiScope.axes[3].attachSignal(name='slewedPWM')

# Axis 4: Wrist Motor Current
multiScope.addAxis(row=1, col=1, timeWindow=scopeTimeWindow, yLabel='Current (A)', yLim=currentLim)
multiScope.axes[4].attachSignal(name='wristCurrent')

# Axis 5: PWM Slew Rate (dPWM/dt vs Slew Limit Envelopes)
multiScope.addAxis(row=1, col=2, timeWindow=scopeTimeWindow, yLabel='Slew Rate (PWM/s)', yLim=slewLim)
multiScope.axes[5].attachSignal(name='posSlewLimit')
multiScope.axes[5].attachSignal(name='negSlewLimit')
multiScope.axes[5].attachSignal(name='actualSlewRate')
# endregion


# region: Experiment constants
# TEAM: Define any constants needed for your experiment here. This is ideally where students change values and see the result based on the lab

hardwareMode = 1                  # 1 for Physical Hardware
readMode = 0                      # 0 for Immediate I/O, 1 for Task-based I/O
jointMode = [0, 0, 0, 1]          # Hybrid Mode: Joints 0-2 Position Mode, Joint 3 PWM Mode
scopeSampleInterval = 0.02        # Sampling rate for scope data collection (50 Hz)
defaultLedColor = [0, 1, 0]       # Base LED color (Green)

# Safe elevated pose for Base, Shoulder, Elbow
ELEVATED_POSE = np.array([0.0, -1.5, 1.2, 0.0], dtype=np.float64)
PULSE_AMPLITUDE = np.pi / 4.0     # ±45 degrees (±0.785 rad)
PULSE_PERIOD = 6.0                # Seconds per full cycle

# Default starting controller parameters
DEFAULT_KP = 1.0                  # Initial proportional gain
DEFAULT_KD = 0.05                 # Initial derivative gain
DEFAULT_SATURATION = 0.40         # Initial PWM saturation limit (-0.40 to +0.40)
DEFAULT_SLEW_RATE = 2.0           # Maximum PWM change rate per second (units/s)
# endregion


# region: Main Loop
# TEAM: Implement the main code for your experiment here. Use comments to guide students through the steps needed to complete the lab.

# 1. Kill switch setup for clean exit on Ctrl+C
KILL_THREAD = False
def sig_handler(*args):
    global KILL_THREAD
    KILL_THREAD = True
signal.signal(signal.SIGINT, sig_handler)

# 2. Hardware Initialization in Hybrid Position/PWM Mode
myArm = QArm(hardware=hardwareMode, readMode=readMode, jointMode=jointMode)
commanded_wrist_pos = 0.0
raw_wrist_pwm = 0.0
applied_wrist_pwm = 0.0
last_applied_pwm = 0.0
current_slew_rate = 0.0
active_slew_limit = DEFAULT_SLEW_RATE
last_control_time = time.time()

# Variables for filtered acceleration
last_speed = 0.0
last_speed_time = time.time()
meas_accel = 0.0

desired_LED_color = list(defaultLedColor)
myArm.read_write_pwm(
    pwmCMD=[0.0, 0.0, 0.0, 0.0],
    phiCMD=ELEVATED_POSE.tolist(),
    baseLED=desired_LED_color
)

# 3. Background Data Collection Thread
def scope_loop():
    """Samples hardware values and pushes them to the MultiScope plots."""
    global last_speed, last_speed_time, meas_accel
    t0 = time.time()

    while not KILL_THREAD:
        t = time.time() - t0
        meas_phi = np.array(myArm.measJointPosition)[:4]
        meas_phidot = np.array(myArm.measJointSpeed)[:4]
        meas_current = np.array(myArm.measJointCurrent)[:4]

        now = time.time()
        dt = max(now - last_speed_time, 1e-4)
        raw_accel = (meas_phidot[3] - last_speed) / dt
        meas_accel = 0.8 * meas_accel + 0.2 * raw_accel
        last_speed = meas_phidot[3]
        last_speed_time = now

        # Axis 0: Wrist Commanded vs Measured Position
        multiScope.axes[0].sample(t, [commanded_wrist_pos, meas_phi[3]])
        # Axis 1: Wrist Speed
        multiScope.axes[1].sample(t, meas_phidot[3])
        # Axis 2: Wrist Acceleration
        multiScope.axes[2].sample(t, meas_accel)
        # Axis 3: PWM Duty Cycle (Raw Saturated vs Slewed)
        multiScope.axes[3].sample(t, [raw_wrist_pwm, applied_wrist_pwm])
        # Axis 4: Wrist Current
        multiScope.axes[4].sample(t, meas_current[3])
        # Axis 5: Slew Rate vs Slew Limits
        multiScope.axes[5].sample(t, [active_slew_limit, -active_slew_limit, current_slew_rate])

        time.sleep(scopeSampleInterval)

thread = Thread(target=scope_loop, daemon=True)
thread.start()

# 4. PyQt6 Custom Control Panel
app = QApplication.instance()
if app is None:
    app = QApplication([])

control_window = QWidget()
control_window.setWindowTitle("QArm Hardware - Part 2: Wrist Hybrid PWM & Safety Tuning")
control_window.resize(540, 560)

main_layout = QVBoxLayout()
control_window.setLayout(main_layout)

title_label = QLabel(
    "<b>QArm Low-Level Control - Part 2: Wrist Hybrid PWM</b><br>"
    "<small>Tune Kp, Kd, PWM Saturation (speed limit), and Slew Rate Limiter (accel limit)</small>"
)
title_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
main_layout.addWidget(title_label)

# Switches Group (matching manual switches in Simulink model)
switch_group = QGroupBox("Manual Controller Switches")
switch_layout = QHBoxLayout()
switch_group.setLayout(switch_layout)

chk_enable_controller = QCheckBox("Enable PD Controller")
chk_enable_controller.setChecked(False)

chk_enable_cmd = QCheckBox("Enable ±45° Pulse Cmd")
chk_enable_cmd.setChecked(False)

switch_layout.addWidget(chk_enable_controller)
switch_layout.addWidget(chk_enable_cmd)
main_layout.addWidget(switch_group)

# Tuning Controls Group
tune_group = QGroupBox("Wrist Controller Tuning & Safety Limits")
tune_grid = QGridLayout()
tune_group.setLayout(tune_grid)

# Kp
lbl_kp = QLabel("<b>Kp (Proportional Gain):</b>")
spin_kp = QDoubleSpinBox()
spin_kp.setRange(0.0, 10.0)
spin_kp.setSingleStep(0.1)
spin_kp.setDecimals(2)
spin_kp.setValue(DEFAULT_KP)
spin_kp.setFixedWidth(80)
spin_kp.setToolTip("Proportional gain: increases stiffness and reduces steady-state error")
tune_grid.addWidget(lbl_kp, 0, 0)
tune_grid.addWidget(spin_kp, 0, 1)

# Kd
lbl_kd = QLabel("<b>Kd (Derivative Gain):</b>")
spin_kd = QDoubleSpinBox()
spin_kd.setRange(0.0, 1.0)
spin_kd.setSingleStep(0.01)
spin_kd.setDecimals(3)
spin_kd.setValue(DEFAULT_KD)
spin_kd.setFixedWidth(80)
spin_kd.setToolTip("Derivative damping gain: removes overshoot and oscillations")
tune_grid.addWidget(lbl_kd, 1, 0)
tune_grid.addWidget(spin_kd, 1, 1)

# PWM Saturation Limit
lbl_sat = QLabel(
    "<b>PWM Saturation Limit:</b><br>"
    "<small style='color: #888;'>Unit: <b>duty cycle</b> (|u| ≤ sat) &bull; Goal: Cap speed ≤ 90°/s or 1.57rad/s</small>"
)
spin_sat = QDoubleSpinBox()
spin_sat.setRange(0.05, 1.00) # Full PWM duty cycle range
spin_sat.setSingleStep(0.05)
spin_sat.setDecimals(2)
spin_sat.setValue(DEFAULT_SATURATION)
spin_sat.setFixedWidth(80)
spin_sat.setToolTip("Clamps maximum motor voltage. Adjust downwards until peak speed on Scope stays ≤ 90°/s")
tune_grid.addWidget(lbl_sat, 2, 0)
tune_grid.addWidget(spin_sat, 2, 1)

# PWM Rate Limiter (Slew Rate)
lbl_slew = QLabel(
    "<b>PWM Slew Rate Limit:</b><br>"
    "<small style='color: #888;'>Unit: <b>duty cycle / sec</b> (|Δu/Δt| ≤ slew) &bull; Goal: Cap accel ≤ 60°/s² or 1.04rad/s²</small>"
)
spin_slew = QDoubleSpinBox()
spin_slew.setRange(0.2, 50.0) # Allows testing fast step transitions (near-unlimited) down to smooth ramps
spin_slew.setSingleStep(0.5)
spin_slew.setDecimals(2)
spin_slew.setValue(DEFAULT_SLEW_RATE)
spin_slew.setToolTip("Limits how fast PWM ramps up/down per second. Adjust downwards until peak acceleration stays ≤ 60°/s²")
tune_grid.addWidget(lbl_slew, 3, 0)
tune_grid.addWidget(spin_slew, 3, 1)

main_layout.addWidget(tune_group)

# Preset Buttons Row
preset_layout = QHBoxLayout()
btn_initial = QPushButton("Initial (Kp=1.0, Kd=0.05)")
btn_tuned = QPushButton("Roughly Tuned Example (Kp=2.2, Kd=0.12)")

def set_presets(kp, kd, sat, slew):
    spin_kp.setValue(kp)
    spin_kd.setValue(kd)
    spin_sat.setValue(sat)
    spin_slew.setValue(slew)

btn_initial.clicked.connect(lambda: set_presets(1.0, 0.05, 0.40, 2.0))
btn_tuned.clicked.connect(lambda: set_presets(2.2, 0.12, 0.35, 1.8))

preset_layout.addWidget(btn_initial)
preset_layout.addWidget(btn_tuned)
main_layout.addLayout(preset_layout)

# Emergency Stop / Zero PWM Button
btn_stop = QPushButton("Emergency Stop / Disable Controller")
def emergency_stop():
    chk_enable_controller.setChecked(False)
    chk_enable_cmd.setChecked(False)
btn_stop.clicked.connect(emergency_stop)
main_layout.addWidget(btn_stop)

control_window.show()

# 5. Periodic Control Loop via QTimer (30 Hz)
start_time = time.time()

def control_step():
    global commanded_wrist_pos, raw_wrist_pwm, applied_wrist_pwm, last_applied_pwm
    global current_slew_rate, active_slew_limit, last_control_time

    now = time.time()
    dt = max(now - last_control_time, 1e-4)
    last_control_time = now
    current_time = now - start_time

    # Determine commanded wrist position
    if chk_enable_cmd.isChecked():
        phase = (current_time % PULSE_PERIOD) / PULSE_PERIOD
        sign = 1.0 if phase < 0.5 else -1.0
        commanded_wrist_pos = sign * PULSE_AMPLITUDE
    else:
        commanded_wrist_pos = 0.0

    # Read latest measurements
    meas_phi = np.array(myArm.measJointPosition)[:4]
    meas_phidot = np.array(myArm.measJointSpeed)[:4]

    wrist_meas = meas_phi[3]
    wrist_speed = meas_phidot[3]
    error = commanded_wrist_pos - wrist_meas

    if chk_enable_controller.isChecked():
        kp = spin_kp.value()
        kd = spin_kd.value()
        sat_limit = spin_sat.value()
        slew_limit = spin_slew.value()

        # Step 1: Raw PD Controller (derivative on measurement)
        u_raw = (kp * error) - (kd * wrist_speed)

        # Step 2: PWM Saturation (Limits max joint velocity)
        u_sat = np.clip(u_raw, -sat_limit, sat_limit)
        raw_wrist_pwm = float(u_sat)

        # Step 3: PWM Rate Limiter (Limits max joint acceleration)
        max_delta = slew_limit * dt
        u_rate_limited = np.clip(u_sat, last_applied_pwm - max_delta, last_applied_pwm + max_delta)

        # Record actual slew rate and active limit for Axis 5 plotting
        current_slew_rate = float((u_rate_limited - last_applied_pwm) / dt)
        active_slew_limit = float(slew_limit)

        # CATASTROPHIC SAFETY PROTECTION ONLY:
        # Prevent extreme over-rotation beyond ±1.6 rad (~92°) to protect internal cabling
        if wrist_meas > 1.6 and u_rate_limited > 0:
            u_rate_limited = 0.0
        elif wrist_meas < -1.6 and u_rate_limited < 0:
            u_rate_limited = 0.0

        applied_wrist_pwm = float(np.clip(u_rate_limited, -1.0, 1.0))
        last_applied_pwm = applied_wrist_pwm
    else:
        raw_wrist_pwm = 0.0
        applied_wrist_pwm = 0.0
        current_slew_rate = 0.0
        last_applied_pwm = 0.0

    # Build hybrid commands:
    # Joints 0-2 hold ELEVATED_POSE in Position Mode, Joint 3 driven by applied_wrist_pwm
    pwm_cmd = [0.0, 0.0, 0.0, applied_wrist_pwm]
    phi_cmd = ELEVATED_POSE.tolist()

    myArm.read_write_pwm(
        pwmCMD=pwm_cmd,
        phiCMD=phi_cmd,
        gprCMD=0.0,
        baseLED=desired_LED_color
    )

    # Refresh MultiScope plots
    MultiScope.refreshAll()

# Clean shutdown handler
def close_event(event):
    global KILL_THREAD
    KILL_THREAD = True
    myArm.read_write_pwm([0.0, 0.0, 0.0, 0.0], phiCMD=[0.0, 0.0, 0.0, 0.0], baseLED=[1, 0, 0])
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
myArm.read_write_pwm([0.0, 0.0, 0.0, 0.0], phiCMD=[0.0, 0.0, 0.0, 0.0], baseLED=[1, 0, 0])
myArm.terminate()
print("\n[Hardware Part 2] Shut down gracefully.")
# endregion
