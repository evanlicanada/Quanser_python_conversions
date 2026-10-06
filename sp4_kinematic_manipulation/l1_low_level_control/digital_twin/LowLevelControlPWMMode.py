# TEAM: QArm Low-Level Control (PWM Mode) - Digital Twin
# TEAM: Interactive engineering laboratory for designing, tuning, and testing closed-loop joint controllers.
# Students implement and tune proportional-derivative (PD) and PID gains, observe saturation non-linearities,
# evaluate gravitational cross-coupling effects across three physical scenarios, and compare tracking performance
# against the Position Mode baseline.

# region: Python level imports
import os
import sys
import time
import signal
from threading import Thread
import numpy as np

from PyQt6.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QLabel, QPushButton, QSlider, QRadioButton, QButtonGroup, QGroupBox,
    QDoubleSpinBox, QComboBox, QCheckBox, QTabWidget
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
phiLim = [-3.5, 3.5]          # Angular position limits (rad)
phiDotLim = [-3.0, 3.0]       # Angular speed limits (rad/s)
errorLim = [-2.0, 2.0]        # Position error limits (rad)
pwmLim = [-1.05, 1.05]        # PWM duty cycle limits (-1.0 to 1.0 showing saturation)
currentLim = [-2.0, 2.0]      # Motor current limits (A)
tempLim = [15.0, 45.0]        # Motor temperature limits (°C)
# endregion


# region: Scope setup or Probe/Observer setup (pick one)
# TEAM: Setup any scopes or probes/observers needed for your experiment here. Leave blank if not using the scope.
multiScope = MultiScope(
    rows=2,
    cols=3,
    title='QArm Low-Level Control: PWM Closed-Loop PID Tuning & Dynamics',
    fps=scopeFps,
    bufferSize=scopeBufferSize
)

# Axis 0: Joint Positions (Commanded vs Measured)
multiScope.addAxis(row=0, col=0, timeWindow=scopeTimeWindow, yLabel='Phi (rad)', yLim=phiLim)
multiScope.axes[0].attachSignal(name='cmdPhi')
multiScope.axes[0].attachSignal(name='measPhi')

# Axis 1: Tracking Error (Cmd - Meas)
multiScope.addAxis(row=0, col=1, timeWindow=scopeTimeWindow, yLabel='Error (rad)', yLim=errorLim)
multiScope.axes[1].attachSignal(name='baseError')
multiScope.axes[1].attachSignal(name='shoulderError')
multiScope.axes[1].attachSignal(name='elbowError')
multiScope.axes[1].attachSignal(name='wristError')

# Axis 2: Joint Speeds
multiScope.addAxis(row=0, col=2, timeWindow=scopeTimeWindow, yLabel='Speed (rad/s)', yLim=phiDotLim)
multiScope.axes[2].attachSignal(name='baseSpeed')
multiScope.axes[2].attachSignal(name='shoulderSpeed')
multiScope.axes[2].attachSignal(name='elbowSpeed')
multiScope.axes[2].attachSignal(name='wristSpeed')

# Axis 3: Commanded PWM (Visualizes ±1.0 Saturation Clamping)
multiScope.addAxis(row=1, col=0, timeWindow=scopeTimeWindow, yLabel='PWM Duty Cycle', yLim=pwmLim)
multiScope.axes[3].attachSignal(name='basePWM')
multiScope.axes[3].attachSignal(name='shoulderPWM')
multiScope.axes[3].attachSignal(name='elbowPWM')
multiScope.axes[3].attachSignal(name='wristPWM')

# Axis 4: Motor Currents
multiScope.addAxis(row=1, col=1, timeWindow=scopeTimeWindow, yLabel='Current (A)', yLim=currentLim)
multiScope.axes[4].attachSignal(name='baseCurrent')
multiScope.axes[4].attachSignal(name='shoulderCurrent')
multiScope.axes[4].attachSignal(name='elbowCurrent')
multiScope.axes[4].attachSignal(name='wristCurrent')

# Axis 5: Motor Temperatures
multiScope.addAxis(row=1, col=2, timeWindow=scopeTimeWindow, yLabel='Temperature (°C)', yLim=tempLim)
multiScope.axes[5].attachSignal(name='baseTemp')
multiScope.axes[5].attachSignal(name='shoulderTemp')
multiScope.axes[5].attachSignal(name='elbowTemp')
multiScope.axes[5].attachSignal(name='wristTemp')
# endregion


# region: Experiment constants
# TEAM: Define any constants needed for your experiment here. This is ideally where students change values and see the result based on the lab

hardwareMode = 0            # 0 for Virtual QArm (Digital Twin), 1 for Physical Hardware
readMode = 0                # 0 for Immediate I/O, 1 for Task-based I/O
jointMode = 1               # 1 for PWM Mode (Raw duty cycle commands)
scopeSampleInterval = 0.02  # Sampling rate for scope data collection (50 Hz)
defaultLedColor = [0, 1, 0] # Base LED color (Green)

# Initial controller gains matching Lab Procedure Step 5 (Starting baseline with high steady-state error)
INITIAL_KP = np.array([0.1, 0.1, 0.1, 0.1], dtype=np.float64)
INITIAL_KD = np.array([0.0, 0.0, 0.0, 0.0], dtype=np.float64)
INITIAL_KI = np.array([0.0, 0.0, 0.0, 0.0], dtype=np.float64)

# Well-tuned reference gains for student comparison
TUNED_KP = np.array([2.5, 2.5, 2.0, 1.5], dtype=np.float64)
TUNED_KD = np.array([0.12, 0.15, 0.12, 0.06], dtype=np.float64)
TUNED_KI = np.array([0.0, 0.0, 0.0, 0.0], dtype=np.float64)

# Navigation coordinates (Point A and Point B)
POINT_A = np.array([0.7854, 0.6002, 0.2059, 0.0])  # [Base=45°, Shoulder=34.4°, Elbow=11.8°, Wrist=0°]
POINT_B = np.array([2.3562, 0.6002, 0.2059, 0.0])  # [Base=135°, Shoulder=34.4°, Elbow=11.8°, Wrist=0°]

# Pulse benchmark parameters
PULSE_AMPLITUDE = np.pi / 4.0   # ±45 degrees
PULSE_PERIOD = 6.0              # Seconds per full cycle
# endregion


# region: Main Loop
# TEAM: Implement the main code for your experiment here. Use comments to guide students through the steps needed to complete the lab.

# 1. Kill switch setup for clean exit on Ctrl+C
KILL_THREAD = False
def sig_handler(*args):
    global KILL_THREAD
    KILL_THREAD = True
signal.signal(signal.SIGINT, sig_handler)

# 2. Hardware / Digital Twin Initialization in PWM Mode
myArm = QArm(hardware=hardwareMode, readMode=readMode, jointMode=jointMode)
commanded_joint_pos = np.array([0.0, 0.0, 0.0, 0.0], dtype=np.float64)
applied_joint_pwm = np.array([0.0, 0.0, 0.0, 0.0], dtype=np.float64)
integral_error = np.array([0.0, 0.0, 0.0, 0.0], dtype=np.float64)
last_control_time = time.time()
active_plot_joint = 0  # 0=Base, 1=Shoulder, 2=Elbow, 3=Wrist for Axis 0 display

desired_LED_color = list(defaultLedColor)
myArm.read_write_pwm([0.0, 0.0, 0.0, 0.0], baseLED=desired_LED_color)

# 3. Background Data Collection Thread
def scope_loop():
    """Samples hardware values and pushes them to the MultiScope plots."""
    t0 = time.time()

    while not KILL_THREAD:
        t = time.time() - t0
        meas_phi = np.array(myArm.measJointPosition)[:4]
        meas_phidot = np.array(myArm.measJointSpeed)[:4]
        meas_current = np.array(myArm.measJointCurrent)[:4]
        meas_temp = np.array(myArm.measJointTemperature)[:4]
        err = commanded_joint_pos - meas_phi

        # Axis 0: Commanded vs Measured for the currently selected active joint
        j = active_plot_joint
        multiScope.axes[0].sample(t, [commanded_joint_pos[j], meas_phi[j]])
        # Axis 1: Tracking error for all 4 joints
        multiScope.axes[1].sample(t, err)
        # Axis 2: Measured joint speeds
        multiScope.axes[2].sample(t, meas_phidot)
        # Axis 3: Applied PWM commands (shows saturation)
        multiScope.axes[3].sample(t, applied_joint_pwm)
        # Axis 4: Motor currents
        multiScope.axes[4].sample(t, meas_current)
        # Axis 5: Motor temperatures
        multiScope.axes[5].sample(t, meas_temp)

        time.sleep(scopeSampleInterval)

thread = Thread(target=scope_loop, daemon=True)
thread.start()

# 4. PyQt6 Custom Control Panel
app = QApplication.instance()
if app is None:
    app = QApplication([])

control_window = QWidget()
control_window.setWindowTitle("QArm Control Panel (PWM Closed-Loop Mode)")
control_window.resize(520, 580)

main_layout = QVBoxLayout()
control_window.setLayout(main_layout)

title_label = QLabel(
    "<b>QArm Closed-Loop Low-Level Control (PWM Mode)</b><br>"
    "<small>Tune PID gains, observe saturation, and evaluate gravitational non-linearities</small>"
)
title_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
main_layout.addWidget(title_label)

# --- 1. Operating Mode Selection Group ---
mode_group = QGroupBox("Operating Mode & Scenarios")
mode_layout = QVBoxLayout()
mode_group.setLayout(mode_layout)

rb_home = QRadioButton("Home Position [0, 0, 0, 0] rad")
rb_exploration = QRadioButton("Exploration Pulse (±45° square wave on selected joint)")
rb_scenario_1 = QRadioButton("Gravity Scenario 1: Vertical Symmetrical (Shoulder ±30°, Elbow -90°)")
rb_scenario_2 = QRadioButton("Gravity Scenario 2: Asymmetric Offset (Shoulder +15° to +75°, Elbow -90°)")
rb_scenario_3 = QRadioButton("Gravity Scenario 3: Bent Arm CoM Shift (Shoulder ±30°, Elbow 0°)")
rb_point_ab = QRadioButton("Navigation: Point A ↔ Point B (Base 45° ↔ 135°)")

rb_home.setChecked(True)

btn_mode_group = QButtonGroup()
btn_mode_group.addButton(rb_home, 0)
btn_mode_group.addButton(rb_exploration, 1)
btn_mode_group.addButton(rb_scenario_1, 2)
btn_mode_group.addButton(rb_scenario_2, 3)
btn_mode_group.addButton(rb_scenario_3, 4)
btn_mode_group.addButton(rb_point_ab, 5)

mode_layout.addWidget(rb_home)
mode_layout.addWidget(rb_exploration)

# Sub-row for Exploration Pulse Active Joint
pulse_joint_layout = QHBoxLayout()
pulse_joint_layout.setContentsMargins(20, 0, 0, 0)
pulse_joint_layout.addWidget(QLabel("Active Joint:"))

rb_exp_joints = [
    QRadioButton("Base"),
    QRadioButton("Shoulder"),
    QRadioButton("Elbow"),
    QRadioButton("Wrist")
]
rb_exp_joints[0].setChecked(True)
grp_exp = QButtonGroup()
for idx, rb in enumerate(rb_exp_joints):
    grp_exp.addButton(rb, idx)
    pulse_joint_layout.addWidget(rb)
pulse_joint_layout.addStretch()

mode_layout.addLayout(pulse_joint_layout)
mode_layout.addWidget(rb_scenario_1)
mode_layout.addWidget(rb_scenario_2)
mode_layout.addWidget(rb_scenario_3)
mode_layout.addWidget(rb_point_ab)
main_layout.addWidget(mode_group)

# --- 2. PID Gains Tuning Group ---
gain_group = QGroupBox("Joint PID Gains")
gain_layout = QVBoxLayout()
gain_group.setLayout(gain_layout)

gain_grid = QGridLayout()
gain_grid.addWidget(QLabel("<b>Joint</b>"), 0, 0)
gain_grid.addWidget(QLabel("<b>Kp</b>"), 0, 1)
gain_grid.addWidget(QLabel("<b>Kd</b>"), 0, 2)
gain_grid.addWidget(QLabel("<b>Ki</b>"), 0, 3)

joint_names = ["Base:", "Shoulder:", "Elbow:", "Wrist:"]
spin_kp = []
spin_kd = []
spin_ki = []

for i, name in enumerate(joint_names):
    lbl = QLabel(name)
    gain_grid.addWidget(lbl, i + 1, 0)

    skp = QDoubleSpinBox()
    skp.setRange(0.0, 50.0)
    skp.setSingleStep(0.1)
    skp.setDecimals(2)
    skp.setValue(INITIAL_KP[i])
    skp.setFixedWidth(75)
    skp.setAlignment(Qt.AlignmentFlag.AlignCenter)
    gain_grid.addWidget(skp, i + 1, 1)
    spin_kp.append(skp)

    skd = QDoubleSpinBox()
    skd.setRange(0.0, 5.0)
    skd.setSingleStep(0.02)
    skd.setDecimals(3)
    skd.setValue(INITIAL_KD[i])
    skd.setFixedWidth(75)
    skd.setAlignment(Qt.AlignmentFlag.AlignCenter)
    gain_grid.addWidget(skd, i + 1, 2)
    spin_kd.append(skd)

    ski = QDoubleSpinBox()
    ski.setRange(0.0, 5.0)
    ski.setSingleStep(0.01)
    ski.setDecimals(3)
    ski.setValue(INITIAL_KI[i])
    ski.setFixedWidth(75)
    ski.setAlignment(Qt.AlignmentFlag.AlignCenter)
    gain_grid.addWidget(ski, i + 1, 3)
    spin_ki.append(ski)

gain_layout.addLayout(gain_grid)

# Preset Buttons Row
preset_layout = QHBoxLayout()
btn_reset_initial = QPushButton("Reset Initial (Kp=0.1, Kd=0)")
btn_apply_tuned = QPushButton("Apply (Roughly) Tuned Baseline")
btn_zero_gains = QPushButton("Zero All Gains")

def set_all_gains(kp_vals, kd_vals, ki_vals):
    for i in range(4):
        spin_kp[i].setValue(kp_vals[i])
        spin_kd[i].setValue(kd_vals[i])
        spin_ki[i].setValue(ki_vals[i])

btn_reset_initial.clicked.connect(lambda: set_all_gains(INITIAL_KP, INITIAL_KD, INITIAL_KI))
btn_apply_tuned.clicked.connect(lambda: set_all_gains(TUNED_KP, TUNED_KD, TUNED_KI))
btn_zero_gains.clicked.connect(lambda: set_all_gains([0]*4, [0]*4, [0]*4))

preset_layout.addWidget(btn_reset_initial)
preset_layout.addWidget(btn_apply_tuned)
preset_layout.addWidget(btn_zero_gains)
gain_layout.addLayout(preset_layout)

main_layout.addWidget(gain_group)

# --- 3. Scope Focus Joint Row ---
scope_focus_layout = QHBoxLayout()
scope_focus_layout.addWidget(QLabel("<b>MultiScope Axis 0 Focus:</b>"))
combo_active_joint = QComboBox()
combo_active_joint.addItems(["Base (Joint 0)", "Shoulder (Joint 1)", "Elbow (Joint 2)", "Wrist (Joint 3)"])

def on_active_joint_change(idx):
    global active_plot_joint
    active_plot_joint = idx

combo_active_joint.currentIndexChanged.connect(on_active_joint_change)
scope_focus_layout.addWidget(combo_active_joint)
scope_focus_layout.addStretch()
main_layout.addLayout(scope_focus_layout)

def on_mode_toggled(mode_id):
    # Enable/disable active joint radio buttons for exploration mode
    is_exp = (mode_id == 1)
    for rb in rb_exp_joints:
        rb.setEnabled(is_exp)
    if is_exp:
        combo_active_joint.setCurrentIndex(grp_exp.checkedId())
    elif mode_id in [2, 3, 4]:
        combo_active_joint.setCurrentIndex(1) # Shoulder
    elif mode_id == 5:
        combo_active_joint.setCurrentIndex(0) # Base

btn_mode_group.idClicked.connect(on_mode_toggled)
grp_exp.idClicked.connect(lambda idx: combo_active_joint.setCurrentIndex(idx))

# Initial state for exploration pulse joint toggles
on_mode_toggled(0)

# --- 4. Emergency Stop / Reset to Home Button ---
btn_stop = QPushButton("Emergency Stop / Reset to Home")
def emergency_stop():
    rb_home.setChecked(True)
    on_mode_toggled(0)
    for i in range(4):
        integral_error[i] = 0.0
btn_stop.clicked.connect(emergency_stop)
main_layout.addWidget(btn_stop)

control_window.show()

# 5. Periodic Control Loop via QTimer (30 Hz)
start_time = time.time()
last_time = time.time()

def control_step():
    global commanded_joint_pos, applied_joint_pwm, integral_error, last_time

    now = time.time()
    dt = max(now - last_time, 1e-4)
    last_time = now
    current_time = now - start_time

    mode_id = btn_mode_group.checkedId()
    target = np.array([0.0, 0.0, 0.0, 0.0], dtype=np.float64)

    if mode_id == 0:
        # Home Position
        target = np.array([0.0, 0.0, 0.0, 0.0])

    elif mode_id == 1:
        # Exploration Mode: ±45 deg square wave on selected joint
        active_joint = grp_exp.checkedId()
        phase = (current_time % PULSE_PERIOD) / PULSE_PERIOD
        sign = 1.0 if phase < 0.5 else -1.0
        target = np.array([0.0, 0.0, 0.0, 0.0])
        target[active_joint] = sign * PULSE_AMPLITUDE

    elif mode_id == 2:
        # Scenario 1 (Table 1): Outstretched vertically, Shoulder ±30° (±π/6), Elbow -90° (-π/2)
        period = 10.0
        phase = (current_time % period) / period
        sign = 1.0 if phase < 0.5 else -1.0
        sh_amp = np.pi / 6.0
        target = np.array([0.0, sign * sh_amp, -np.pi / 2.0, 0.0])

    elif mode_id == 3:
        # Scenario 2 (Table 1): Asymmetric Offset, Shoulder +15° to +75° (Bias = π/6, Gain = 0.5*π/4)
        period = 10.0
        phase = (current_time % period) / period
        sign = 1.0 if phase < 0.5 else -1.0
        bias = np.pi / 6.0
        amp = 0.5 * (np.pi / 4.0)
        target = np.array([0.0, bias + sign * amp, -np.pi / 2.0, 0.0])

    elif mode_id == 4:
        # Scenario 3 (Table 1): Bent Arm (Elbow = 0°), Shoulder ±30°
        period = 10.0
        phase = (current_time % period) / period
        sign = 1.0 if phase < 0.5 else -1.0
        sh_amp = 0.75 * (np.pi / 4.0) # ~33.75° or ±30°
        target = np.array([0.0, sign * sh_amp, 0.0, 0.0])

    elif mode_id == 5:
        # Point A <-> Point B Navigation (8s period)
        nav_period = 8.0
        phase = (current_time % nav_period) / nav_period
        target = POINT_A if phase < 0.5 else POINT_B

    commanded_joint_pos = target

    # Read current state
    meas_phi = np.array(myArm.measJointPosition)[:4]
    meas_phidot = np.array(myArm.measJointSpeed)[:4]

    # Compute PID Control Law
    # Error: e(t) = cmd - meas
    error = commanded_joint_pos - meas_phi

    # Read current gains from GUI spinboxes
    kp = np.array([s.value() for s in spin_kp])
    kd = np.array([s.value() for s in spin_kd])
    ki = np.array([s.value() for s in spin_ki])

    # Integral accumulation with anti-windup clamping
    integral_error += error * dt
    integral_error = np.clip(integral_error, -1.0, 1.0)

    # Compute raw PWM: Derivative on measurement (-phidot) prevents derivative kick on setpoint step
    raw_pwm = (kp * error) - (kd * meas_phidot) + (ki * integral_error)

    # Actuator Saturation: PWM duty cycle clamped between -1.0 and 1.0
    applied_joint_pwm = np.clip(raw_pwm, -1.0, 1.0)

    # Send PWM commands to QArm hardware/digital twin and read sensor values
    myArm.read_write_pwm(
        pwmCMD=applied_joint_pwm.tolist(),
        gprCMD=0.0,
        baseLED=desired_LED_color
    )

    # Refresh MultiScope plots
    MultiScope.refreshAll()

# Clean shutdown when window is closed
def close_event(event):
    global KILL_THREAD
    KILL_THREAD = True
    myArm.read_write_pwm([0.0, 0.0, 0.0, 0.0], baseLED=[1, 0, 0])
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
myArm.read_write_pwm([0.0, 0.0, 0.0, 0.0], baseLED=[1, 0, 0])
myArm.terminate()
print("\n[PWM Mode] Shut down gracefully.")
# endregion
