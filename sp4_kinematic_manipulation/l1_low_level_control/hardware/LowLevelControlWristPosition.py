# TEAM: QArm Low-Level Control - Part 1: Wrist Position Mode (Hardware)
# TEAM: Physical hardware baseline benchmark. Elevates the manipulator above the base to a safe inspection pose,
# pulses the wrist joint ±45 degrees in factory closed-loop Position Mode, and records baseline position tracking,
# angular velocity, and acceleration on a real-time MultiScope.

# region: Python level imports
import os
import sys
import time
import signal
from threading import Thread
import numpy as np

from PyQt6.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QPushButton, QRadioButton, QButtonGroup, QGroupBox
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
wristPosLim = [-1.5, 1.5]         # Wrist position limits (rad) [approx ±85°]
wristSpeedLim = [-3.0, 3.0]       # Wrist angular speed limits (rad/s) [approx ±170°/s]
wristAccelLim = [-15.0, 15.0]     # Wrist angular acceleration limits (rad/s^2)
currentLim = [-1.5, 1.5]          # Motor current limits (A)
pwmLim = [-1.05, 1.05]            # Internal PWM duty cycle limits
tempLim = [15.0, 50.0]            # Motor temperature limits (°C)
# endregion


# region: Scope setup or Probe/Observer setup (pick one)
# TEAM: Setup any scopes or probes/observers needed for your experiment here. Leave blank if not using the scope.
multiScope = MultiScope(
    rows=2,
    cols=3,
    title='QArm Hardware Low-Level Control - Part 1: Wrist Position Diagnostic',
    fps=scopeFps,
    bufferSize=scopeBufferSize
)

# Axis 0: Wrist Position (Commanded vs Measured)
multiScope.addAxis(row=0, col=0, timeWindow=scopeTimeWindow, yLabel='Wrist Pos (rad)', yLim=wristPosLim)
multiScope.axes[0].attachSignal(name='wristCmd')
multiScope.axes[0].attachSignal(name='wristMeas')

# Axis 1: Wrist Angular Speed
multiScope.addAxis(row=0, col=1, timeWindow=scopeTimeWindow, yLabel='Wrist Speed (rad/s)', yLim=wristSpeedLim)
multiScope.axes[1].attachSignal(name='wristSpeed')

# Axis 2: Wrist Angular Acceleration
multiScope.addAxis(row=0, col=2, timeWindow=scopeTimeWindow, yLabel='Wrist Accel (rad/s²)', yLim=wristAccelLim)
multiScope.axes[2].attachSignal(name='wristAccel')

# Axis 3: Joint Motor Currents
multiScope.addAxis(row=1, col=0, timeWindow=scopeTimeWindow, yLabel='Wrist Current (A)', yLim=currentLim)
multiScope.axes[3].attachSignal(name='wristCurrent')

# Axis 4: Internal Joint PWM Feedback
multiScope.addAxis(row=1, col=1, timeWindow=scopeTimeWindow, yLabel='PWM Duty Cycle', yLim=pwmLim)
multiScope.axes[4].attachSignal(name='wristPWM')

# Axis 5: Motor Temperatures
multiScope.addAxis(row=1, col=2, timeWindow=scopeTimeWindow, yLabel='Wrist Temp (°C)', yLim=tempLim)
multiScope.axes[5].attachSignal(name='wristTemp')
# endregion


# region: Experiment constants
# TEAM: Define any constants needed for your experiment here. This is ideally where students change values and see the result based on the lab

hardwareMode = 1            # 1 for Physical Hardware (USB)
readMode = 0                # 0 for Immediate I/O, 1 for Task-based I/O
jointMode = 0               # 0 for Position Mode (Factory Closed-Loop Firmware)
scopeSampleInterval = 0.02  # Sampling rate for scope data collection (50 Hz)
defaultLedColor = [0, 1, 0] # Base LED color (Green)

# Elevated test pose lifting arm above base away from table (from Figure 2 of hardware lab manual)
ELEVATED_POSE = np.array([0.0, -1.5, 1.2, 0.0], dtype=np.float64)
PULSE_AMPLITUDE = np.pi / 4.0   # ±45 degrees (±0.785 rad)
PULSE_PERIOD = 6.0              # Seconds per full square-wave cycle
# endregion


# region: Main Loop
# TEAM: Implement the main code for your experiment here. Use comments to guide students through the steps needed to complete the lab.

# 1. Kill switch setup for clean exit on Ctrl+C
KILL_THREAD = False
def sig_handler(*args):
    global KILL_THREAD
    KILL_THREAD = True
signal.signal(signal.SIGINT, sig_handler)

# 2. Hardware Initialization in Position Mode
myArm = QArm(hardware=hardwareMode, readMode=readMode, jointMode=jointMode)
commanded_joint_pos = np.array([0.0, 0.0, 0.0, 0.0], dtype=np.float64)
desired_LED_color = list(defaultLedColor)
myArm.read_write_std(commanded_joint_pos, baseLED=desired_LED_color)

# Variables for calculating filtered acceleration
last_speed = 0.0
last_speed_time = time.time()
meas_accel = 0.0

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
        meas_pwm = np.array(myArm.measJointPWM)[:4]
        meas_temp = np.array(myArm.measJointTemperature)[:4]

        # Numerical differentiation for acceleration with low-pass smoothing
        now = time.time()
        dt = max(now - last_speed_time, 1e-4)
        raw_accel = (meas_phidot[3] - last_speed) / dt
        meas_accel = 0.8 * meas_accel + 0.2 * raw_accel
        last_speed = meas_phidot[3]
        last_speed_time = now

        # Axis 0: Wrist Commanded vs Measured Position
        multiScope.axes[0].sample(t, [commanded_joint_pos[3], meas_phi[3]])
        # Axis 1: Wrist Angular Speed
        multiScope.axes[1].sample(t, meas_phidot[3])
        # Axis 2: Wrist Acceleration
        multiScope.axes[2].sample(t, meas_accel)
        # Axis 3: Wrist Motor Current
        multiScope.axes[3].sample(t, meas_current[3])
        # Axis 4: Wrist PWM Duty Cycle
        multiScope.axes[4].sample(t, meas_pwm[3])
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
control_window.setWindowTitle("QArm Hardware - Part 1: Wrist Position Mode")
control_window.resize(480, 360)

main_layout = QVBoxLayout()
control_window.setLayout(main_layout)

title_label = QLabel(
    "<b>QArm Low-Level Control - Part 1: Wrist Position Mode</b><br>"
    "<small>Record baseline tracking, velocity, and acceleration on physical hardware</small>"
)
title_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
main_layout.addWidget(title_label)

# Mode Selection Group Box
mode_group = QGroupBox("Operating Mode")
mode_layout = QVBoxLayout()
mode_group.setLayout(mode_layout)

rb_home = QRadioButton("0. Home Position [0, 0, 0, 0] rad")
rb_elevated = QRadioButton("1. Elevated Test Pose [0.0, -1.5, 1.2, 0.0] rad")
rb_pulse = QRadioButton("2. Pulse Wrist (±45° square wave around elevated pose)")

rb_home.setChecked(True)

btn_group = QButtonGroup()
btn_group.addButton(rb_home, 0)
btn_group.addButton(rb_elevated, 1)
btn_group.addButton(rb_pulse, 2)

mode_layout.addWidget(rb_home)
mode_layout.addWidget(rb_elevated)
mode_layout.addWidget(rb_pulse)
main_layout.addWidget(mode_group)

# Emergency Stop / Reset Button
btn_stop = QPushButton("Emergency Stop / Reset to Home")
def reset_home():
    rb_home.setChecked(True)
btn_stop.clicked.connect(reset_home)
main_layout.addWidget(btn_stop)

control_window.show()

# 5. Periodic Control Loop via QTimer (30 Hz)
start_time = time.time()

def control_step():
    global commanded_joint_pos
    current_time = time.time() - start_time
    mode_id = btn_group.checkedId()

    target = np.array([0.0, 0.0, 0.0, 0.0], dtype=np.float64)

    if mode_id == 0:
        # Home Position
        target = np.array([0.0, 0.0, 0.0, 0.0])
    elif mode_id == 1:
        # Elevated Test Pose
        target = np.array(ELEVATED_POSE)
    elif mode_id == 2:
        # Elevated Test Pose with Wrist Pulse (±45 deg)
        phase = (current_time % PULSE_PERIOD) / PULSE_PERIOD
        sign = 1.0 if phase < 0.5 else -1.0
        target = np.array([ELEVATED_POSE[0], ELEVATED_POSE[1], ELEVATED_POSE[2], sign * PULSE_AMPLITUDE])

    commanded_joint_pos = target

    # Send position command to QArm hardware
    myArm.read_write_std(
        phiCMD=commanded_joint_pos.tolist(),
        gprCMD=0.0,
        baseLED=desired_LED_color
    )

    # Refresh MultiScope plots
    MultiScope.refreshAll()

# Clean shutdown handler
def close_event(event):
    global KILL_THREAD
    KILL_THREAD = True
    myArm.read_write_std([0.0, 0.0, 0.0, 0.0], baseLED=[1, 0, 0])
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
myArm.read_write_std([0.0, 0.0, 0.0, 0.0], baseLED=[1, 0, 0])
myArm.terminate()
print("\n[Hardware Part 1] Shut down gracefully.")
# endregion
