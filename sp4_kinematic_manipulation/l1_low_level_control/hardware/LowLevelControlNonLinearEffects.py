# TEAM: QArm Low-Level Control - Part 4: Non-Linear Effects & Gravity (Hardware)
# TEAM: Physical hardware evaluation of gravitational cross-coupling and non-linear dynamics.
# Runs the physical manipulator in Position Mode safely across three gravitational scenarios (Vertical,
# Asymmetric Offset, and Bent Arm CoM offset) and Point A to Point B navigation, monitoring real motor current
# surges and overshoot caused by gravity assisting vs. opposing motion.

# region: Python level imports
import os
import sys
import time
import signal
from threading import Thread
import numpy as np

from PyQt6.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout,
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
scopeTimeWindow = 8.0

# Fixed Y-axis limits prevent misleading micro-scale auto-zooming around near-zero values
phiLim = [-3.5, 3.5]          # Angular position limits (rad)
phiDotLim = [-2.5, 2.5]       # Angular speed limits (rad/s)
currentLim = [-2.5, 2.5]      # Motor current limits (A) [shows gravitational torque spikes]
errorLim = [-1.0, 1.0]        # Tracking error limits (rad)
pwmLim = [-1.05, 1.05]        # Internal PWM duty cycle limits
tempLim = [15.0, 50.0]        # Motor temperature limits (°C)
# endregion


# region: Scope setup or Probe/Observer setup (pick one)
# TEAM: Setup any scopes or probes/observers needed for your experiment here. Leave blank if not using the scope.
multiScope = MultiScope(
    rows=2,
    cols=3,
    title='QArm Hardware Low-Level Control - Part 4: Gravity & Non-Linear Effects',
    fps=scopeFps,
    bufferSize=scopeBufferSize
)

# Axis 0: Joint Positions (Commanded vs Measured for Shoulder & Elbow)
multiScope.addAxis(row=0, col=0, timeWindow=scopeTimeWindow, yLabel='Phi (rad)', yLim=phiLim)
multiScope.axes[0].attachSignal(name='shoulderCmd')
multiScope.axes[0].attachSignal(name='shoulderMeas')
multiScope.axes[0].attachSignal(name='elbowCmd')
multiScope.axes[0].attachSignal(name='elbowMeas')

# Axis 1: Joint Position Tracking Error (Cmd - Meas)
multiScope.addAxis(row=0, col=1, timeWindow=scopeTimeWindow, yLabel='Error (rad)', yLim=errorLim)
multiScope.axes[1].attachSignal(name='baseError')
multiScope.axes[1].attachSignal(name='shoulderError')
multiScope.axes[1].attachSignal(name='elbowError')
multiScope.axes[1].attachSignal(name='wristError')

# Axis 2: Joint Angular Speeds
multiScope.addAxis(row=0, col=2, timeWindow=scopeTimeWindow, yLabel='Speed (rad/s)', yLim=phiDotLim)
multiScope.axes[2].attachSignal(name='shoulderSpeed')
multiScope.axes[2].attachSignal(name='elbowSpeed')

# Axis 3: Joint Motor Currents (Crucial for observing physical gravitational torques!)
multiScope.addAxis(row=1, col=0, timeWindow=scopeTimeWindow, yLabel='Current (A)', yLim=currentLim)
multiScope.axes[3].attachSignal(name='baseCurrent')
multiScope.axes[3].attachSignal(name='shoulderCurrent')
multiScope.axes[3].attachSignal(name='elbowCurrent')
multiScope.axes[3].attachSignal(name='wristCurrent')

# Axis 4: PWM Duty Cycle
multiScope.addAxis(row=1, col=1, timeWindow=scopeTimeWindow, yLabel='PWM Duty Cycle', yLim=pwmLim)
multiScope.axes[4].attachSignal(name='shoulderPWM')
multiScope.axes[4].attachSignal(name='elbowPWM')

# Axis 5: Motor Temperature
multiScope.addAxis(row=1, col=2, timeWindow=scopeTimeWindow, yLabel='Temp (°C)', yLim=tempLim)
multiScope.axes[5].attachSignal(name='shoulderTemp')
multiScope.axes[5].attachSignal(name='elbowTemp')
# endregion


# region: Experiment constants
# TEAM: Define any constants needed for your experiment here. This is ideally where students change values and see the result based on the lab

hardwareMode = 1            # 1 for Physical Hardware
readMode = 0                # 0 for Immediate I/O, 1 for Task-based I/O
jointMode = 0               # 0 for Position Mode (Safe physical trajectory execution)
scopeSampleInterval = 0.02  # Sampling rate for scope data collection (50 Hz)
defaultLedColor = [0, 1, 0] # Base LED color (Green)

# Navigation coordinates (Point A and Point B)
POINT_A = np.array([0.7854, 0.6002, 0.2059, 0.0])  # [Base=45°, Shoulder=34.4°, Elbow=11.8°, Wrist=0°]
POINT_B = np.array([2.3562, 0.6002, 0.2059, 0.0])  # [Base=135°, Shoulder=34.4°, Elbow=11.8°, Wrist=0°]
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
desired_gripper_cmd = 0.0
desired_LED_color = list(defaultLedColor)
myArm.read_write_std(commanded_joint_pos, baseLED=desired_LED_color)

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

        err = commanded_joint_pos - meas_phi

        # Axis 0: Commanded vs Measured for Shoulder & Elbow
        multiScope.axes[0].sample(t, [commanded_joint_pos[1], meas_phi[1], commanded_joint_pos[2], meas_phi[2]])
        # Axis 1: Tracking error for all 4 joints
        multiScope.axes[1].sample(t, err)
        # Axis 2: Measured joint speeds for Shoulder & Elbow
        multiScope.axes[2].sample(t, [meas_phidot[1], meas_phidot[2]])
        # Axis 3: Joint currents for all 4 joints
        multiScope.axes[3].sample(t, meas_current)
        # Axis 4: PWM Duty Cycle for Shoulder & Elbow
        multiScope.axes[4].sample(t, [meas_pwm[1], meas_pwm[2]])
        # Axis 5: Motor temperatures
        multiScope.axes[5].sample(t, [meas_temp[1], meas_temp[2]])

        time.sleep(scopeSampleInterval)

thread = Thread(target=scope_loop, daemon=True)
thread.start()

# 4. PyQt6 Custom Control Panel
app = QApplication.instance()
if app is None:
    app = QApplication([])

control_window = QWidget()
control_window.setWindowTitle("QArm Hardware - Part 4: Gravity & Non-Linear Effects")
control_window.resize(520, 520)

main_layout = QVBoxLayout()
control_window.setLayout(main_layout)

title_label = QLabel(
    "<b>QArm Low-Level Control - Part 4: Non-Linear Effects</b><br>"
    "<small>Evaluate physical gravitational torque, cross-coupling, and current surges on hardware</small>"
)
title_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
main_layout.addWidget(title_label)

# Mode Selection Group Box
mode_group = QGroupBox("Operating Mode & Scenarios")
mode_layout = QVBoxLayout()
mode_group.setLayout(mode_layout)

rb_home = QRadioButton("0. Home Position [0, 0, 0, 0] rad")
rb_scen1 = QRadioButton("1. Scenario 1: Arm Outstretched Vertically (Shoulder ±30°, Elbow -90°)")
rb_scen2 = QRadioButton("2. Scenario 2: Asymmetric Offset (Shoulder +15° to +75°, Elbow -90°)")
rb_scen3 = QRadioButton("3. Scenario 3: Bent Arm CoM Shift (Shoulder ±30°, Elbow 0°)")
rb_nav = QRadioButton("4. Navigation: Point A ↔ Point B (Base 45° ↔ 135°)")

rb_home.setChecked(True)

grp_modes = QButtonGroup()
grp_modes.addButton(rb_home, 0)
grp_modes.addButton(rb_scen1, 1)
grp_modes.addButton(rb_scen2, 2)
grp_modes.addButton(rb_scen3, 3)
grp_modes.addButton(rb_nav, 4)

mode_layout.addWidget(rb_home)
mode_layout.addWidget(rb_scen1)
mode_layout.addWidget(rb_scen2)
mode_layout.addWidget(rb_scen3)
mode_layout.addWidget(rb_nav)
main_layout.addWidget(mode_group)

# Gripper Switch (Step 6 of Part 4 in lab manual)
grip_box = QGroupBox("Gripper Configuration")
grip_layout = QHBoxLayout()
grip_box.setLayout(grip_layout)
chk_grip = QCheckBox("Close Gripper (Recommended for Point A ↔ B Navigation)")
chk_grip.setChecked(False)
grip_layout.addWidget(chk_grip)
main_layout.addWidget(grip_box)

# Emergency Stop / Reset Button
btn_stop = QPushButton("Emergency Stop / Reset to Home")
def emergency_stop():
    rb_home.setChecked(True)
    chk_grip.setChecked(False)
btn_stop.clicked.connect(emergency_stop)
main_layout.addWidget(btn_stop)

control_window.show()

# 5. Periodic Control Loop via QTimer (30 Hz)
start_time = time.time()

def control_step():
    global commanded_joint_pos, desired_gripper_cmd
    now = time.time()
    current_time = now - start_time
    mode_id = grp_modes.checkedId()

    target = np.array([0.0, 0.0, 0.0, 0.0], dtype=np.float64)

    if mode_id == 0:
        # Home Position
        target = np.array([0.0, 0.0, 0.0, 0.0])

    elif mode_id == 1:
        # Scenario 1: Arm fully stretched out vertically (Elbow -90°), Shoulder ±30° (10s period)
        period = 10.0
        phase = (current_time % period) / period
        sign = 1.0 if phase < 0.5 else -1.0
        sh_amp = np.radians(30.0)
        target = np.array([0.0, sign * sh_amp, -np.pi / 2.0, 0.0])

    elif mode_id == 2:
        # Scenario 2: Arm fully stretched out, Shoulder +15° to +75° (10s period)
        period = 10.0
        phase = (current_time % period) / period
        sign = 1.0 if phase < 0.5 else -1.0
        sh_center = np.radians(45.0)
        sh_amp = np.radians(30.0)
        target = np.array([0.0, sh_center + sign * sh_amp, -np.pi / 2.0, 0.0])

    elif mode_id == 3:
        # Scenario 3: Elbow at 0° (arm bent), Shoulder ±30° (10s period)
        period = 10.0
        phase = (current_time % period) / period
        sign = 1.0 if phase < 0.5 else -1.0
        sh_amp = np.radians(30.0)
        target = np.array([0.0, sign * sh_amp, 0.0, 0.0])

    elif mode_id == 4:
        # Navigation: Point A <-> Point B (8s period)
        nav_period = 8.0
        phase = (current_time % nav_period) / nav_period
        target = POINT_A if phase < 0.5 else POINT_B

    commanded_joint_pos = target
    desired_gripper_cmd = 0.8 if chk_grip.isChecked() else 0.0

    # Send position commands to physical hardware
    myArm.read_write_std(
        phiCMD=commanded_joint_pos.tolist(),
        gprCMD=desired_gripper_cmd,
        baseLED=desired_LED_color
    )

    # Refresh MultiScope plots
    MultiScope.refreshAll()

# Clean shutdown handler
def close_event(event):
    global KILL_THREAD
    KILL_THREAD = True
    myArm.read_write_std([0.0, 0.0, 0.0, 0.0], gprCMD=0.0, baseLED=[1, 0, 0])
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
myArm.read_write_std([0.0, 0.0, 0.0, 0.0], gprCMD=0.0, baseLED=[1, 0, 0])
myArm.terminate()
print("\n[Hardware Part 4] Shut down gracefully.")
# endregion
