# TEAM: QArm Basic I/O (PWM Mode) - Digital Twin

# TEAM: Command open-loop PWM voltage signals to each joint of the QArm manipulator (-12V to 12V / -100% to 100%),
# observe gravity effects and positive joint conventions, and monitor real-time sensor feedback on a MultiScope.

# region: Python level imports
import os
import sys
import time
import signal
from threading import Thread
import numpy as np

from PyQt6.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QLineEdit, QSlider, QPushButton
)
from PyQt6.QtCore import Qt, QTimer

from hal.products.qarm import QArmUtilities
from hal.content.qarm import QArmKeyboardNavigator, DataIO, PlotData

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
scopeTimeWindow = 5.0

# Fixed Y-axis limits prevent misleading micro-scale auto-zooming around near-zero values
phiLim = [-3.5, 3.5]          # Angular position limits (rad)
phiDotLim = [-2.0, 2.0]       # Angular speed limits (rad/s)
currentLim = [-2.0, 2.0]      # Motor current limits (A)
gripperLim = [0.0, 1.0]       # Gripper position limits (0.0 open - 1.0 closed)
pwmLim = [-1.0, 1.0]          # PWM duty cycle limits (-1.0 to 1.0)
tempLim = [20.0, 50.0]        # Motor temperature limits (°C)
# endregion


# region: Scope setup or Probe/Observer setup (pick one)
# TEAM: Setup any scopes or probes/observers needed for your experiment here. Leave blank if not using the scope.
multiScope = MultiScope(
    rows=2,
    cols=3,
    title='QArm Measurements (PWM Mode)',
    fps=scopeFps,
    bufferSize=scopeBufferSize
)

# Axis 0: Phi (angular position)
multiScope.addAxis(row=0, col=0, timeWindow=scopeTimeWindow, yLabel='Phi (rad)', yLim=phiLim)
multiScope.axes[0].attachSignal(name='basePhi')
multiScope.axes[0].attachSignal(name='shoulderPhi')
multiScope.axes[0].attachSignal(name='elbowPhi')
multiScope.axes[0].attachSignal(name='wristPhi')

# Axis 1: Phi Dot (angular speed)
multiScope.addAxis(row=0, col=1, timeWindow=scopeTimeWindow, yLabel='Phi dot (rad/s)', yLim=phiDotLim)
multiScope.axes[1].attachSignal(name='basePhiDot')
multiScope.axes[1].attachSignal(name='shoulderPhiDot')
multiScope.axes[1].attachSignal(name='elbowPhiDot')
multiScope.axes[1].attachSignal(name='wristPhiDot')

# Axis 2: Current
multiScope.addAxis(row=0, col=2, timeWindow=scopeTimeWindow, yLabel='Current (A)', yLim=currentLim)
multiScope.axes[2].attachSignal(name='baseCurrent')
multiScope.axes[2].attachSignal(name='shoulderCurrent')
multiScope.axes[2].attachSignal(name='elbowCurrent')
multiScope.axes[2].attachSignal(name='wristCurrent')

# Axis 3: Gripper Position
multiScope.addAxis(row=1, col=0, timeWindow=scopeTimeWindow, yLabel='Gripper Position', yLim=gripperLim)
multiScope.axes[3].attachSignal(name='gripperPosition')

# Axis 4: PWM
multiScope.addAxis(row=1, col=1, timeWindow=scopeTimeWindow, yLabel='PWM', yLim=pwmLim)
multiScope.axes[4].attachSignal(name='basePWM')
multiScope.axes[4].attachSignal(name='shoulderPWM')
multiScope.axes[4].attachSignal(name='elbowPWM')
multiScope.axes[4].attachSignal(name='wristPWM')

# Axis 5: Temperature
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
jointMode = 1               # 0 for Position Mode, 1 for PWM Mode
scopeSampleInterval = 0.02  # Sampling rate for scope data collection (50 Hz)
defaultLedColor = [0, 1, 0] # Base LED color (Green)
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
desired_joint_pwm = [0.0, 0.0, 0.0, 0.0]
desired_LED_color = list(defaultLedColor)
myArm.read_write_pwm(desired_joint_pwm, baseLED=desired_LED_color)

# 3. Background Data Collection Thread
def scope_loop():
    """Samples hardware values and pushes them to the MultiScope plots."""
    t0 = time.time()

    while not KILL_THREAD:
        t = time.time() - t0
        phi = np.array(myArm.measJointPosition)
        phidot = np.array(myArm.measJointSpeed)
        current = np.array(myArm.measJointCurrent)
        gripperPosition = myArm.writeOtherBufferPWM[4]
        pwm = np.array(myArm.measJointPWM)
        temp = np.array(myArm.measJointTemperature)

        multiScope.axes[0].sample(t, phi)
        multiScope.axes[1].sample(t, phidot)
        multiScope.axes[2].sample(t, current)
        multiScope.axes[3].sample(t, gripperPosition)
        multiScope.axes[4].sample(t, pwm)
        multiScope.axes[5].sample(t, temp)

        time.sleep(scopeSampleInterval)

thread = Thread(target=scope_loop, daemon=True)
thread.start()

# 4. PyQt6 Custom Control Panel
app = QApplication.instance()
if app is None:
    app = QApplication([])

control_window = QWidget()
control_window.setWindowTitle("QArm Control Panel (PWM Mode)")
control_window.resize(460, 320)

main_layout = QVBoxLayout()
control_window.setLayout(main_layout)

title_label = QLabel(
    "<b>Joint PWM Control</b><br>"
    "<small>Click -1 / 0 / +1 for directional testing, or enter custom PWM values (-1.0 to 1.0)</small>"
)
title_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
main_layout.addWidget(title_label)

joint_entries = []

# Build the 4 rows for joint PWM inputs and -1 / 0 / +1 buttons
for i in range(4):
    row_layout = QHBoxLayout()
    row_layout.addWidget(QLabel(f"Joint {i}:"))

    btn_neg = QPushButton("-1")
    btn_zero = QPushButton("0")
    btn_pos = QPushButton("+1")

    btn_neg.setFixedWidth(45)
    btn_zero.setFixedWidth(45)
    btn_pos.setFixedWidth(45)

    entry = QLineEdit("0.0")
    entry.setFixedWidth(75)
    entry.setAlignment(Qt.AlignmentFlag.AlignCenter)
    joint_entries.append(entry)

    # Connect buttons to autofill the custom number box
    btn_neg.clicked.connect(lambda _, e=entry: e.setText("-1.0"))
    btn_zero.clicked.connect(lambda _, e=entry: e.setText("0.0"))
    btn_pos.clicked.connect(lambda _, e=entry: e.setText("1.0"))

    row_layout.addWidget(btn_neg)
    row_layout.addWidget(btn_zero)
    row_layout.addWidget(btn_pos)
    row_layout.addWidget(QLabel("PWM:"))
    row_layout.addWidget(entry)

    main_layout.addLayout(row_layout)

# Build Gripper Control row (0-100 mapped to 0.0-1.0)
grip_layout = QHBoxLayout()
grip_layout.addWidget(QLabel("Gripper:"))
gripper_slider = QSlider(Qt.Orientation.Horizontal)
gripper_slider.setRange(0, 100)
gripper_slider.setValue(0)
grip_layout.addWidget(gripper_slider)
main_layout.addLayout(grip_layout)

# Emergency Stop / Zero All PWM Button
btn_zero_all = QPushButton("Stop / Zero All PWM")
def zero_all():
    for entry in joint_entries:
        entry.setText("0.0")
btn_zero_all.clicked.connect(zero_all)
main_layout.addWidget(btn_zero_all)

control_window.show()

# 5. Periodic Control Loop via QTimer
def control_step():
    # Read GUI inputs and form PWM command
    for i in range(4):
        try:
            target_val = float(joint_entries[i].text())
        except ValueError:
            target_val = 0.0
        desired_joint_pwm[i] = target_val

    gripper_val = gripper_slider.value() / 100.0

    # Send physical PWM commands to arm and read feedback
    myArm.read_write_pwm(
        desired_joint_pwm,
        gprCMD=gripper_val,
        baseLED=desired_LED_color
    )

    # Refresh MultiScope plots
    MultiScope.refreshAll()

# 6. Start Qt Timer and Event Loop
timer = QTimer()
timer.timeout.connect(control_step)
timer.start(int(1000 / scopeFps))

app.exec()

# 7. Clean Shutdown: Ensure 0 PWM sent to motors before terminating
myArm.read_write_pwm([0.0, 0.0, 0.0, 0.0], gprCMD=0.1, baseLED=[1, 0, 0])
KILL_THREAD = True
myArm.terminate()
print("\nShutting down gracefully...")
# endregion
