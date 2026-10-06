# TEAM: QArm Low-Level Control (Position Mode) - Digital Twin
# TEAM: Baseline benchmark demonstrating factory closed-loop position control.
# Students monitor reference tracking, velocities, and currents during pulse tests
# and Point A to Point B navigation to serve as a target comparison for their custom PWM controllers.

# region: Python level imports
import os
import sys
import time
import signal
from threading import Thread
import numpy as np

from PyQt6.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QLabel, QPushButton, QSlider, QRadioButton, QButtonGroup, QGroupBox
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
phiDotLim = [-2.5, 2.5]       # Angular speed limits (rad/s)
currentLim = [-2.0, 2.0]      # Motor current limits (A)
errorLim = [-1.0, 1.0]        # Position tracking error limits (rad)
pwmLim = [-1.05, 1.05]        # Internal PWM duty cycle limits
tempLim = [15.0, 45.0]        # Motor temperature limits (°C)
# endregion


# region: Scope setup or Probe/Observer setup (pick one)
# TEAM: Setup any scopes or probes/observers needed for your experiment here. Leave blank if not using the scope.
multiScope = MultiScope(
    rows=2,
    cols=3,
    title='QArm Low-Level Control: Position Mode (Baseline Benchmark)',
    fps=scopeFps,
    bufferSize=scopeBufferSize
)

# Axis 0: Joint Positions (Commanded vs Measured for Base & Wrist)
multiScope.addAxis(row=0, col=0, timeWindow=scopeTimeWindow, yLabel='Phi (rad)', yLim=phiLim)
multiScope.axes[0].attachSignal(name='baseCmd')
multiScope.axes[0].attachSignal(name='baseMeas')
multiScope.axes[0].attachSignal(name='wristCmd')
multiScope.axes[0].attachSignal(name='wristMeas')

# Axis 1: Joint Position Tracking Error (Cmd - Meas)
multiScope.addAxis(row=0, col=1, timeWindow=scopeTimeWindow, yLabel='Error (rad)', yLim=errorLim)
multiScope.axes[1].attachSignal(name='baseError')
multiScope.axes[1].attachSignal(name='shoulderError')
multiScope.axes[1].attachSignal(name='elbowError')
multiScope.axes[1].attachSignal(name='wristError')

# Axis 2: Joint Angular Speeds
multiScope.addAxis(row=0, col=2, timeWindow=scopeTimeWindow, yLabel='Speed (rad/s)', yLim=phiDotLim)
multiScope.axes[2].attachSignal(name='baseSpeed')
multiScope.axes[2].attachSignal(name='shoulderSpeed')
multiScope.axes[2].attachSignal(name='elbowSpeed')
multiScope.axes[2].attachSignal(name='wristSpeed')

# Axis 3: Joint Motor Currents
multiScope.addAxis(row=1, col=0, timeWindow=scopeTimeWindow, yLabel='Current (A)', yLim=currentLim)
multiScope.axes[3].attachSignal(name='baseCurrent')
multiScope.axes[3].attachSignal(name='shoulderCurrent')
multiScope.axes[3].attachSignal(name='elbowCurrent')
multiScope.axes[3].attachSignal(name='wristCurrent')

# Axis 4: Internal PWM Feedback
multiScope.addAxis(row=1, col=1, timeWindow=scopeTimeWindow, yLabel='PWM Duty Cycle', yLim=pwmLim)
multiScope.axes[4].attachSignal(name='basePWM')
multiScope.axes[4].attachSignal(name='shoulderPWM')
multiScope.axes[4].attachSignal(name='elbowPWM')
multiScope.axes[4].attachSignal(name='wristPWM')

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
jointMode = 0               # 0 for Position Mode (Factory Controller)
scopeSampleInterval = 0.02  # Sampling rate for scope data collection (50 Hz)
defaultLedColor = [0, 1, 0] # Base LED color (Green)

# Point A and Point B for Navigation benchmark (only Base rotates between 45 deg and 135 deg)
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

# 2. Hardware / Digital Twin Initialization in Position Mode
myArm = QArm(hardware=hardwareMode, readMode=readMode, jointMode=jointMode)
commanded_joint_pos = np.array([0.0, 0.0, 0.0, 0.0], dtype=np.float64)
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

        # Axis 0: Commanded vs Measured for Base & Wrist
        multiScope.axes[0].sample(t, [commanded_joint_pos[0], meas_phi[0], commanded_joint_pos[3], meas_phi[3]])
        # Axis 1: Tracking error for all 4 joints
        multiScope.axes[1].sample(t, err)
        # Axis 2: Measured joint speeds
        multiScope.axes[2].sample(t, meas_phidot)
        # Axis 3: Joint currents
        multiScope.axes[3].sample(t, meas_current)
        # Axis 4: Internal PWM signals
        multiScope.axes[4].sample(t, meas_pwm)
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
control_window.setWindowTitle("QArm Low-Level Control - Position Mode (Baseline Benchmark)")
control_window.resize(520, 420)

main_layout = QVBoxLayout()
control_window.setLayout(main_layout)

title_label = QLabel(
    "<h3>QArm Position Mode (Factory Baseline)</h3>"
    "<p>Select an operational mode to observe factory closed-loop tracking metrics "
    "(rise time, overshoot, steady-state error) as the baseline for PWM tuning.</p>"
)
title_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
main_layout.addWidget(title_label)

# Operating Mode Selection Group
mode_group = QGroupBox("Operating Mode")
mode_layout = QVBoxLayout()
mode_group.setLayout(mode_layout)

rb_home = QRadioButton("Home Position [0, 0, 0, 0] rad")
rb_pulse_base = QRadioButton("Pulse Test: Base Joint (±45°, 6s period)")
rb_pulse_wrist = QRadioButton("Pulse Test: Wrist Joint (±45°, 6s period)")
rb_point_ab = QRadioButton("Application / Navigation: Point A ↔ Point B (Base 45° ↔ 135°)")
rb_manual = QRadioButton("Manual Joint Sliders")

rb_home.setChecked(True)

btn_group = QButtonGroup()
btn_group.addButton(rb_home, 0)
btn_group.addButton(rb_pulse_base, 1)
btn_group.addButton(rb_pulse_wrist, 2)
btn_group.addButton(rb_point_ab, 3)
btn_group.addButton(rb_manual, 4)

mode_layout.addWidget(rb_home)
mode_layout.addWidget(rb_pulse_base)
mode_layout.addWidget(rb_pulse_wrist)
mode_layout.addWidget(rb_point_ab)
mode_layout.addWidget(rb_manual)
main_layout.addWidget(mode_group)

# Manual Sliders Group
manual_group = QGroupBox("Manual Joint Positioning (rad)")
slider_layout = QGridLayout()
manual_group.setLayout(slider_layout)

joint_sliders = []
joint_labels = []
joint_names = ["Base", "Shoulder", "Elbow", "Wrist"]

for i, name in enumerate(joint_names):
    slider = QSlider(Qt.Orientation.Horizontal)
    slider.setRange(-314, 314)  # -3.14 to +3.14 rad (scale 100)
    slider.setValue(0)
    lbl = QLabel("0.00 rad")
    lbl.setFixedWidth(65)
    slider.valueChanged.connect(lambda val, l=lbl: l.setText(f"{val / 100.0:+.2f} rad"))

    slider_layout.addWidget(QLabel(f"{name}:"), i, 0)
    slider_layout.addWidget(slider, i, 1)
    slider_layout.addWidget(lbl, i, 2)
    joint_sliders.append(slider)
    joint_labels.append(lbl)

manual_group.setEnabled(False)
main_layout.addWidget(manual_group)

def on_mode_changed(btn_id):
    manual_group.setEnabled(btn_id == 4)

btn_group.idClicked.connect(on_mode_changed)

# Stop / Home Button
btn_stop = QPushButton("Emergency Stop / Reset to Home")
def reset_home():
    rb_home.setChecked(True)
    manual_group.setEnabled(False)
    for s in joint_sliders:
        s.setValue(0)
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
        # Base Joint Pulse (±45 deg)
        phase = (current_time % PULSE_PERIOD) / PULSE_PERIOD
        sign = 1.0 if phase < 0.5 else -1.0
        target = np.array([sign * PULSE_AMPLITUDE, 0.0, 0.0, 0.0])

    elif mode_id == 2:
        # Wrist Joint Pulse (±45 deg)
        phase = (current_time % PULSE_PERIOD) / PULSE_PERIOD
        sign = 1.0 if phase < 0.5 else -1.0
        target = np.array([0.0, 0.0, 0.0, sign * PULSE_AMPLITUDE])

    elif mode_id == 3:
        # Point A <-> Point B Navigation (8s period)
        nav_period = 8.0
        phase = (current_time % nav_period) / nav_period
        target = POINT_A if phase < 0.5 else POINT_B

    elif mode_id == 4:
        # Manual Sliders
        target = np.array([s.value() / 100.0 for s in joint_sliders])

    commanded_joint_pos = target

    # Send position command to QArm and read feedback
    myArm.read_write_std(
        phiCMD=commanded_joint_pos.tolist(),
        gprCMD=0.0,
        baseLED=desired_LED_color
    )

    # Refresh MultiScope plots
    MultiScope.refreshAll()

# Clean shutdown when window is closed
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
print("\n[Position Mode] Shut down gracefully.")
# endregion
