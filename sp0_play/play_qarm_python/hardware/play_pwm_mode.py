# TEAM: QArm Play (PWM Mode) - Hardware Sequential Joint Direction Verification

# TEAM: In PWM mode on hardware, Student A firmly holds the QArm at the gripper motor to feel joint torque and prevent gravity fall,
# while Student B monitors the UI and presses SPACE to run sequential ±1 PWM actuation for each joint to verify direction conventions.

# region: Python level imports
import os
import sys
import time
import signal
from threading import Thread
import numpy as np

from PyQt6.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QLabel, QPushButton, QProgressBar, QFrame
)
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QKeyEvent, QFont

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
pwmLim = [-0.5, 0.5]          # PWM duty cycle limits (-0.5 to 0.5)
tempLim = [20.0, 50.0]        # Motor temperature limits (°C)
# endregion


# region: Scope setup or Probe/Observer setup (pick one)
# TEAM: Setup any scopes or probes/observers needed for your experiment here. Leave blank if not using the scope.
multiScope = MultiScope(
    rows=2,
    cols=3,
    title='QArm Hardware Measurements (PWM Mode)',
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

# Axis 3: PWM
multiScope.addAxis(row=1, col=0, timeWindow=scopeTimeWindow, yLabel='PWM', yLim=pwmLim)
multiScope.axes[3].attachSignal(name='basePWM')
multiScope.axes[3].attachSignal(name='shoulderPWM')
multiScope.axes[3].attachSignal(name='elbowPWM')
multiScope.axes[3].attachSignal(name='wristPWM')

# Axis 4: Temperature
multiScope.addAxis(row=1, col=1, timeWindow=scopeTimeWindow, yLabel='Temperature (°C)', yLim=tempLim)
multiScope.axes[4].attachSignal(name='baseTemp')
multiScope.axes[4].attachSignal(name='shoulderTemp')
multiScope.axes[4].attachSignal(name='elbowTemp')
multiScope.axes[4].attachSignal(name='wristTemp')
# endregion


# region: Experiment constants
# TEAM: Define any constants needed for your experiment here. This is ideally where students change values and see the result based on the lab

hardwareMode = 1            # 1 for Physical Hardware, 0 for Virtual Digital Twin
readMode = 0                # 0 for Immediate I/O, 1 for Task-based I/O
jointMode = 1               # 1 for PWM Mode (open-loop voltage control)

# Safe testing parameters matching Simulink model (play_pwm_mode.slx):
# In Simulink, ±1 commands pass through a 10% gain block (0.10 duty cycle / ~1.2V)
# so the arm exerts a gentle, easily palpable torque without moving aggressively.
pwm_amplitude = 0.10        # 10% duty cycle (safe for manual holding)
step_duration = 2.0         # Seconds per step (matches Simulink tsamp=2)
defaultLedColor = [0, 1, 0] # Base LED color (Green)
scopeSampleInterval = 0.02  # 50 Hz background sampling rate

# Sequential joint actuation sequence: (joint_index, direction_multiplier, display_label)
# Matches Simulink system_766: Base +1, Base -1, Shoulder +1, Shoulder -1, Elbow +1, Elbow -1, Wrist +1, Wrist -1
SEQUENCE_STEPS = [
    (0, +1, "Base Joint (+1)"),
    (0, -1, "Base Joint (-1)"),
    (1, +1, "Shoulder Joint (+1)"),
    (1, -1, "Shoulder Joint (-1)"),
    (2, +1, "Elbow Joint (+1)"),
    (2, -1, "Elbow Joint (-1)"),
    (3, +1, "Wrist Joint (+1)"),
    (3, -1, "Wrist Joint (-1)"),
]
# endregion


# region: Main Loop
# TEAM: Implement the main code for your experiment here. Use comments to guide students through the steps needed to complete the lab.

# Kill switch setup for clean exit on Ctrl+C
KILL_THREAD = False
def sig_handler(*args):
    global KILL_THREAD
    KILL_THREAD = True
signal.signal(signal.SIGINT, sig_handler)

# Hardware Initialization in PWM Mode
# WARNING: When in PWM mode on hardware, joint motors are in open-loop voltage control.
# Student A must support the arm before launching this script!
myArm = QArm(hardware=hardwareMode, readMode=readMode, jointMode=jointMode)
desired_joint_pwm = [0.0, 0.0, 0.0, 0.0]
desired_LED_color = list(defaultLedColor)
myArm.read_write_pwm(desired_joint_pwm, baseLED=desired_LED_color)

# Background Data Collection Thread
def scope_loop():
    """Samples hardware values and pushes them to the MultiScope plots."""
    t0 = time.time()

    while not KILL_THREAD:
        t = time.time() - t0
        phi = np.array(myArm.measJointPosition)
        phidot = np.array(myArm.measJointSpeed)
        current = np.array(myArm.measJointCurrent)
        pwm = np.array(myArm.measJointPWM)
        temp = np.array(myArm.measJointTemperature)

        multiScope.axes[0].sample(t, phi)
        multiScope.axes[1].sample(t, phidot)
        multiScope.axes[2].sample(t, current)
        multiScope.axes[3].sample(t, pwm)
        multiScope.axes[4].sample(t, temp)

        time.sleep(scopeSampleInterval)

thread = Thread(target=scope_loop, daemon=True)
thread.start()

# PyQt6 Custom Control Panel with Spacebar Toggle and Joint Direction Displays
app = QApplication.instance()
if app is None:
    app = QApplication([])

# State variables for sequence execution
sequence_active = False
current_step_idx = -1
step_start_time = 0.0

class PWMControlWindow(QWidget):
    """Control window that intercepts Spacebar key presses to toggle the sequence."""
    def __init__(self):
        super().__init__()
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)

    def keyPressEvent(self, event: QKeyEvent):
        if event.key() == Qt.Key.Key_Space:
            toggle_sequence()
        elif event.key() == Qt.Key.Key_Escape:
            stop_sequence("STOPPED (Escape Key)")
        else:
            super().keyPressEvent(event)

control_window = PWMControlWindow()
control_window.setWindowTitle("QArm PWM Hardware Teleoperation — Direction Verification")
control_window.resize(540, 480)

main_layout = QVBoxLayout()
main_layout.setSpacing(12)
control_window.setLayout(main_layout)

# Header Title
title_label = QLabel("<h2>QArm PWM Direction Verification (Hardware)</h2>")
title_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
main_layout.addWidget(title_label)

# Safety Warning Box
safety_frame = QFrame()
safety_frame.setStyleSheet("""
    QFrame {
        background-color: #3d2c00;
        border: 2px solid #e6a100;
        border-radius: 6px;
        padding: 8px;
    }
""")
safety_layout = QVBoxLayout(safety_frame)
safety_text = QLabel(
    "<b>⚠️ SAFETY INSTRUCTIONS (2-Person Team):</b><br>"
    "• <b>Student A:</b> Firmly hold the QArm at the gripper motor. In PWM mode, the arm will <b>not</b> "
    "hold position against gravity without support!<br>"
    "• <b>Student B:</b> Press <b>SPACE</b> or click the button below to start the sequence. "
    "Press <b>SPACE</b> again at any time to <b>immediately stop</b> all actuation."
)
safety_text.setWordWrap(True)
safety_layout.addWidget(safety_text)
main_layout.addWidget(safety_frame)

# Main Status Banner
status_banner = QLabel("READY — Press SPACE to Start Sequence")
status_banner.setAlignment(Qt.AlignmentFlag.AlignCenter)
status_banner.setStyleSheet("""
    QLabel {
        font-size: 15px;
        font-weight: bold;
        color: #ffffff;
        background-color: #2b2b2b;
        border: 2px solid #555555;
        border-radius: 6px;
        padding: 10px;
    }
""")
main_layout.addWidget(status_banner)

# Subsystem: "What is about to move?" (Matching Simulink system_761)
subsys_group = QFrame()
subsys_group.setStyleSheet("QFrame { background-color: #1e1e1e; border-radius: 6px; padding: 6px; }")
subsys_layout = QVBoxLayout(subsys_group)

subsys_title = QLabel("<b>What is about to move?</b>")
subsys_title.setAlignment(Qt.AlignmentFlag.AlignCenter)
subsys_layout.addWidget(subsys_title)

# Grid of 4 joint indicator cards
cards_layout = QGridLayout()
joint_names = ["Base (Joint 0)", "Shoulder (Joint 1)", "Elbow (Joint 2)", "Wrist (Joint 3)"]
joint_cards = []
joint_val_labels = []

card_default_style = """
    QFrame {
        background-color: #2a2a2a;
        border: 1px solid #444444;
        border-radius: 6px;
        padding: 6px;
    }
"""
card_active_pos_style = """
    QFrame {
        background-color: #0d3b66;
        border: 2px solid #00aaff;
        border-radius: 6px;
        padding: 6px;
    }
"""
card_active_neg_style = """
    QFrame {
        background-color: #4a1e1e;
        border: 2px solid #ff4444;
        border-radius: 6px;
        padding: 6px;
    }
"""

for i in range(4):
    card = QFrame()
    card.setStyleSheet(card_default_style)
    c_layout = QVBoxLayout(card)
    c_layout.setContentsMargins(6, 6, 6, 6)

    lbl_name = QLabel(joint_names[i])
    lbl_name.setAlignment(Qt.AlignmentFlag.AlignCenter)
    lbl_name.setStyleSheet("font-weight: bold; font-size: 12px; color: #cccccc;")

    lbl_val = QLabel("0")
    lbl_val.setAlignment(Qt.AlignmentFlag.AlignCenter)
    lbl_val.setStyleSheet("font-size: 24px; font-weight: bold; color: #888888;")

    c_layout.addWidget(lbl_name)
    c_layout.addWidget(lbl_val)

    cards_layout.addWidget(card, 0, i)
    joint_cards.append(card)
    joint_val_labels.append(lbl_val)

subsys_layout.addLayout(cards_layout)
main_layout.addWidget(subsys_group)

# Progress Bar & Time Remaining
progress_bar = QProgressBar()
progress_bar.setRange(0, len(SEQUENCE_STEPS))
progress_bar.setValue(0)
progress_bar.setTextVisible(True)
progress_bar.setFormat("Step 0 / 8")
progress_bar.setStyleSheet("""
    QProgressBar {
        border: 1px solid #555;
        border-radius: 4px;
        text-align: center;
        height: 18px;
    }
    QProgressBar::chunk {
        background-color: #00aa66;
    }
""")
main_layout.addWidget(progress_bar)

time_label = QLabel("Step Time: 0.0s / 2.0s")
time_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
time_label.setStyleSheet("color: #aaaaaa; font-size: 11px;")
main_layout.addWidget(time_label)

# Big Start / Stop Button
btn_toggle = QPushButton("▶ Start Sequence (Press SPACE)")
btn_toggle.setFixedHeight(45)
btn_toggle.setStyleSheet("""
    QPushButton {
        background-color: #008844;
        color: white;
        font-size: 14px;
        font-weight: bold;
        border-radius: 6px;
    }
    QPushButton:hover {
        background-color: #00aa55;
    }
""")
main_layout.addWidget(btn_toggle)

# Emergency Stop Button
btn_stop = QPushButton("⏹ Emergency All Stop")
btn_stop.setFixedHeight(32)
btn_stop.setStyleSheet("""
    QPushButton {
        background-color: #882222;
        color: white;
        font-size: 12px;
        border-radius: 4px;
    }
    QPushButton:hover {
        background-color: #aa3333;
    }
""")
main_layout.addWidget(btn_stop)

def update_ui_cards(active_joint=None, direction=0):
    """Updates the 'What is about to move?' display cards."""
    for i in range(4):
        if active_joint == i:
            if direction > 0:
                joint_cards[i].setStyleSheet(card_active_pos_style)
                joint_val_labels[i].setText("+1")
                joint_val_labels[i].setStyleSheet("font-size: 24px; font-weight: bold; color: #00ddff;")
            else:
                joint_cards[i].setStyleSheet(card_active_neg_style)
                joint_val_labels[i].setText("-1")
                joint_val_labels[i].setStyleSheet("font-size: 24px; font-weight: bold; color: #ff6666;")
        else:
            joint_cards[i].setStyleSheet(card_default_style)
            joint_val_labels[i].setText("0")
            joint_val_labels[i].setStyleSheet("font-size: 24px; font-weight: bold; color: #666666;")

def start_sequence():
    """Starts the sequential joint movement testing."""
    global sequence_active, current_step_idx, step_start_time
    sequence_active = True
    current_step_idx = 0
    step_start_time = time.time()

    joint_idx, dir_mult, label = SEQUENCE_STEPS[current_step_idx]
    desired_joint_pwm[joint_idx] = dir_mult * pwm_amplitude

    status_banner.setText(f"ACTUATING: {label} (Signal: {dir_mult:+d})")
    status_banner.setStyleSheet("""
        QLabel {
            font-size: 15px; font-weight: bold; color: #ffffff;
            background-color: #005588; border: 2px solid #00aaff;
            border-radius: 6px; padding: 10px;
        }
    """)
    btn_toggle.setText("⏹ STOP Sequence (Press SPACE)")
    btn_toggle.setStyleSheet("""
        QPushButton {
            background-color: #cc2222; color: white;
            font-size: 14px; font-weight: bold; border-radius: 6px;
        }
        QPushButton:hover { background-color: #ee3333; }
    """)
    progress_bar.setValue(1)
    progress_bar.setFormat("Step 1 / 8: " + label)
    update_ui_cards(joint_idx, dir_mult)

def stop_sequence(reason="STOPPED by User (All Joints 0.0 PWM)"):
    """Immediately stops all motor commands and resets sequence state."""
    global sequence_active, current_step_idx
    sequence_active = False
    current_step_idx = -1

    for j in range(4):
        desired_joint_pwm[j] = 0.0

    myArm.read_write_pwm(desired_joint_pwm, baseLED=desired_LED_color)

    status_banner.setText(reason)
    status_banner.setStyleSheet("""
        QLabel {
            font-size: 15px; font-weight: bold; color: #ffffff;
            background-color: #551111; border: 2px solid #ff4444;
            border-radius: 6px; padding: 10px;
        }
    """)
    btn_toggle.setText("▶ Start Sequence (Press SPACE)")
    btn_toggle.setStyleSheet("""
        QPushButton {
            background-color: #008844; color: white;
            font-size: 14px; font-weight: bold; border-radius: 6px;
        }
        QPushButton:hover { background-color: #00aa55; }
    """)
    progress_bar.setValue(0)
    progress_bar.setFormat("Idle (Press SPACE to start)")
    time_label.setText("Step Time: 0.0s / 2.0s")
    update_ui_cards(None, 0)

def toggle_sequence():
    """Toggles between starting and stopping the sequence."""
    if sequence_active:
        stop_sequence("STOPPED by User (All Joints 0.0 PWM)")
    else:
        start_sequence()

btn_toggle.clicked.connect(toggle_sequence)
btn_stop.clicked.connect(lambda: stop_sequence("EMERGENCY STOP (All Joints 0.0 PWM)"))

control_window.show()

# Periodic Control Loop via QTimer (30 FPS)
def control_step():
    global sequence_active, current_step_idx, step_start_time

    now = time.time()

    if sequence_active and current_step_idx >= 0:
        elapsed = now - step_start_time
        time_label.setText(f"Step Time: {min(elapsed, step_duration):.1f}s / {step_duration:.1f}s")

        # Check if current step duration expired
        if elapsed >= step_duration:
            current_step_idx += 1

            if current_step_idx < len(SEQUENCE_STEPS):
                # Advance to next joint actuation
                step_start_time = now
                for j in range(4):
                    desired_joint_pwm[j] = 0.0

                joint_idx, dir_mult, label = SEQUENCE_STEPS[current_step_idx]
                desired_joint_pwm[joint_idx] = dir_mult * pwm_amplitude

                status_banner.setText(f"ACTUATING: {label} (Signal: {dir_mult:+d})")
                progress_bar.setValue(current_step_idx + 1)
                progress_bar.setFormat(f"Step {current_step_idx + 1} / 8: {label}")
                update_ui_cards(joint_idx, dir_mult)
            else:
                # All 8 steps completed! Cleanly finish sequence
                stop_sequence("SEQUENCE COMPLETE — Verified all 4 joints in both directions!")

    # Send physical PWM commands to arm and read sensor feedback
    myArm.read_write_pwm(
        desired_joint_pwm,
        gprCMD=0.1,
        baseLED=desired_LED_color
    )

    # Refresh MultiScope plots
    MultiScope.refreshAll()

# Start Qt Timer and Event Loop
timer = QTimer()
timer.timeout.connect(control_step)
timer.start(int(1000 / scopeFps))

app.exec()

# Clean Shutdown: Ensure 0 PWM sent to motors before terminating
for j in range(4):
    desired_joint_pwm[j] = 0.0
myArm.read_write_pwm(desired_joint_pwm, gprCMD=0.1, baseLED=[1, 0, 0])
KILL_THREAD = True
myArm.terminate()
print("\nShutting down gracefully...")
# endregion
