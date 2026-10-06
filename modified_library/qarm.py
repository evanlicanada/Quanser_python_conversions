"""
qarm.py: Enhanced QArm class supporting Position Mode, PWM Mode, Current Mode, and Hybrid Modes.
Inherits from pal.products.qarm.QArm to maintain full API compatibility.
"""
import numpy as np
from pal.products.qarm import QArm as _PalQArm
from quanser.hardware import HIL, HILError, MAX_STRING_LENGTH, Clock
from quanser.hardware.enumerations import BufferOverflowMode

class QArm(_PalQArm):
    """
    QArm class for initialization, I/O, and termination supporting:
    - Position Mode (jointMode=0)
    - PWM Mode (jointMode=1)
    - Hybrid Modes (e.g. jointMode=[0, 0, 0, 1] for Position Base/Shoulder/Elbow and PWM Wrist)
    - Current Mode (e.g. jointMode=[0, 0, 0, 254] for Position Base/Shoulder/Elbow and Current Wrist)
    """

    WRITE_OTHER_CHANNELS_PWM = np.array([
        1000, 1001, 1002, 1003,     # Phi cmd (0 to 3)
        1004,                       # Gripper cmd (0.1 - 0.9)
        11000, 11001, 11002, 11003, # Joint PWM commands (-1.0 to 1.0)
        11005, 11006, 11007         # Base RGB LED (0 - 1)
    ], dtype=np.int32)

    WRITE_ANALOG_CHANNELS_CURRENT = np.array([3], dtype=np.int32)

    def __init__(self, hardware=1, readMode=0, frequency=500, deviceId=0, hilPort=18900, jointMode=0):
        """
        Initializes and configures the QArm in Position Mode, PWM Mode, or Hybrid Mode.

        Parameters
        ----------
        hardware : int, optional
            0 for virtual QArm (Digital Twin), 1 for physical hardware. Defaults to 1.
        readMode : int, optional
            0 for immediate I/O, 1 for task-based I/O. Defaults to 0.
        frequency : int, optional
            Sampling frequency in Hz. Defaults to 500.
        deviceId : int, optional
            Identifier for the QArm device. Defaults to 0.
        hilPort : int, optional
            Port number for virtual QArm connection. Defaults to 18900.
        jointMode : int or sequence of 4 ints, optional
            Control mode(s):
            - Scalar 0: Position Mode for all joints
            - Scalar 1: PWM Mode for all joints
            - Sequence of 4 ints [j0, j1, j2, j3]: Custom mode per joint
              (e.g., [0, 0, 0, 1] for hybrid PWM, or [0, 0, 0, 254] for hybrid current).
            Defaults to 0.
        """
        self.jointMode = jointMode
        self.readMode = readMode
        self.hardware = hardware
        self.status = False
        self.card = HIL()

        if isinstance(jointMode, (list, tuple, np.ndarray)):
            self.j_modes = [int(m) for m in jointMode]
        else:
            self.j_modes = [int(jointMode)] * 4

        if self.hardware:
            boardIdentifier = str(deviceId)
        else:
            boardIdentifier = "0@tcpip://localhost:" + str(hilPort) + "?nagle='off'"

        boardSpecificOptions = (
            f"j0_mode={self.j_modes[0]};j1_mode={self.j_modes[1]};"
            f"j2_mode={self.j_modes[2]};j3_mode={self.j_modes[3]};"
            "gripper_mode=0;j0_profile_config=0;j0_profile_velocity=1.5708;"
            "j0_profile_acceleration=1.0472;j1_profile_config=0;"
            "j1_profile_velocity=1.5708;j1_profile_acceleration=1.0472;"
            "j2_profile_config=0;j2_profile_velocity=1.5708;"
            "j2_profile_acceleration=1.0472;j3_profile_config=0;"
            "j3_profile_velocity=1.5708;j3_profile_acceleration=1.0472;"
        )

        try:
            self.card.open("qarm_usb", boardIdentifier)
            if self.card.is_valid():
                self.card.set_card_specific_options(boardSpecificOptions, MAX_STRING_LENGTH)
                self.status = True
                if self.readMode == 1:
                    self.frequency = frequency
                    self.samples = HIL.INFINITE
                    self.samplesToRead = 1

                    self.readTask = self.card.task_create_reader(
                        int(self.frequency),
                        self.READ_ANALOG_CHANNELS, len(self.READ_ANALOG_CHANNELS),
                        None, 0,
                        None, 0,
                        self.READ_OTHER_CHANNELS, len(self.READ_OTHER_CHANNELS)
                    )
                    if self.hardware:
                        self.card.task_set_buffer_overflow_mode(self.readTask, BufferOverflowMode.OVERWRITE_ON_OVERFLOW)
                    else:
                        self.card.task_set_buffer_overflow_mode(self.readTask, BufferOverflowMode.SYNCHRONIZED)
                    self.card.task_start(self.readTask, Clock.HARDWARE_CLOCK_0, self.frequency, 2**32-1)
                else:
                    if self.j_modes == [0, 0, 0, 0]:
                        mode_name = "Position Mode"
                    elif self.j_modes == [1, 1, 1, 1]:
                        mode_name = "PWM Mode"
                    else:
                        mode_name = f"Hybrid Mode (j0={self.j_modes[0]}, j1={self.j_modes[1]}, j2={self.j_modes[2]}, j3={self.j_modes[3]})"
                    print(f"QArm configured in {mode_name}.")
        except HILError as h:
            print(h.get_error_message())

        self.writeOtherBufferPWM = np.zeros(len(self.WRITE_OTHER_CHANNELS_PWM), dtype=np.float64)
        self.writeAnalogBufferCurrent = np.zeros(len(self.WRITE_ANALOG_CHANNELS_CURRENT), dtype=np.float64)

    def read_write_pwm(self, pwmCMD=np.zeros(4, dtype=np.float64),
                       phiCMD=np.zeros(4, dtype=np.float64),
                       gprCMD=np.zeros(1, dtype=np.float64),
                       baseLED=np.array([0, 1, 0], dtype=np.float64)):
        """
        Writes PWM commands to joints (normalized duty cycle, -1.0 to 1.0) and/or position commands
        to position-controlled joints in hybrid mode, and reads sensor measurements.

        Parameters
        ----------
        pwmCMD : numpy.ndarray or list, optional
            PWM commands for joints 0 to 3 as a 4-element array (-1.0 to 1.0). Defaults to zeros.
        phiCMD : numpy.ndarray or list, optional
            Position commands for joints 0 to 3 in radians (used for joints in Position mode). Defaults to zeros.
        gprCMD : numpy.ndarray, optional
            Gripper command (0.1 open to 0.9 closed). Defaults to zeros.
        baseLED : numpy.ndarray, optional
            Base RGB LED state [R, G, B]. Defaults to green: [0, 1, 0].
        """
        self.writeOtherBufferPWM[0:4] = phiCMD
        self.writeOtherBufferPWM[4] = np.clip(gprCMD, 0.1, 0.9)
        self.writeOtherBufferPWM[5:9] = np.clip(pwmCMD, -1.0, 1.0)
        self.writeOtherBufferPWM[9:] = baseLED

        try:
            self.card.write(
                None, 0,
                None, 0,
                None, 0,
                self.WRITE_OTHER_CHANNELS_PWM, len(self.WRITE_OTHER_CHANNELS_PWM),
                None,
                None,
                None,
                self.writeOtherBufferPWM
            )
            self.read_std()
        except HILError as h:
            print(h.get_error_message())

    def read_write_current(self, currentCMD=0.0,
                           phiCMD=np.zeros(4, dtype=np.float64),
                           gprCMD=np.zeros(1, dtype=np.float64),
                           baseLED=np.array([0, 1, 0], dtype=np.float64)):
        """
        Writes wrist current command (Amperes) to analog channel 3 and position commands to
        other joints in hybrid mode, and reads sensor measurements.

        Parameters
        ----------
        currentCMD : float, optional
            Wrist motor current command in Amperes (typically -1.0 to 1.0 A).
        phiCMD : numpy.ndarray or list, optional
            Position commands for joints 0 to 3 in radians.
        gprCMD : numpy.ndarray, optional
            Gripper command (0.1 open to 0.9 closed). Defaults to zeros.
        baseLED : numpy.ndarray, optional
            Base RGB LED state [R, G, B]. Defaults to green: [0, 1, 0].
        """
        self.writeAnalogBufferCurrent[0] = currentCMD
        self.writeOtherBufferPWM[0:4] = phiCMD
        self.writeOtherBufferPWM[4] = np.clip(gprCMD, 0.1, 0.9)
        self.writeOtherBufferPWM[5:9] = 0.0
        self.writeOtherBufferPWM[9:] = baseLED

        try:
            self.card.write(
                self.WRITE_ANALOG_CHANNELS_CURRENT, len(self.WRITE_ANALOG_CHANNELS_CURRENT),
                None, 0,
                None, 0,
                self.WRITE_OTHER_CHANNELS_PWM, len(self.WRITE_OTHER_CHANNELS_PWM),
                self.writeAnalogBufferCurrent,
                None,
                None,
                self.writeOtherBufferPWM
            )
            self.read_std()
        except HILError as h:
            print(h.get_error_message())
