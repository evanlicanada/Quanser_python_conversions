"""
qarm.py: Enhanced QArm class supporting both Position Mode (0) and PWM Mode (1).
Inherits from pal.products.qarm.QArm to maintain full API compatibility.
"""
import numpy as np
from pal.products.qarm import QArm as _PalQArm
from quanser.hardware import HIL, HILError, MAX_STRING_LENGTH, Clock
from quanser.hardware.enumerations import BufferOverflowMode

class QArm(_PalQArm):
    """
    QArm class for initialization, I/O, and termination supporting both
    Position Mode (jointMode=0) and PWM Mode (jointMode=1).
    """

    WRITE_OTHER_CHANNELS_PWM = np.array([
        1000, 1001, 1002, 1003, # Phi cmd (zeros in PWM mode)
        1004,                   # Gripper cmd (0.1 - 0.9)
        11000, 11001, 11002, 11003, # Joint PWM commands (-1.0 to 1.0)
        11005, 11006, 11007     # Base RGB LED (0 - 1)
    ], dtype=np.int32)

    def __init__(self, hardware=1, readMode=0, frequency=500, deviceId=0, hilPort=18900, jointMode=0):
        """
        Initializes and configures the QArm in Position Mode or PWM Mode.

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
        jointMode : int, optional
            Control mode: 0 = Position Mode, 1 = PWM Mode. Defaults to 0.
        """
        self.jointMode = jointMode
        self.readMode = readMode
        self.hardware = hardware
        self.status = False
        self.card = HIL()

        if self.hardware:
            boardIdentifier = str(deviceId)
        else:
            boardIdentifier = "0@tcpip://localhost:" + str(hilPort) + "?nagle='off'"

        boardSpecificOptions = (
            f"j0_mode={jointMode};j1_mode={jointMode};j2_mode={jointMode};j3_mode={jointMode};"
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
                    mode_name = "PWM Mode" if jointMode == 1 else "Position Mode"
                    print(f"QArm configured in {mode_name}.")
        except HILError as h:
            print(h.get_error_message())

        self.writeOtherBufferPWM = np.zeros(len(self.WRITE_OTHER_CHANNELS_PWM), dtype=np.float64)

    def read_write_pwm(self, pwmCMD=np.zeros(4, dtype=np.float64),
                       gprCMD=np.zeros(1, dtype=np.float64),
                       baseLED=np.array([0, 1, 0], dtype=np.float64)):
        """
        Writes PWM commands to joints (normalized duty cycle, -1.0 to 1.0) and reads measurements.

        Parameters
        ----------
        pwmCMD : numpy.ndarray, optional
            PWM commands for joints 0 to 3 as a 4-element array (-1.0 to 1.0). Defaults to zeros.
        gprCMD : numpy.ndarray, optional
            Gripper command (0.1 open to 0.9 closed). Defaults to zeros.
        baseLED : numpy.ndarray, optional
            Base RGB LED state [R, G, B]. Defaults to green: [0, 1, 0].
        """
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
