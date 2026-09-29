# Recording changes made to the Repo

Temperary Documents live in:
https://drive.google.com/drive/folders/12CBtRyjxJgaLnXKr31zrNcBCggSQFpnT?dmr=1&ec=wgc-drive-%5Bmodule%5D-goto

## SP0_Play

### Modified: pal/products/qarm.py
- Added a `jointmode` argument to the contructor
    - This is used to specify position mode (0) or pwm mode (1)

### Created: hal/content/qarm.py
- Implemented QArmKeyboardNavigator (from QArmMiniKeyboardNavigator)
    - The keypress in statements had to be adjusted from `if self.kbd.K_UP` to `if self.kbd.states[self.kbd.K_UP]`
    - In the constructor `self.armMath.forward_kinematics(phi=initialPose)` took a gamma output, which does not exist, so set gamma to `0` (open)

- Did NOT implement QArmMiniFunctions --> Most of it is already coverd by QArmUtilities

### play.py
- Based on the QArmMini one.

### digital_twin/BasicIO_position_mode.py
- Based on the simulink program of the same name
