#-----------------------------------------------------------------------------#
#------------------Skills Progression 0 - Play--------------------------------#
#-----------------------------------------------------------------------------#
#---------------------------Lab 1 - QArm -------------------------------------#
#-----------------------------------------------------------------------------#

# imports
import numpy as np
from pal.products.qarm import QArm
from hal.products.qarm import QArmUtilities
from hal.content.qarm import QArmKeyboardNavigator, DataIO, PlotData # I need to figure how to register this file
from pal.utilities.keyboard import QKeyboard
from pal.utilities.timing import QTimer

# Setup
poseData, jointData, timeData  = [], [], []
kbd         = QKeyboard()
myArm       = QArm(hardware=1, readMode = 0)
kbdNav      = QArmKeyboardNavigator(keyboardDriver=kbd, initialPose=myArm.HOME_POSE)
myArmMath   = QArmUtilities()
timer       = QTimer(sampleRate=30.0, totalTime=300.0)

try :
    # main loop
    while timer.check():
        kbd.update()

        new_pos = kbdNav.move_joints_with_keyboard(timer.get_sample_time(), speed=np.pi/4)

        ## Section C - QArm hardware I/O
        myArm.read_write_std(
            new_pos)

        ## Section D - Record data for plotting
        pose, rotationMatrix = myArmMath.forward_kinematics(myArm.measJointPosition)

        if kbd.states[kbd.K_SPACE]:
            poseData.append(np.array(pose))
            jointData.append(np.array(myArm.measJointPosition))
            timeData.append(timer.get_current_time())

        timer.sleep(verbose=False)

except KeyboardInterrupt:
    print('Received user terminate command.')

finally:
    # Section E - Plot data
    plotter = PlotData()
    plotter.plot3D1Curve(poseData)
    plotter.plot2DJointCurves(jointData, timeData)

    # Section F - Save data
    dataHandler = DataIO(filename='end_effector_data.csv')
    dataHandler.write(poseData, mode='end-effector-pose')

    # terminate devices
    myArm.terminate()
