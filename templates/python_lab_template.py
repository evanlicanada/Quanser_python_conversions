# TEAM: Add name of the experiment here

# TEAM: Add a description of the experiment here 

# region: Python level imports
import numpy as np
import cv2
from pal.utilities.timing import Timer
from pal.products.sensors import SensorsCamera
from hal.utilities.image_processing import ImageProcessing 

# endregion 


#region: Scope constants
# TEAM: Define any constants needed for using the quanser scope here. Leave blank if not using the scope.
# endregion

#region: Scope setup or or Probe/Observer setup (pick one)
# TEAM: Setup any scopes or probes/observers needed for your experiment here. Leave blank if not using the scope.
# endregion

# region: Experiment constants
# TEAM Define any constants needed for your experiment here. This is ideally where students change values and see the result based on the lab

simulationTime = 120 # will run for this amount of seconds
frameRate = 30
imageSize = [640,480]
# endregion


# region: Main Loop
# TEAM: Implement the main code for your experiment here. Use comments to guide students through the steps needed to complete the lab.

timer = Timer(sampleRate=frameRate, totalTime=simulationTime)
imageProcessingTool = ImageProcessing()


with  SensorsCamera(frameWidth=imageSize[0], 
                    frameHeight=imageSize[1], 
                    frameRate=frameRate, 
                    cameraID=1) as camera:

    while timer.check():
        
        currentTime = timer.get_current_time()

        
        # read camera for 30Hz
        frame = camera.read()
        if frame:

            # obtain image information
            image = camera.imageData

            cv2.imshow("Color Image", image)
            cv2.waitKey(1)



        timer.sleep()
    

cv2.destroyAllWindows()
# endregion

input ("Press Enter to exit...")





