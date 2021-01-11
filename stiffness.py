import numpy as np
from arm import Manipulator

####################
### Set the simulation time parameters
###################

totalTime = 10 #sec
time = np.linspace(0,totalTime,totalTime*100 + 1)
deltaTime = time[1] - time[0]
timesteps = len(time)

#########################
#### A FORCE IS APPLIED TO THE END EFFECTOR FOR SOME DURATION
#### THE END EFFECTOR HAS TO MAINTAIN THE DESIRED POSITION
#########################

externalForce = np.zeros((timesteps,2))
externalForce[101:201] = np.array([-1,-1])

#####################

