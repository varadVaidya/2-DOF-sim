import numpy as np
from arm import Manipulator
from angles import normalize
import matplotlib.pyplot as plt
####################
### Set the simulation time parameters
###################

totalTime = 10 #sec
time = np.linspace(0,totalTime,totalTime*100 + 1)
deltaTime = time[1] - time[0]
timesteps = len(time)

############
## CREATE A MATRIX TO STORE ALL THE STUFF

storageMatrix = np.empty([timesteps,8])
########### 

#########################
#### A FORCE IS APPLIED TO THE END EFFECTOR FOR SOME DURATION
#### THE END EFFECTOR WILL MOVE ACCORDINGLY
#########################

externalForce = np.zeros((timesteps,2))
### APPLY FORCE FOR 1 sec
# externalForce[101:201] = np.array([-10,-10])
externalForce[101:103] = np.array([10,10])

##################
### SET UP THE MANIPULATOR
##################

Arm = Manipulator()
Arm.jointAngle1,Arm.jointAngle2 = np.pi/6, -np.pi/6
Arm.getForwardKinematics()
Arm.getJacobian()
Arm.getMassMatrix()
Arm.getCoriolisMatrix()
Arm.getGravityMatix()

desiredJointAngles = np.array([Arm.jointAngle1,Arm.jointAngle2])



                                                                                     


for i in range(timesteps):
    # Arm.jointTorque = np.dot(Arm.massMatrix,Arm.jointAccel) + Arm.coriolisMatrix + \
    #                               Arm.gravityMatix + np.dot(Arm.Jacobian.T,externalForce[i])
                                  
    ##############
    ##### STORE STUFF
    ##############
    storageMatrix[i,0] = Arm.jointAngle1
    storageMatrix[i,1] = Arm.jointAngle2
    storageMatrix[i,2:4] = Arm.jointVelocity
    storageMatrix[i,4:6] = Arm.jointAccel
    storageMatrix[i,6:8] = Arm.jointTorque
    ###############
    
    
    ###################################
    ############        WRONG CODE BELOW.
    ############        DID NOT WORK. CAUSED NAN IN JOINT ANGLES
    ############        JOINT VALUE CROSSES THE DOUBLE SCALAR LIMIT WITHIN ONE ITERATION
    ############        DONOT TOUCH.
    ###################################
    
    # # torqueP = np.linalg.multi_dot([Arm.Jacobian.T,Arm.matrix_KX,(desiredJointAngles - np.array([normalize(Arm.jointAngle1),normalize(Arm.jointAngle2)]))])                              
    # # torqueV = np.linalg.multi_dot([Arm.matrix_KV,np.linalg.inv(Arm.Jacobian),-Arm.jointVelocity])
    
    # # Arm.jointTorque = torqueP + torqueV + Arm.gravityMatix
    
    #### WROND CODE ENDS
    #################################
    
    
    
    ###########
    #### Applying stuff from an old paper
    #### https://ieeexplore.ieee.org/document/4046624
    ##### hope for the best
    
    jointStiffness = np.linalg.multi_dot([ Arm.Jacobian , Arm.matrix_KX , np.transpose(Arm.Jacobian) , (desiredJointAngles - np.array([normalize(Arm.jointAngle1),normalize(Arm.jointAngle2)]) )])
    jointDamping   = np.linalg.multi_dot( [Arm.Jacobian , Arm.matrix_KV , np.transpose(Arm.Jacobian) , Arm.jointVelocity])
    
    jointExternalForce = np.dot(np.transpose(Arm.Jacobian),externalForce[i])
    
    
     
    
    
    # print(Arm.coriolisMatrix)                             
    # print(initTorque)
    # input("HELL")
    
    #################
    ### Calculate Forward Dynamics
    #################
    Arm.jointAccel = np.dot( np.linalg.inv(Arm.massMatrix) , Arm.jointTorque - 
                            Arm.coriolisMatrix - Arm.gravityMatix - np.dot(Arm.Jacobian.T,externalForce[i]))
    ############
    ### Apply Euler Step
    ############
    
    Arm.jointVelocity = Arm.jointAccel * deltaTime + Arm.jointVelocity
    Arm.jointAngle1,Arm.jointAngle2 = Arm.jointVelocity * deltaTime + np.array([Arm.jointAngle1,Arm.jointAngle2])
    
    #Arm.jointAngle1,Arm.jointAngle2 = normalize(Arm.jointAngle1,0,2*np.pi), normalize(Arm.jointAngle2,0,2*np.pi)
    
    
    
    
    #############
    ## Calculate All the parameters again
    #############
    
    Arm.getForwardKinematics()
    Arm.getJacobian()
    Arm.getMassMatrix()
    Arm.getCoriolisMatrix()
    Arm.getGravityMatix()
    Arm.jointTorque = jointStiffness + jointDamping + jointExternalForce + Arm.gravityMatix 
    
    
    

###################################
########PLOT AND ANIMATION
Arm.plotStuff(storageMatrix,time)

plt.plot(time,storageMatrix[:,6])
plt.plot(time,storageMatrix[:,6])
###################################


    
    
    
    
        
    







#####################

