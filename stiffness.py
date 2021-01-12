import numpy as np
from arm import Manipulator
from angles import normalize
####################
### Set the simulation time parameters
###################

totalTime = 5 #sec
time = np.linspace(0,totalTime,totalTime*100 + 1)
deltaTime = time[1] - time[0]
timesteps = len(time)

#########################
#### A FORCE IS APPLIED TO THE END EFFECTOR FOR SOME DURATION
#### THE END EFFECTOR WILL MOVE ACCORDINGLY
#########################

externalForce = np.zeros((timesteps,2))
### APPLY FORCE FOR 1 sec
externalForce[101:201] = np.array([-10,-10])

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
                                  
    
    torqueP = np.linalg.multi_dot([Arm.Jacobian.T,Arm.matrix_KX,(desiredJointAngles - np.array([Arm.jointAngle1,Arm.jointAngle2]))])                              
    torqueV = -1 * np.linalg.multi_dot([Arm.matrix_KV,np.linalg.inv(Arm.Jacobian),Arm.jointVelocity])
    
    Arm.jointTorque = torqueP + torqueV 
    
    
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
    
    Arm.jointAngle1,Arm.jointAngle2 = normalize(Arm.jointAngle1,-np.pi,np.pi), normalize(Arm.jointAngle2,-np.pi,np.pi)
    
    
    #############
    ## Calculate All the parameters again
    #############
    
    Arm.getForwardKinematics()
    Arm.getJacobian()
    Arm.getMassMatrix()
    Arm.getCoriolisMatrix()
    Arm.getGravityMatix()
    
    
    print(Arm.jointAngle1)
    
    
    
    
    
        
    







#####################

