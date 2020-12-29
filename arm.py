import numpy as np
from math import sin,cos,tan

#########################
##      MANIPULATOR CLASS FOR 2 DOF SERIAL MANIPULATOR   ##
##################
class Manipulator:
    
    def __init__(self) -> None:

        """
        Set parameters for the robot.
        """
        self.linkmass1 = 1
        self.linkmass2 = 1
        self.linklength = 1
        self.linklength = 1


        ### JOINT VECTOR OF THE ROBOT.
        self.jointAngle1 = None
        self.jointAngle2 = None

        ### CURRENT END EFFECOTR POSITION
        self.endEffectorPosition = None

        super().__init__()


    def getForwardKinematics():
        """
        Use standard trig. to calculate the end effector position....
        returns: (x,y) np array :: containing the final end effector position
        returns in the endEffectorPosition variable 
        """
        pass
    
    def getInverseKinematics():
        """
        Use Analytical method to return the joint angles vector
        Returns the value in the jointAngle1 and jointAngle2 variable

        """
        pass

    def getJacobian():
        """
        get the jacobian matrix to relate the end effector velocity to the joint vector velocity.
        """
        pass

    def getMassMatrix():
        """
        Calculate the Mass Matrix for Dynamics
        Calculation is done Analytically due to simplicity of the model..

        """
        pass


