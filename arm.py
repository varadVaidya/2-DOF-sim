import numpy as np
from math import acos, sin,cos, sqrt,tan,atan2
from numpy.lib.function_base import select

from numpy.lib.polynomial import polyder

#########################
##      MANIPULATOR CLASS FOR 2 DOF SERIAL MANIPULATOR   ##


### Mass of the link is assumed to be at the end of the link to simplify calculations
##################
class Manipulator:
    
    def __init__(self) :

        """
        Set parameters for the robot.
        """
        self.linkmass1 = 1
        self.linkmass2 = 1
        self.linklength1 = 1
        self.linklength2 = 1
        self.gravity = 9.81


        ### JOINT VECTOR OF THE ROBOT.
        ### ZERP<ZERO is set as the home position
        self.jointAngle1 = 0    #rads
        self.jointAngle2 = 0    #rads


        ### Joint VELOCITY VECTOR
        ### a numpy array of sixe 1*2
        self.jointVelocity = np.array([0,0])

        ### Joint Acceleration Vector
        ###  ### a numpy array of sixe 1*2

        self.jointAccel = np.array([0,0])


        ### both velocity and acceleration are set to zero at the initial state...

        ### SET THE TORQUE OF THE JOINTS
        
        self.jointTorque = np.array([0,0])

        ### CURRENT END EFFECOTR POSITION
        self.endEffectorPosition = None
        self.getForwardKinematics()
        #### CURRENT JACOBIAN MATRIX
        self.Jacobian = None
        self.getJacobian()

        #### Current Mass Matrix
        self.massMatrix = None
        self.getMassMatrix()

        self.coriolisMatrix = None
        self.getCoriolisMatrix()
        #### Torques to conteract gravity
        self.gravityMatix = None
        self.getGravityMatix()
        
        
        
        ############
        ### Stuff for impedance control
        ############
        Kx = [20,20]
        self.matrix_KX = np.diag(np.array(Kx))
        
        Kv = [8,8]
        self.matrix_KV = np.diag(np.array(Kv))


    def getForwardKinematics(self):
        """
        Use standard trig. to calculate the end effector position....
        returns: (x,y) np array :: containing the final end effector position
        returns in the endEffectorPosition variable
        """
        x = self.linklength1 * cos(self.jointAngle1) + self.linklength2 * cos(self.jointAngle1 + self.jointAngle2)

        y = self.linklength1 * sin(self.jointAngle1) + self.linklength2 * sin(self.jointAngle1 + self.jointAngle2)

        self.endEffectorPosition = np.array([x,y])


    
    def getInverseKinematics(self):
        """
        Use Analytical method to return the joint angles vector
        Returns the value in the jointAngle1 and jointAngle2 variable



        """
        ### One issue that wuill be faced is that
        ### there will be mulitiple solutions (two to be precise.)
        ### Need to find out a way to solve this

        x,y = self.endEffectorPosition

        squaredL1 , squaredL2 , = pow(self.linklength1,2) , pow(self.linklength2,2) 
        squaredX , squaredY = pow(x,2), pow(y,2)

        gamma = atan2(y,x)
        beta = acos((squaredL1 + squaredL2 - squaredX - squaredY)/ 2 * self.linklength1 * self.linklength2)
        
        alpha = acos((squaredX + squaredY + squaredL1 - squaredL2) / 2 * self.linklength1 * sqrt(squaredX + squaredY))

        self.jointAngle1 = gamma - alpha
        self.jointAngle2 = np.pi - beta

    def getJacobian(self):
        """
        get the jacobian matrix to relate the end effector velocity to the joint vector velocity.
        """

        J11 = -self.linklength1 * sin(self.jointAngle1) - self.linklength2 * sin(self.jointAngle1 + self.jointAngle2)
        J12 = -self.linklength2 * sin(self.jointAngle1 + self.jointAngle2)
        J21 = self.linklength1 * cos(self.jointAngle1) + self.linklength2 * cos(self.jointAngle1 + self.jointAngle2)
        J22 = self.linklength2 * cos(self.jointAngle1 + self.jointAngle2)

        jacobian = np.array([
            [J11,J12],
            [J21,J22]
        ])
        self.Jacobian = jacobian

    def getMassMatrix(self):
        """
        Calculate the Mass Matrix for Dynamics
        Calculation is done Analytically due to simplicity of the model..
        """
        squaredL1 , squaredL2 , = pow(self.linklength1,2) , pow(self.linklength2,2) 
        M11 = self.linkmass1 * squaredL1 + self.linkmass2 * (squaredL1 + 2 * self.linklength1 * self.linklength1 * cos(self.jointAngle2) + squaredL2 )
        M12 = self.linkmass2 * (self.linklength1 * self.linklength2 * cos(self.jointAngle2) + squaredL2)
        M21 = self.linkmass2 * (self.linklength1 * self.linklength2 * cos(self.jointAngle2) + squaredL2 )
        M22 = self.linkmass2 * squaredL2

        massMatrix = np.array([
            [M11,M12],
            [M21,M22]
        ])

        self.massMatrix = massMatrix
        pass
    

    def getCoriolisMatrix(self):
        """
        sets the value of corilios forces matrix to use in the dynamic model

        """
        

        C11 = -self.linkmass2 * self.linklength1 * self.linklength2 * sin(self.jointAngle2) * (2 * self.jointVelocity[0] * self.jointVelocity[1] + \
                                                                                                                            pow(self.jointVelocity[1],2))

        C12 = self.linkmass2 * self.linklength1 * self.linklength2 * pow (self.jointVelocity[0],2) * sin(self.jointAngle2)

        coriolisMatrix = np.array([C11,C12])

        self.coriolisMatrix = coriolisMatrix

        pass


    def getGravityMatix(self):
        """
        returns the vector containing the gravitational torques.
        """
        
        G11 = (self.linkmass1 + self.linkmass2) * self.linklength1 * self.gravity * cos(self.jointAngle1) + self.linkmass2 * self.gravity \
            * self.linklength2 * cos(self.jointAngle1 + self.jointAngle2)
        
        G21 = self.linkmass2 * self.gravity * self.linklength2 * cos(self.jointAngle1 + self.jointAngle2)

        gravityMatrix = np.array([G11,G21  ])
        self.gravityMatix = gravityMatrix
        pass

if __name__ == "__main__": 

    Arm = Manipulator()
    Arm.jointAngle1,Arm.jointAngle2 = np.pi/6, -np.pi/4 
    Arm.getForwardKinematics()
    Arm.getJacobian()
    Arm.getMassMatrix()
    Arm.getGravityMatix()

    print ("Testing the math")
    print("The current end effector config is: ", Arm.endEffectorPosition)

    print("The current Jacobian is: ", Arm.Jacobian)
    print("The current Mass Matrix is: ", Arm.massMatrix)
    print("The current gravity is: ", Arm.gravityMatix)



    ### assume the force applied by the end effector is:
    Ftip = np.array([1,1])

    ## thus the torque required to generate the force will be:

    ## So the basic force control can be applied as:
    torque = Arm.gravityMatix + np.dot( Arm.Jacobian.T , Ftip)

    print(torque)
    ### Setting up the manip.

    