"""
This script contains helper functions to debug the URDF of the full triped model
"""
import pybullet as p
import os
from math import pi, sin, cos
import time
from math import sin

def calculate_swing_joints(number):
    """calculates the origin of the swing joint as a function of the leg number

    Args:
        number ([type]): the number of the leg
    """
    print("the joint configuration of the revolut joints of leg "+str(number)+" is:")
    DISTANCE = 0.150237035756833
    left_swing_angle = (-21.4745-(120*number))*pi/180
    right_swing_angle = (21.4745-(120*number))*pi/180

    left_origin = str(DISTANCE*cos(left_swing_angle))+" "+str(DISTANCE*sin(left_swing_angle))+" -0.051"
    right_origin = str(DISTANCE*cos(right_swing_angle))+" "+str(DISTANCE*sin(right_swing_angle))+" -0.051"
    print("swing_left: rpy = 0.0 0.0 "+str(left_swing_angle)+", xyz = "+left_origin)
    print("swing_right: rpy = 0.0 0.0 "+str(right_swing_angle)+", xyz = "+right_origin)

if __name__ == "__main__":
    physics_client = p.connect(p.GUI)
    p.setPhysicsEngineParameter(numSolverIterations=1000)
    start_position = [0, 0, 1]
    start_orientation = p.getQuaternionFromEuler([0, 0, 0])
    dirname = os.path.dirname(__file__)
    urdf_file = os.path.join(dirname, "triped.urdf")
    urdf = p.loadURDF(urdf_file,
                      start_position, start_orientation)
    
    # find the link indexes to create the constraints of the closed chain
    
    link_name_to_index = {p.getBodyInfo(urdf)[0].decode('UTF-8'):-1,}  
    for id in range(p.getNumJoints(urdf)):
	    name = p.getJointInfo(urdf, id)[12].decode('UTF-8')
	    link_name_to_index[name] = id
    #print(link_name_to_index)
    

    
    joint_number = p.getNumJoints(urdf)
    for i in range(joint_number):
        '''
        p.setJointMotorControl2(urdf, i, p.POSITION_CONTROL,
                            force=80000,
                            targetPosition=1)
        '''
        print(p.getJointInfo(urdf,i))
   
    p.createConstraint(urdf, 
                       -1,-1,-1,
                        p.JOINT_FIXED,
                        [0, 0, 0],
                        [0,0,0], 
                        [0, 0, 1])


    #leg_0 drive module
    
    
    p.createConstraint(urdf, 
                       link_name_to_index['leg_0_drive_module'], 
                       urdf,
                       link_name_to_index['leg_0_rotZ_drive_module_left'],
                        p.JOINT_POINT2POINT,
                        [0, 0, 0],
                        [-0.015-0.0259 , -0.029-0.0507, -0.0965-0.0455], 
                        [0, 0, 0])

    p.createConstraint(urdf, 
                       link_name_to_index['leg_0_drive_module'], 
                       urdf,
                       link_name_to_index['leg_0_rotZ_drive_module_right'],
                        p.JOINT_POINT2POINT,
                        [0, 0, 0],
                        [-0.015-0.0259 , 0.029-0.0507, -0.0965-0.0455], 
                        [0, 0, 0])
    
    

    #calculate_swing_joints(0)
    #calculate_swing_joints(1)
    #calculate_swing_joints(2)

    p.setRealTimeSimulation(1)

    joint_number = p.getNumJoints(urdf)
    for i in range(10000):
        '''
        p.setJointMotorControl2(urdf, 0, p.POSITION_CONTROL,
                            force=80000,
                            targetPosition=-0.4)
        p.setJointMotorControl2(urdf, 21, p.POSITION_CONTROL,
                            force=80000,
                            targetPosition=0.4)
        '''
        time.sleep(1./200.)
    p.disconnect()
