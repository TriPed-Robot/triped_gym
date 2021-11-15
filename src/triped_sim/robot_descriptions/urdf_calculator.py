"""
This script contains helper functions to setup the URDF of the full triped model
"""
import pybullet as p
import os
from math import pi, sin, cos

def calculate_swing_joints(number):
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
    start_position = [0, 0, 1]
    start_orientation = p.getQuaternionFromEuler([0, 0, 0])
    urdfFlags = p.URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS
    dirname = os.path.dirname(__file__)
    urdf_file = os.path.join(dirname, "triped.urdf")
    urdf = p.loadURDF(urdf_file,
                      start_position, start_orientation,
                      flags=urdfFlags,
                      useFixedBase=False)

    calculate_swing_joints(0)
    calculate_swing_joints(1)
    calculate_swing_joints(2)

    p.setRealTimeSimulation(1)

    while True:
        a=0
    p.disconnect()
