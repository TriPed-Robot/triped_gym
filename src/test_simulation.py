import pybullet as p
import pybullet_data
import numpy as np
from models.simplified_triped import SimplifiedTriped
import time

if __name__ == "__main__":
    physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
    p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
    p.setGravity(0, 0, -9.81)
    p.setPhysicsEngineParameter(numSolverIterations=1000)
    planeId = p.loadURDF("plane.urdf")
    startPos = [0, 0, 1]
    startOrientation = p.getQuaternionFromEuler([0, 0, 0])

    robot = SimplifiedTriped(startPos, startOrientation)

    initial_pos_0 = np.array([0.4*1.2, 0,  -0.6])
    initial_pos_1 = np.array([-0.2025*1.2, -0.35074*1.2,  -0.6])
    initial_pos_2 = np.array([-0.2025*1.2,  0.35074*1.2,  -0.6])

    poses = [initial_pos_0, initial_pos_1, initial_pos_2]

    for i in range(10000):
        p.stepSimulation()

        actuated_state = {'leg0_swing_left': -0.3,
                          'leg0_swing_right': 0.3,
                          'leg0_extend_joint_ry': -0.4,
                          'leg1_swing_left': 0,
                          'leg1_swing_right': 0,
                          'leg1_extend_joint_ry': 0,
                          'leg2_swing_left': 0,
                          'leg2_swing_right': 0,
                          'leg2_extend_joint_ry': 0}

        robot.set_actuated_state(actuated_state)
        '''
        for leg_number in [0, 1, 2]:
            # + 0.3 * (1-max(1, i/1000))
            pos_with_height = poses[leg_number] - 0.1 * \
                np.array([np.cos(i/200), np.sin(i/200), 0])
            robot.set_foot_position(leg_number, pos_with_height)
            current_foot = robot.get_foot_position(leg_number)
        '''
        time.sleep(1./240.)
    p.disconnect()
