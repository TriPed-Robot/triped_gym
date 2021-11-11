import time
import pybullet as p
import pybullet_data
import numpy as np

from triped_sim import SimplifiedTriped

"""This short demo showcases the full body control of the TriPed

   Use ctr+alt and left click to move the camera
"""
if __name__ == "__main__":
    physics_client = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.setPhysicsEngineParameter(numSolverIterations=1000)
    planeId = p.loadURDF("plane.urdf")
    startPos = [0, 0, 1]
    start_orientation = p.getQuaternionFromEuler([0, 0, 0])

    robot = SimplifiedTriped(startPos, start_orientation)

    initial_pos = [np.array([0.4*1.2, 0,  -0.6]),
                   np.array([-0.2025*1.2, -0.35074*1.2,  -0.6]),
                   np.array([-0.2025*1.2,  0.35074*1.2,  -0.6])]

    p.setRealTimeSimulation(1)
    for i in range(10000):
        robot.set_body_state(
            [0.3*np.sin(0.1*i), 0.3*np.cos(0.1*i), 0.3*np.sin(0.1*i)], initial_pos)
        print(robot.get_body_state())
        time.sleep(1./200.)
    p.disconnect()
