import time
import pybullet as p
import pybullet_data

from triped_sim import SimplifiedTriped
from circular_walker import CircularWalker

"""This short demo uses the CircularWalker gait pattern generator
   to let the TriPed walk accross a plane.

   Use ctr+alt and left click to move the camera
"""
if __name__ == "__main__":
    physicsClient = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.setPhysicsEngineParameter(numSolverIterations=1000)
    planeId = p.loadURDF("plane.urdf")
    startPos = [0, 0, 1]
    startOrientation = p.getQuaternionFromEuler([0, 0, 0])

    robot = SimplifiedTriped(startPos, startOrientation)

    agent = CircularWalker()
    p.setRealTimeSimulation(1)
    for i in range(10000):
        foot_positions = agent.move_robot()
        for j in [0, 1, 2]:
            robot.set_foot_position(j, foot_positions[j])

        time.sleep(1./200.)
    p.disconnect()
