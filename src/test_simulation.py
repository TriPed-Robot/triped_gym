import time
import numpy as np
import pybullet as p
import pybullet_data


from models.simplified_triped import SimplifiedTriped
from agents.circular_walker import CircularWalker


if __name__ == "__main__":
    physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
    p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
    p.setGravity(0, 0, -9.81)
    p.setPhysicsEngineParameter(numSolverIterations=1000)
    planeId = p.loadURDF("plane.urdf")
    startPos = [0, 0, 1]
    startOrientation = p.getQuaternionFromEuler([0, 0, 0])

    robot = SimplifiedTriped(startPos, startOrientation)

    agent = CircularWalker()
    p.setRealTimeSimulation(1)
    for i in range(10000):
        # p.stepSimulation()

        foot_positions = agent.move_robot()
        for j in [0, 1, 2]:
            robot.set_foot_position(j, foot_positions[j])

        time.sleep(1./200.)
    p.disconnect()
