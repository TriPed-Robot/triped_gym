import gym
import pybullet as p
import pybullet_data
from triped_sim import SimplifiedTriped


class PlaneEnvF(gym.Env):

    def __init__(self):
        """A simple environment in which the feet of the robot are directly controlled
        """
        physicsClient = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.setPhysicsEngineParameter(numSolverIterations=1000)
        planeId = p.loadURDF("plane.urdf")
        self.startPos = [0, 0, 1]
        self.startOrientation = p.getQuaternionFromEuler([0, 0, 0])

        self.robot = SimplifiedTriped(self.startPos, self.startOrientation)

    def reset(self):
        self.robot.reset_robot(self.start_pos, self.startOrientation)
