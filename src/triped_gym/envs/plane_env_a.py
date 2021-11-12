import gym
import numpy as np
import pybullet as p
import pybullet_data
from triped_sim import SimplifiedTriped


class PlaneEnvA(gym.Env):

    def __init__(self):
        """A simple environment in which the feet of the robot are directly controlled
        """
        self.physics_client = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.setPhysicsEngineParameter(numSolverIterations=1000)
        planeId = p.loadURDF("plane.urdf")
        self.start_position = [0, 0, 1]
        self.start_orientation = p.getQuaternionFromEuler([0, 0, 0])

        self.robot = SimplifiedTriped(
            self.start_position, self.start_orientation)

        # actions follow convention legi_swing_left, legi_swing_right, leg_iextend_joint_ry
        self.action_space = gym.spaces.box.Box(
            low=np.array([-0.523599, -0.523599, -0.523599,
                          -0.48869219055, -0.48869219055, -0.48869219055,
                          -0.523599, -0.523599, -0.523599]),
            high=np.array([0.523599, 0.523599, 0.523599,
                           0.0034906585, 0.0034906585, 0.0034906585,
                           0.523599, 0.523599, 0.523599]))

        # observations made up of chassis orientation, foot positions and ground forces
        observation_range = np.inf * np.ones([3*3+3+3])
        self.observation_space = gym.spaces.box.Box(
            low=-observation_range, high=observation_range)

        self.done = False

    def _get_reward(self):
        # placeholder reward
        return 0

    def _get_observation(self):
        chassis_orientation, foot_positions = self.robot.get_body_state()
        ground_forces = self.robot.get_ground_forces()
        return np.concatenate([chassis_orientation, np.concatenate(foot_positions), ground_forces])

    def _apply_action(self, action):
        new_actuated_state = {'leg0_swing_left': action[0],
                              'leg1_swing_left': action[1],
                              'leg2_swing_left': action[2],
                              'leg0_extend_joint_ry': action[3],
                              'leg1_extend_joint_ry': action[4],
                              'leg2_extend_joint_ry': action[5],
                              'leg0_swing_right': action[6],
                              'leg1_swing_right': action[7],
                              'leg2_swing_right': action[8]}

        self.robot.set_actuated_state(new_actuated_state)

    def step(self, action):

        self._apply_action(action)
        p.stepSimulation()
        observation = self._get_observation()
        reward = self._get_reward()

        return observation, reward, self.done, {}

    def reset(self):
        "gym function resetting the robot to a initial state"
        self.robot.reset_robot(self.start_position, self.start_orientation)
        observation = self._get_observation()
        return observation

    def close(self):
        p.disconnect(self.physics_client)
