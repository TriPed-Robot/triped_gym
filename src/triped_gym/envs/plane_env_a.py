import gym
import numpy as np
import pybullet as p
import pybullet_data
from triped_sim import Triped


class PlaneEnvA(gym.Env):

    def __init__(self):
        """A simple environment in which the feet of the robot are directly controlled
        """
        # initial state of the robot
        self._start_position = [0, 0, 1]
        self._start_orientation = p.getQuaternionFromEuler([0, 0, 0])

        time_step_length = 0.1

        self.done = False

        self.physics_client = p.connect(p.GUI)
        p.setGravity(0, 0, -9.81)
        p.setPhysicsEngineParameter(numSolverIterations=1000)
        p.setTimeStep(time_step_length)

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        planeId = p.loadURDF("plane.urdf")

        self.robot = Triped(
            self._start_position, self._start_orientation)

        # actions follow convention legi_swing_left, legi_swing_right, leg_iextend_joint_ry
        self.action_space = gym.spaces.box.Box(
            low=np.array([-1.2, -0.48869219055, 0,
                          -1.2, -0.48869219055, 0,
                          -1.2, -0.48869219055, 0]),
            high=np.array([0, 0.0034906585, 1.2,
                           0, 0.0034906585, 1.2,
                           0, 0.0034906585, 1.2]))

        # observations made up of chassis orientation, foot positions and ground forces
        observation_range = np.inf * np.ones([3*3+3+3])
        self.observation_space = gym.spaces.box.Box(
            low=-observation_range, high=observation_range)

    def set_timestep_length(self, time_step_length):
        """Sets the timestep between subsequent step function calls.

        Args:
            time_step_length ([type]): The time step when calling the step function
        """
        p.setTimeStep(time_step_length)

    def _get_reward(self):
        # placeholder reward
        return 0

    def _is_standing(self):
        position, orientation = self.robot.get_world_state()
        height = position[2]
        euler_angles = p.getEulerFromQuaternion(
            orientation)

        return (height >= 0.4) and (np.abs(euler_angles[1]) <= np.pi*0.5)

    def _get_observation(self):
        chassis_orientation, foot_positions = self.robot.get_body_state()
        ground_forces = self.robot.get_ground_forces()
        return np.concatenate([chassis_orientation, np.concatenate(foot_positions), ground_forces])

    def _apply_action(self, action):
        new_actuated_state = {'leg_0_swing_left': action[0],
                              'leg_0_extend_joint_ry': action[1],
                              'leg_0_swing_right': action[2],
                              'leg_1_swing_left': action[3],
                              'leg_1_extend_joint_ry': action[4],
                              'leg_1_swing_right': action[5],
                              'leg_2_swing_left': action[6],
                              'leg_2_extend_joint_ry': action[7],
                              'leg_2_swing_right': action[8]}

        self.robot.set_actuated_state(new_actuated_state)

    def step(self, action):

        self._apply_action(action)
        p.stepSimulation()
        observation = self._get_observation()
        reward = self._get_reward()
        self.done = self._is_standing()

        return observation, reward, self.done, {}

    def reset(self):
        "gym function resetting the robot to a initial state"
        self.robot.reset_robot(self._start_position, self._start_orientation)
        observation = self._get_observation()
        return observation

    def close(self):
        p.disconnect(self.physics_client)
