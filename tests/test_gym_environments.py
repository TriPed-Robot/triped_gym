import os
import gym
import numpy as np
import pybullet as p
import triped_gym

# test environment TriPed-v0


def test_step_causality():
    """This function tests wheter actions have the desired 
       causal effect on the foot position observation.
    """
    precision = 0.12

    action_consequence_causal = True

    env = gym.make('TriPed-v0')
    env.reset()

    forward_reference = open(os.path.join(
        'tests', 'data', 'endeffector_coordinates.csv'))
    inverse_reference = open(os.path.join(
        'tests', 'data', 'joint_values.csv'))

    expected_foot_pos = np.loadtxt(forward_reference, delimiter=",")
    action = np.loadtxt(inverse_reference, delimiter=",")

    for i in range(len(action)):
        obs, _, _, _ = env.step(action[i])
        foot_pos = obs[3:12]

        action_consequence_causal = action_consequence_causal and np.linalg.norm(
            foot_pos-expected_foot_pos[i]) <= precision

    env.close()
    assert action_consequence_causal
