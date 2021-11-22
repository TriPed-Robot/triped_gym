from math import exp
import gym
import triped_gym
import os
import csv
import numpy as np

# test environment TriPed-v0


def test_step_causality():
    precision = 0.12

    action_consequence_causal = True

    env = gym.make('TriPed-v0')
    env.set_timestep_length(0.1)
    prev_obs = env.reset()

    forward_reference = open(os.path.join(
        'tests', 'data', 'endeffector_coordinates.csv'))
    inverse_reference = open(os.path.join(
        'tests', 'data', 'joint_values.csv'))

    expected_foot_pos = np.loadtxt(forward_reference, delimiter=",")
    action = np.loadtxt(inverse_reference, delimiter=",")

    env._position_control_freq = 600

    for i in range(len(action)):
        obs, _, _, _ = env.step(action[i])
        foot_pos = obs[3:12]

        action_consequence_causal = action_consequence_causal and np.linalg.norm(
            foot_pos-expected_foot_pos[i]) <= precision

    assert action_consequence_causal


test_step_causality()
