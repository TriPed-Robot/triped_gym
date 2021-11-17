import pybullet as p
import os
import csv
from triped_sim import Triped
import numpy as np


def test_actuated_interface():
    precision = 0.02

    physics_client = p.connect(p.DIRECT)
    p.setPhysicsEngineParameter(numSolverIterations=1000)
    start_orientation = p.getQuaternionFromEuler([0, 0, 0])
    robot = Triped([0, 0, 0], start_orientation)

    p.createConstraint(robot.urdf,
                       -1, -1, -1,
                       p.JOINT_FIXED,
                       [0, 0, 0],
                       [0, 0, 0],
                       [0, 0, 0])

    inverse_reference = os.path.join(
        'tests', 'data', 'joint_values.csv')

    input_t1 = []
    input_e = []
    input_t2 = []

    with open(inverse_reference, newline='') as csvfile:
        reader = csv.reader(csvfile, delimiter=',')
        for row in reader:
            input_t1.append(float(row[0]))
            input_e.append(float(row[1]))
            input_t2.append(float(row[2]))

    for i in range(len(input_t1)):
        within_precision = True
        target_state = {}
        # set the desired actuated state
        for leg_number in [0, 1, 2]:
            target_state['leg_'+str(leg_number) +
                         '_swing_left'] = input_t1[i]
            target_state['leg_'+str(leg_number) +
                         '_extend_joint_ry'] = input_e[i]
            target_state['leg_'+str(leg_number) +
                         '_swing_right'] = input_t2[i]
        robot.set_actuated_state(target_state)

        # update the simulation multiple times.
        for i in range(100):
            p.stepSimulation()

        # check if the correct actuated state has been reached
        leg_row = robot.get_actuated_state()
        for leg_number in [0, 1, 2]:
            difference = np.abs([leg_row['leg_'+str(leg_number)+'_swing_left'] -
                                 target_state['leg_' +
                                              str(leg_number)+'_swing_left'],
                                 leg_row['leg_'+str(leg_number)+'_extend_joint_ry'] -
                                 target_state['leg_' +
                                              str(leg_number)+'_extend_joint_ry'],
                                 leg_row['leg_'+str(leg_number)+'_swing_right'] -
                                 target_state['leg_'+str(leg_number)+'_swing_right']])
            within_precision = within_precision and all(
                difference <= precision)

    p.disconnect()
    assert within_precision


def test_foot_interface():
    physics_client = p.connect(p.DIRECT)
    p.setPhysicsEngineParameter(numSolverIterations=1000)
    start_orientation = p.getQuaternionFromEuler([0, 0, 0])
    robot = Triped([0, 0, 0], start_orientation)

    p.createConstraint(robot.urdf,
                       -1, -1, -1,
                       p.JOINT_FIXED,
                       [0, 0, 0],
                       [0, 0, 0],
                       [0, 0, 1])

    forward_reference = os.path.join(
        'tests', 'data', 'endeffector_coordinates.csv')
    inverse_reference = os.path.join(
        'tests', 'data', 'joint_values.csv')

    input_x = []
    input_y = []
    input_z = []

    input_t1_tip = []
    input_t2_tip = []

    reference = []

    calculated = []
    tip = {'swing_left': 0, 'swing_right': 0, 'ry': 0}

    with open(forward_reference, newline='') as csvfile:
        reader = csv.reader(csvfile, delimiter=',')
        for row in reader:
            input_x.append([float(row[0]), float(row[3]), float(row[6])])
            input_y.append([float(row[1]), float(row[4]), float(row[7])])
            input_z.append([float(row[2]), float(row[5]), float(row[8])])

    with open(inverse_reference, newline='') as csvfile:
        reader = csv.reader(csvfile, delimiter=',')
        for row in reader:
            reference.append(np.array([float(row[i])
                             for i in range(len(row))]))
            input_t1_tip.append([float(row[0]), float(row[3]), float(row[6])])
            input_t2_tip.append([float(row[2]), float(row[5]), float(row[8])])

    for i in range(len(input_x)):
        row = []
        # set the desired foot positions
        for leg_number in [0, 1, 2]:
            tip['leg_'+str(leg_number) +
                '_swing_left'] = input_t1_tip[i][leg_number]
            tip['leg_'+str(leg_number) +
                '_swing_right'] = input_t2_tip[i][leg_number]
            mapping_argument = {'leg_'+str(leg_number)+'_closed_chain': [tip]}
            robot._inv_kin_solver[leg_number]._robot.pass_group_arg_v_to_a(
                mapping_argument)
            robot.set_foot_position(leg_number,
                                    [input_x[i][leg_number],
                                     input_y[i][leg_number],
                                     input_z[i][leg_number]])

        # update the simulation multiple times.
        for i in range(20):
            p.stepSimulation()

        # check if the correct actuated state has been reached
        leg_row = robot.get_actuated_state()
        for leg_number in [0, 1, 2]:
            row.extend([leg_row['leg_'+str(leg_number)+'_swing_left'],
                       leg_row['leg_'+str(leg_number)+'_extend_joint_ry'],
                       leg_row['leg_'+str(leg_number)+'_swing_right']])
        calculated.append(row)

    p.disconnect()

    # precision is tied to the number of control steps the simulation can perform
    precision = 0.2
    sample_results = [(np.abs(reference[i]-calculated[i]) <
                      precision).all() for i in range(1, len(reference))]
    assert all(sample_results)
