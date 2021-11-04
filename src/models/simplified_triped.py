from copy import deepcopy
from typing import Dict
import pybullet as p
import trip_kinematics as trip
from trip_robots.triped import triped
import numpy as np
import os


class SimplifiedTriped:

    def __init__(self, startPos, startOrientation):
        urdfFlags = p.URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS
        dirname = os.path.dirname(__file__)
        urdf_file = os.path.join(dirname, 'meshes', 'TriPed.urdf')
        self.urdf = p.loadURDF(urdf_file,
                               startPos, startOrientation,
                               flags=urdfFlags,
                               useFixedBase=False)

        # possible virtual state joint_targets
        self._virtual_state_shape = {'leg0_gimbal_joint': {'rx': 0, 'ry': 0, 'rz': 0},
                                     'leg0_extend_joint': {'ry': 0},
                                     'leg1_gimbal_joint': {'rx': 0, 'ry': 0, 'rz': 0},
                                     'leg1_extend_joint': {'ry': 0},
                                     'leg2_gimbal_joint': {'rx': 0, 'ry': 0, 'rz': 0},
                                     'leg2_extend_joint': {'ry': 0}}

        # possible actuated state joint_targets
        self._actuated_state_shape = {'leg0_swing_left': 0,
                                      'leg0_swing_right': 0,
                                      'leg0_extend_joint_ry': 0,
                                      'leg1_swing_left': 0,
                                      'leg1_swing_right': 0,
                                      'leg1_extend_joint_ry': 0,
                                      'leg2_swing_left': 0,
                                      'leg2_swing_right': 0,
                                      'leg2_extend_joint_ry': 0}

        self._joint_mappings = [('leg0_gimbal_joint', 'rx'),
                                ('leg0_gimbal_joint', 'ry'),
                                ('leg0_gimbal_joint', 'rz'),
                                ('leg0_extend_joint', 'ry'),
                                ('leg1_gimbal_joint', 'rx'),
                                ('leg1_gimbal_joint', 'ry'),
                                ('leg1_gimbal_joint', 'rz'),
                                ('leg1_extend_joint', 'ry'),
                                ('leg2_gimbal_joint', 'rx'),
                                ('leg2_gimbal_joint', 'ry'),
                                ('leg2_gimbal_joint', 'rz'),
                                ('leg2_extend_joint', 'ry'), ]

        self._kinematic_model = deepcopy(triped)
        self._inv_kin_solver = [trip.SimpleInvKinSolver(triped, 'leg0_A_LL_Joint_FCS'),
                                trip.SimpleInvKinSolver(
            triped, 'leg1_A_LL_Joint_FCS'),
            trip.SimpleInvKinSolver(triped, 'leg2_A_LL_Joint_FCS')]

        # disable the default velocity motors
        # and set some position control with small force
        #  to emulate joint friction/return to a rest pose
        self.max_joint_force = 800*np.ones(p.getNumJoints(self.urdf))
        for joint in range(p.getNumJoints(self.urdf)):
            p.resetJointState(self.urdf, joint, targetValue=0)

    def get_virtual_state(self):
        """Returns the position of each joint following trip_kinematics conventions

        Returns:
            Dict[str,Dict[str,float]]: A virtual state following trip_kinematics conventions
        """
        virtual_state = {}
        for joint in range(p.getNumJoints(self.urdf)):
            dict_values = self._joint_mappings[joint]
            if dict_values[0] not in virtual_state.keys():
                virtual_state[dict_values[0]] = {}
            virtual_state[dict_values[0]][dict_values[1]
                                          ] = p.getJointState(self.urdf, joint)[0]
        return virtual_state

    def set_virtual_state(self, target: Dict[str, Dict[str, float]]):
        """Sets the target position of each joint following Trip_kinematics conventions.
        The maximum force of each joint is set according to the max_joint_force class attribute.

        Args:
            target (Dict[str, Dict[str, float]]): valid joint states, note that not all states need
                                                 to be supplied.

        Raises:
            ValueError: If the specified joint state is not valid
        """

        if all(key in self._virtual_state_shape.keys()for key in target.keys()):
            for joint, joint_state in target.items():
                for key, value in joint_state.items():
                    state_tuple = (joint, key)
                    joint_number = self._joint_mappings.index(state_tuple)
                    p.setJointMotorControl2(self.urdf, joint_number, p.POSITION_CONTROL,
                                            force=self.max_joint_force[joint_number],
                                            targetPosition=value)
            self._kinematic_model.set_virtual_state(target)
        else:
            raise ValueError('Error: One or more keys are not part of the triped state. ' +
                             'correct keys are: '+str(self._virtual_state_shape.keys()))

    def get_actuated_state(self):
        """Returns the position of the robots actuated state as if it was controlled by its 
            swing motors. 


        Returns:
            Dict[str,float]: A actuated state following trip_kinematics conventions
        """
        return self._kinematic_model.get_actuated_state()

    def set_actuated_state(self, target):
        if all(key in self._actuated_state_shape.keys()for key in target.keys()):
            self._kinematic_model.set_actuated_state(target)
            virtual_state = self._kinematic_model.get_virtual_state()
            self.set_virtual_state(virtual_state)
        else:
            raise ValueError(
                "Error: One or more keys are not part of the actuated state. correct keys are: "
                + str(self._actuated_state_shape.keys()))

    def get_foot_position(self, leg_number):
        """Returns the position of a foot of the triped

        Args:
            leg_number ([type]): The leg whose footposition is dessired, numbered from zero to two.

        Returns:
            [type]: A 3 dimensional position.
        """
        return trip.get_translation(trip.forward_kinematics(self._kinematic_model,
                                                            'leg'+str(leg_number)+'_A_LL_Joint_FCS'))

    def set_foot_position(self, leg_number, target):
        """Allows the position control of a leg of the TriPed.
        Kinematic calculations are performed using TriP, although pybullet is also capable
        of computing inverse kinematics.

        Args:
            leg_number ([type]): The leg which is to be controlled, numbered from zero to two.
            target ([type]): A 3 dimensional target position.
        """
        solution = self._inv_kin_solver[leg_number].solve_virtual(target)
        self.set_virtual_state(solution)
