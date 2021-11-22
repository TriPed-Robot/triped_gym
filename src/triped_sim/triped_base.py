import os
import numpy as np
import pybullet as p

from copy import deepcopy
from typing import Dict

from trip_robots.triped import triped
import trip_kinematics as trip


class TripedBase:

    def __init__(self, urdf_model, start_position, start_orientation, foot_links):
        urdf_flags = p.URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS
        dirname = os.path.dirname(__file__)
        urdf_file = os.path.join(
            dirname, 'robot_descriptions', urdf_model)
        self.urdf = p.loadURDF(urdf_file,
                               start_position, start_orientation,
                               flags=urdf_flags,
                               useFixedBase=False)

        # possible virtual state joint_targets
        self._virtual_state_shape = {'leg_0_gimbal_joint': {'rx': 0, 'ry': 0, 'rz': 0},
                                     'leg_0_extend_joint': {'ry': 0},
                                     'leg_1_gimbal_joint': {'rx': 0, 'ry': 0, 'rz': 0},
                                     'leg_1_extend_joint': {'ry': 0},
                                     'leg_2_gimbal_joint': {'rx': 0, 'ry': 0, 'rz': 0},
                                     'leg_2_extend_joint': {'ry': 0}}

        # possible actuated state joint_targets
        self._actuated_state_shape = {'leg_0_swing_left': 0,
                                      'leg_0_swing_right': 0,
                                      'leg_0_extend_joint_ry': 0,
                                      'leg_1_swing_left': 0,
                                      'leg_1_swing_right': 0,
                                      'leg_1_extend_joint_ry': 0,
                                      'leg_2_swing_left': 0,
                                      'leg_2_swing_right': 0,
                                      'leg_2_extend_joint_ry': 0}

        self._kinematic_model = deepcopy(triped)
        self._inv_kin_solver = [trip.SimpleInvKinSolver(triped, 'leg_0_A_LL_Joint_FCS'),
                                trip.SimpleInvKinSolver(
            triped, 'leg_1_A_LL_Joint_FCS'),
            trip.SimpleInvKinSolver(triped, 'leg_2_A_LL_Joint_FCS')]

        # disable the default velocity motors
        # and set some position control with small force
        # to emulate joint friction/return to a rest pose
        self.max_joint_force = 800*np.ones(p.getNumJoints(self.urdf))
        for joint in range(p.getNumJoints(self.urdf)):
            p.resetJointState(self.urdf, joint, targetValue=0)

        # specified which links are considered the robots feet
        self.foot_links = foot_links

    def get_virtual_state(self):
        """ [Placeholder]
        Returns the position ofthe virtual state following trip_kinematics conventions

        Returns:
            Dict[str,Dict[str,float]]: A virtual state following trip_kinematics conventions
        """
        pass

    def set_virtual_state(self, target: Dict[str, Dict[str, float]]):
        """ [Placeholder]
        Sets the target position the virtual state following Trip_kinematics conventions.

        Args:
            target (Dict[str, Dict[str, float]]): valid joint states, note that not all states need
                                                 to be supplied.

        Raises:
            ValueError: If the specified joint state is not valid
        """
        pass

    def get_actuated_state(self):
        """ [Placeholder]
        Returns the position of the robots actuated joints.

        Returns:
            Dict[str,float]: A actuated state following trip_kinematics conventions
        """
        pass

    def set_foot_position(self, leg_number, target):
        """ [Placeholder]
        Allows the position control of a leg of the TriPed.
        Kinematic calculations are performed using TriP, although pybullet is also capable
        of computing inverse kinematics.

        Note that TriPs inverse kinematic solevers can sometimes be unstable, so use this interface
        with caution.

        Args:
            leg_number ([type]): The leg which is to be controlled, numbered from zero to two.
            target ([type]): A 3 dimensional target position.
        """
        pass

    def reset_robot(self, start_position, start_orientation, joint_values=None):
        """resets the robots joints to 0 and the base to a specified position and orientation

        Args:
            start_position ([type]): a 3 dimensional position
            start_orientation ([type]): a 4 dimensional quaternion representing
                                       the desired orientation
        """
        p.resetBasePositionAndOrientation(
            self.urdf, start_position, start_orientation)

        if joint_values is None:
            joint_values = np.zeros(p.getNumJoints(self.urdf))
        for joint in range(p.getNumJoints(self.urdf)):
            p.resetJointState(self.urdf, joint,
                              targetValue=joint_values[joint])

    def set_world_state(self, start_position, start_orientation):
        """Resets the robots base to a specified position and orientation

        Args:
            start_position ([type]): a 3 dimensional position
            start_orientation ([type]): a 4 dimensional quaternion representing
                                       the desired orientation
        """
        p.resetBasePositionAndOrientation(
            self.urdf, start_position, start_orientation)

    def get_world_state(self):
        """Returns the position and orientation of the robot relative to the world

        Returns:
            [type]: a 3 dimensional position and a 4 dimensional quaternion representing
                                       the current orientation
        """
        return p.getBasePositionAndOrientation(self.urdf)

    def get_foot_position(self, leg_number):
        """Returns the position of a foot of the triped as calculated using the kinematic model

        Args:
            leg_number ([type]): The leg whose footposition is dessired, numbered from zero to two.

        Returns:
            [type]: A 3 dimensional position.
        """
        return trip.get_translation(
            trip.forward_kinematics(self._kinematic_model,
                                    'leg_'+str(leg_number)+'_A_LL_Joint_FCS'))

    def set_body_state(self, orientation, leg_targets):
        """Allows the control of a robots orientation given the position of all three legs relative
           to a base with orientation [0,0,0]

        Args:
            orientation ([type]): Euler angles in roll pitch yaw convention
            leg_targets ([type]): A list containing the three foot positions ordered from leg 0 to 2
        """
        base_to_floor = trip.Transformation('body_orientation',
                                            {'rx': orientation[0],
                                             'ry': orientation[1],
                                             'rz': orientation[2]})
        for leg in [0, 1, 2]:
            target = leg_targets[leg]
            foot_to_base = trip.Transformation('foot_pos',
                                               {'tx': target[0], 'ty': target[1], 'tz': target[2]})
            body_target = base_to_floor.get_transformation_matrix() \
                @ foot_to_base.get_transformation_matrix()
            self.set_foot_position(leg, trip.get_translation(body_target))

    def get_body_state(self):
        """Returns the body state of a robot comprised of leg positions relative to the base and
           the body orientation

           Important: Note that the leg positions are different
           from the once given to set_body_target!
           This because they are relative to a base with [0,0,0].

        Returns:
            [type]: Euler angles in roll pitch yaw convention
                    and a list of 3 dimensional foot positions
            [type]: The foot positions [x,y,z] relative to the chassis
        """
        _, orientation = p.getBasePositionAndOrientation(self.urdf)
        leg_state = [self.get_foot_position(leg) for leg in [0, 1, 2]]
        euler_orientation = p.getEulerFromQuaternion(orientation)
        return np.array(euler_orientation), leg_state

    def get_ground_forces(self):
        """Returns the normal forces experienced by each leg as a result of ground contact

        Returns:
            [type]: a three dimensional vector of the ground normal forces
        """
        contact_forces = np.zeros(3)
        if len(p.getContactPoints(self.urdf)) > 0:
            contact_points = p.getContactPoints(self.urdf)
            for contact in contact_points:
                if contact[3] == self.foot_links[0]:
                    contact_forces[0] = contact[9]
                elif contact[3] == self.foot_links[1]:
                    contact_forces[1] = contact[9]
                elif contact[3] == self.foot_links[2]:
                    contact_forces[2] = contact[9]
        return contact_forces
