from copy import deepcopy
from typing import Dict
import pybullet as p
import trip_kinematics as trip
from trip_robots.triped import triped
import numpy as np
import os

class TripedBase:

    def __init__(self, urdf_model, start_position, start_orientation,foot_links):
        urdfFlags = p.URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS
        dirname = os.path.dirname(__file__)
        urdf_file = os.path.join(
            dirname, 'robot_descriptions', urdf_model)
        self.urdf = p.loadURDF(urdf_file,
                               start_position, start_orientation,
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

        self._kinematic_model = deepcopy(triped)
        self._inv_kin_solver = [trip.SimpleInvKinSolver(triped, 'leg0_A_LL_Joint_FCS'),
                                trip.SimpleInvKinSolver(
            triped, 'leg1_A_LL_Joint_FCS'),
            trip.SimpleInvKinSolver(triped, 'leg2_A_LL_Joint_FCS')]

        # disable the default velocity motors
        # and set some position control with small force
        # to emulate joint friction/return to a rest pose
        self.max_joint_force = 800*np.ones(p.getNumJoints(self.urdf))
        for joint in range(p.getNumJoints(self.urdf)):
            p.resetJointState(self.urdf, joint, targetValue=0)


        #specified which links are considered the robots feet
        self.foot_links = foot_links


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
            joint_values = np.zeros(len(self._joint_mappings))
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
                                       the desired orientation
        """
        return p.getBasePositionAndOrientation(self.urdf)

    def get_foot_position(self, leg_number):
        """Returns the position of a foot of the triped as calculated using the kinematic model

        Args:
            leg_number ([type]): The leg whose footposition is dessired, numbered from zero to two.

        Returns:
            [type]: A 3 dimensional position.
        """
        return trip.get_translation(trip.forward_kinematics(self._kinematic_model,
                                                            'leg'+str(leg_number)+'_A_LL_Joint_FCS'))

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




class SimplifiedTriped(TripedBase):

    def __init__(self, start_position, start_orientation):
        """The simplified TriPed model forgoes any closed chains by modelling the hip as a single 
           gimbal joint. 
           Although this increases performance it also ads an additional degree of freedom.

        Args:
            start_position ([type]): The world position where the model should be spawned
            start_orientation ([type]): The world orientation at which the model should be spawned
                                        as a quaternion
        """
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

        TripedBase.__init__(self,
                            urdf_model = 'simplified_triped.urdf',
                            start_position = start_position,
                            start_orientation = start_orientation,
                            foot_links = [3,7,11])

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