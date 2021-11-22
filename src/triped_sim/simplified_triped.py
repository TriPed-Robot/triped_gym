from typing import Dict
import pybullet as p
from triped_sim.triped_base import TripedBase


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
        self._joint_mappings = [('leg_0_gimbal_joint', 'rx'),
                                ('leg_0_gimbal_joint', 'ry'),
                                ('leg_0_gimbal_joint', 'rz'),
                                ('leg_0_extend_joint', 'ry'),
                                ('leg_1_gimbal_joint', 'rx'),
                                ('leg_1_gimbal_joint', 'ry'),
                                ('leg_1_gimbal_joint', 'rz'),
                                ('leg_1_extend_joint', 'ry'),
                                ('leg_2_gimbal_joint', 'rx'),
                                ('leg_2_gimbal_joint', 'ry'),
                                ('leg_2_gimbal_joint', 'rz'),
                                ('leg_2_extend_joint', 'ry'), ]

        TripedBase.__init__(self,
                            urdf_model='simplified_triped.urdf',
                            start_position=start_position,
                            start_orientation=start_orientation,
                            foot_links=[3, 7, 11])

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
        """Sets the position of the actuated joints

        Args:
            target ([type]): valid joint states, note that not all states need
                             to be supplied

        Raises:
            ValueError: If the specified joint state is not valid
        """
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
