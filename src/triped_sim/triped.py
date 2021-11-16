from typing import Dict
import pybullet as p
from triped_sim.triped_base import TripedBase


class Triped(TripedBase):

    def __init__(self, start_position, start_orientation):

        TripedBase.__init__(self,
                            urdf_model='triped.urdf',
                            start_position=start_position,
                            start_orientation=start_orientation,
                            foot_links=[3, 7, 11])

        link_name_to_index = {p.getBodyInfo(
            self.urdf)[0].decode('UTF-8'): -1, }
        self._joint_to_name_index = {}
        for id in range(p.getNumJoints(self.urdf)):
            link_name = p.getJointInfo(self.urdf, id)[12].decode('UTF-8')
            joint_name = p.getJointInfo(self.urdf, id)[1].decode('UTF-8')
            link_name_to_index[link_name] = id
            self._joint_to_name_index[joint_name] = id

        # closure equation constraints
        for i in [0, 1, 2]:
            print("created constraint for leg "+str(i))

            p.createConstraint(self.urdf,
                               link_name_to_index['leg_' +
                                                  str(i)+'_drive_module'],
                               self.urdf,
                               link_name_to_index['leg_' +
                                                  str(i)+'_rotZ_drive_module_left'],
                               p.JOINT_POINT2POINT,
                               [0, 0, 0],
                               [0.015-0.0259, -0.029-0.0507, -0.0965-0.0455],
                               [0, 0, 0])

            p.createConstraint(self.urdf,
                               link_name_to_index['leg_' +
                                                  str(i)+'_drive_module'],
                               self.urdf,
                               link_name_to_index['leg_' +
                                                  str(i)+'_rotZ_drive_module_right'],
                               p.JOINT_POINT2POINT,
                               [0, 0, 0],
                               [0.015-0.0259, 0.029-0.0507, -0.0965-0.0455],
                               [0, 0, 0])

    def get_virtual_state(self):
        """Returns the position ofthe virtual state following trip_kinematics conventions

        Returns:
            Dict[str,Dict[str,float]]: A virtual state following trip_kinematics conventions
        """
        return self._kinematic_model.get_virtual_state()

    def set_virtual_state(self, target: Dict[str, Dict[str, float]]):
        """Sets the target position the virtual state following Trip_kinematics conventions.

        Args:
            target (Dict[str, Dict[str, float]]): valid joint states, note that not all states need
                                                 to be supplied.

        Raises:
            ValueError: If the specified joint state is not valid
        """
        if all(key in self._virtual_state_shape.keys()for key in target.keys()):
            self._kinematic_model.set_virtual_state(target)
            virtual_state = self._kinematic_model.get_actuated_state()
            self.set_actuated_state(virtual_state)
        else:
            raise ValueError('Error: One or more keys are not part of the triped state. ' +
                             'correct keys are: '+str(self._virtual_state_shape.keys()))

    def get_actuated_state(self):
        """Returns the position of the robots actuated joints.

        Returns:
            Dict[str,float]: A actuated state following trip_kinematics conventions
        """
        return self._kinematic_model.get_actuated_state()

    def set_actuated_state(self, target):
        if all(key in self._actuated_state_shape.keys()for key in target.keys()):
            for actuator_name, actuator_target in target.items():
                joint_number = self._joint_to_name_index[actuator_name]
                p.setJointMotorControl2(self.urdf, joint_number, p.POSITION_CONTROL,
                                        force=self.max_joint_force[joint_number],
                                        targetPosition=actuator_target)
            self._kinematic_model.set_actuated_state(target)
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

        # provide the internal closure equation with a tip to stay in the same kinematic mode
        current_actuated_state = self._kinematic_model.get_actuated_state()
        mapping_arg = {}
        for i in [0, 1, 2]:
            left_actuator = 'leg_'+str(leg_number) + '_swing_left'
            right_actuator = 'leg_'+str(leg_number) + '_swing_right'
            tip = {left_actuator: current_actuated_state[left_actuator],
                   right_actuator: current_actuated_state[right_actuator]}
            mapping_arg['leg_'+str(leg_number)+'_closed_chain'] = [tip]

        solution = self._inv_kin_solver[leg_number].solve_actuated(
            target, initial_tip=self._kinematic_model.get_virtual_state(), mapping_argument=mapping_arg)
        self.set_actuated_state(solution)
