from typing import Dict
import pybullet as p
import time
import pybullet_data
import trip_kinematics as trip
from trip_robots.triped import triped
import numpy as np


class SimplifiedTriped:

    def __init__(self, startPos, startOrientation):
        urdfFlags = p.URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS
        self.urdf = p.loadURDF("meshes\TriPed.urdf",
                               startPos, startOrientation,
                               flags=urdfFlags,
                               useFixedBase=False)

        # states follow trip_kinematics definition
        self.joint_targets = {'leg0_gimbal_joint': {'rx': 0, 'ry': 0, 'rz': 0},
                              'leg0_extend_joint': {'ry': 0},
                              'leg1_gimbal_joint': {'rx': 0, 'ry': 0, 'rz': 0},
                              'leg1_extend_joint': {'ry': 0},
                              'leg2_gimbal_joint': {'rx': 0, 'ry': 0, 'rz': 0},
                              'leg2_extend_joint': {'ry': 0}}

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

        # disable the default velocity motors
        # and set some position control with small force
        #  to emulate joint friction/return to a rest pose
        self.max_joint_force = 10000*np.ones(p.getNumJoints(self.urdf))
        for joint in range(p.getNumJoints(self.urdf)):
            p.resetJointState(self.urdf, joint, targetValue=0)

    def get_virtual_state(self):
        """Returns the position of each joint following Trip_kinematics conventions

        Returns:
            Fict[str,Dict[str,float]]: [description]
        """
        virtual_state = {}
        for joint in range(p.getNumJoints(self.urdf)):
            dict_values = self._joint_mappings[joint]
            if dict_values[0] not in virtual_state.keys():
                virtual_state[dict_values[0]] = {}
            virtual_state[dict_values[0]][dict_values[1]
                                          ] = p.getJointState(self.urdf, joint)[0]
        return virtual_state

    def set_virtual_state(self, state: Dict[str, Dict[str, float]]):
        """Sets the target position of each joint following Trip_kinematics conventions.
        The maximum force of each joint is set according to the max_joint_force class attribute. 

        Args:
            state (Dict[str, Dict[str, float]]): valid joint states, note that not all states need
                                                 to be supplied.

        Raises:
            ValueError: If the specified joint state is not valid
        """

        if all(key in self.joint_targets.keys()for key in state.keys()):
            for joint, joint_state in state.items():
                for key, value in joint_state.items():
                    state_tuple = (joint, key)
                    joint_number = self._joint_mappings.index(state_tuple)
                    p.setJointMotorControl2(self.urdf, joint_number, p.POSITION_CONTROL,
                                            force=self.max_joint_force[joint_number],
                                            targetPosition=value)
        else:
            raise ValueError('Error: One or more keys are not part of the triped state. ' +
                             'correct keys are: '+str(self.joint_targets.keys()))


if __name__ == "__main__":
    physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
    p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
    p.setGravity(0, 0, -9.81)
    p.setPhysicsEngineParameter(numSolverIterations=1000)
    planeId = p.loadURDF("plane.urdf")
    startPos = [0, 0, 1.5]
    startOrientation = p.getQuaternionFromEuler([0, 0, 0])

    robot = SimplifiedTriped(startPos, startOrientation)

    inv_kin_solver = [trip.SimpleInvKinSolver(triped, 'leg0_A_LL_Joint_FCS', update_robot=False),
                      trip.SimpleInvKinSolver(
                          triped, 'leg1_A_LL_Joint_FCS', update_robot=False),
                      trip.SimpleInvKinSolver(triped, 'leg2_A_LL_Joint_FCS', update_robot=False)]

    initial_pos_0 = np.array([0.4, 0,  -0.6])
    initial_pos_1 = np.array([-0.2025, -0.35074,  -0.6])
    initial_pos_2 = np.array([-0.2025,  0.35074,  -0.6])

    poses = [initial_pos_0, initial_pos_1, initial_pos_2]

    for i in range(10000):
        p.stepSimulation()
        for leg_number in [0, 1, 2]:
            # + 0.3 * (1-max(1, i/1000))
            pos_with_height = poses[leg_number] + 0.2 * \
                np.array([np.cos(i/1000), np.sin(i/1000), 0])
            solution = inv_kin_solver[leg_number].solve_virtual(
                target=pos_with_height, initial_tip=triped.get_virtual_state())
            triped.set_virtual_state(solution)
            print(solution)
            robot.set_virtual_state({'leg'+str(leg_number)+'_gimbal_joint': solution['leg'+str(leg_number)+'_gimbal_joint'],
                                     'leg'+str(leg_number)+'_extend_joint': solution['leg'+str(leg_number)+'_extend_joint']})
        time.sleep(1./240.)
    p.disconnect()
