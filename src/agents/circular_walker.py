from operator import le
import numpy as np
from copy import deepcopy


# Functions natively found in matlab but not python:
def angdiff(x, y):
    return np.abs(np.arctan2(np.sin(x-y), np.cos(x-y)))


def cart2pol(x, y):
    rho = np.sqrt(x**2 + y**2)
    phi = np.arctan2(y, x)
    return(rho, phi)


def targeted_step(s, height, start, stop):
    relative_pos = stop-start
    [len, rot] = cart2pol(relative_pos[0], relative_pos[1])
    straight_step = step(s, len, height)
    rotated_step = np.array([np.cos(rot)*straight_step[0]-np.sin(rot)*straight_step[1],
                             np.sin(rot)*straight_step[0] +
                             np.cos(rot)*straight_step[1],
                             straight_step[2]])
    return rotated_step+start


def step(s, length, height):
    z = height*np.exp(-(s-0.5) ** 2/0.01)
    x = length/(1+np.exp(-20*(s-0.5)))
    y = 0
    return np.array([x, y, z])


def rotz(theta):
    return np.array([np.cos(theta), np.sin(theta), 0])


class CircularWalker:
    """This agent is a simple walking pattern generator adapted 
       from the joystick controlled matlab simulation of the TriPed found here:
        https://github.com/TriPed-Robot/Matlab-Simulation/blob/master/examples/joy_stick_walking.slx

        It is completely open loop and therefore not very stable.
        Since the code was simply converted from matlab to python it may be messy and should not be
        used as a reference for the design of other agents or walking generators.
    """

    def __init__(self):

        # the time step between which the agent is called
        self.timestep = 1./50.

        # how much the angle should be incremented each timestep
        self.increment = np.pi

        # how high the robot should step
        self.step_height = 0.08

        # the radius of rotation
        self.swing_rad = 0.1

        # ----------------------------------------------------------

        # last command velocity [v,theta] v: speed, theta: movement angle
        self.last_vel = np.array([0.1/(2*np.pi), 0])

        # the command velocity [v,theta] v: speed, theta: movement angle
        self.cmd_vel = np.array([0.1, 0])

        # state of the state machine
        self.current_state = [2, 2, 2]

        # base positions for each leg
        self.initial_pos = [np.array([0.4*1.2, 0,  -0.6]),
                            np.array([-0.2025*1.2, -0.35074*1.2,  -0.6]),
                            np.array([-0.2025*1.2,  0.35074*1.2,  -0.6])]

        # the state of the foot as it leaves the ground
        self.pre_step_state = [np.zeros(4), np.zeros(4), np.zeros(4)]

        # position of the foot after it stepped
        self.post_step_state = deepcopy(self.initial_pos)

        # the current angle of rotation
        self.angle = -2*np.pi

        # tracks how far each lag has travelled since the last step
        self.move_memory = np.zeros(3)

        # the angle at which the leg should be lifted
        self.start_angle = [np.mod(2*np.pi/360 * 0 + 0*np.pi/2, 2*np.pi),
                            np.mod(2*np.pi/360 * 240 + 0*np.pi/2, 2*np.pi),
                            np.mod(2*np.pi/360 * 120 + 0*np.pi/2, 2*np.pi)]

        # the angle duration during which a step takes place
        self.rel_step_angle = 2*np.pi/360*60
        # the angle at which the step should touch the ground again
        self.stop_angle = [np.mod(self.start_angle[0]+self.rel_step_angle, 2*np.pi),
                           np.mod(self.start_angle[1] +
                                  self.rel_step_angle, 2*np.pi),
                           np.mod(self.start_angle[2]+self.rel_step_angle, 2*np.pi)]

    def _circular_offset(self, theta):
        return self.swing_rad*np.array([np.cos(theta), np.sin(theta), 0])

    def _swing_controller(self, leg_number):
        # parameterize step trajectory
        start_angle = self.pre_step_state[leg_number][3]
        stop_angle = start_angle+self.rel_step_angle
        s = angdiff(self.angle, start_angle) / \
            angdiff(stop_angle, start_angle)

        # calculate step size
        est_trvl = 2*abs(angdiff(stop_angle, start_angle))
        final_pos = self.initial_pos[leg_number] + \
            self._circular_offset(stop_angle) + \
            rotz(self.last_vel[1]) * est_trvl*self.last_vel[0]/2

        # move foot based on trajectory parameter and step size
        foot = targeted_step(
            s, self.step_height, self.pre_step_state[leg_number][0:3], final_pos)

        return s, foot

    def _body_controller(self, leg_number):

        # This block is redundant if the command velocity is always defined using [v,theta]
        # this would also make it possible ot combine self.last_vel and self.cmd_vel

        foot = self.post_step_state[leg_number]+self._circular_offset(self.angle)-rotz(
            self.cmd_vel[1])*self.move_memory*self.cmd_vel[0]

        angle_in_range = (np.mod(self.angle, 2*np.pi) >= self.start_angle[leg_number]) and (
            np.mod(self.angle, 2*np.pi) <= self.stop_angle[leg_number])

        distance_travelled = np.linalg.norm(
            self.post_step_state[leg_number]+self._circular_offset(self.angle)-foot) >= 0.05

        return angle_in_range, distance_travelled, foot

    def single_leg_state_machine(self, leg_number):

        if self.current_state[leg_number] == 0:
            s, foot = self._swing_controller(leg_number)
            if s <= 1:
                self.current_state[leg_number] = 0
            else:
                self.current_state[leg_number] = 1

                self.move_memory[leg_number] = 0
                # -self._circular_offset(self.angle)
                self.post_step_state[leg_number] = foot

        elif self.current_state[leg_number] == 1:
            angle_in_range, distance_travelled, foot = self._body_controller(
                leg_number)

            if angle_in_range and distance_travelled:
                self.current_state[leg_number] = 0

                self.pre_step_state[leg_number][0:3] = foot
                self.pre_step_state[leg_number][3] = self.angle
                self.last_vel = deepcopy(self.cmd_vel)
            else:
                self.current_state[leg_number] = 1

                self.move_memory[leg_number] += self.increment*self.timestep

        # initialisation case to allow the simulation to settle
        elif self.current_state[leg_number] == 2:
            foot = self.initial_pos[leg_number]+(2*np.pi+self.angle)/(
                2*np.pi)*np.array([self.swing_rad, 0, 0])
            self.pre_step_state[leg_number][0:3] = foot
            self.pre_step_state[leg_number][3] = 0

            if self.angle >= 0:
                self.current_state[leg_number] = 1
            else:
                self.current_state[leg_number] = 2

        return foot

    def move_robot(self):
        self.angle += self.increment*self.timestep
        print(self.post_step_state)
        return [self.single_leg_state_machine(0),
                self.single_leg_state_machine(1),
                self.single_leg_state_machine(2)]
