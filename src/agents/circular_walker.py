from operator import le
import numpy as np
from copy import deepcopy


# Functions natively found in matlab but not python:
def angdiff(x, y):
    return np.abs(np.atan2(np.sin(x-y), np.cos(x-y)))


def cart2pol(x, y):
    rho = np.sqrt(x**2 + y**2)
    phi = np.arctan2(y, x)
    return(rho, phi)


def targeted_step(s, height, start, stop):
    relative_pos = stop-start
    [rot, len] = cart2pol(relative_pos(1), relative_pos(2))
    return rotz(rot/(2*np.pi)*360)@step(s, len, height)+start


def step(s, length, height):
    z = height*np.exp(-(s-0.5) ^ 2/0.01)
    x = length/(1+np.exp(-20*(s-0.5)))
    y = 0
    return np.array([x, y, z])


def rotz(theta):
    return np.array([[np.cos(theta), -np.sin(theta), 0],
                     [np.sin(theta), np.cos(theta), 0],
                     [0, 0, 1]], dtype=object)


class CircularWalker:
    """This agent is a simple demonstrator adapted from the joystick controlled matlab simulation
        of the TriPed found here:
        https://github.com/TriPed-Robot/Matlab-Simulation/blob/master/examples/joy_stick_walking.slx

        Since the code was simply converted from matlab to python it may be messy and should not be
        used as a reference for the design of other agents or walking generators.
    """

    def __init__(self):

        # base positions for each leg
        self.initial_pos = [np.array([0.4*1.2, 0,  -0.6]),
                            np.array([-0.2025*1.2, -0.35074*1.2,  -0.6]),
                            np.array([-0.2025*1.2,  0.35074*1.2,  -0.6])]

        # last command velocity [v,theta] v: speed, theta: movement angle
        self.last_vel = np.array([1, 0])

        # angle at which the robot last stepped
        self.prev_step = [0, 0, 0]

        # state of the state machine
        self.current_state = [2, 2, 2]

        # the current position of the feet
        self.foot_pos = [np.zeros(4), np.zeros(4), np.zeros(4)]
        # position of the foot after it stepped
        self.step_pos = deepcopy(self.initial_pos)

        # the current angle of rotation
        self.angle = -2*np.pi

        # the radius of rotation
        self.swing_rad = 0.1

        # the command velocity [vx,vy]
        self.cmd_vel = np.array([1, 0])

        # the angle at which the leg should be lifted
        self.start_angle = [np.mod(2*np.pi/360 * 0 + 0*np.pi/2, 2*np.pi),
                            np.mod(2*np.pi/360 * 240 + 0*np.pi/2, 2*np.pi),
                            np.mod(2*np.pi/360 * 120 + 0*np.pi/2, 2*np.pi)]

        # the angle duration during which a step takes place
        self.rel_step_angle = 60
        # the angle at which the step should touch the ground again
        self.stop_angle = [np.mod(self.start_angle[0]+2*np.pi/360*self.rel_step_angle, 2*np.pi),
                           np.mod(self.start_angle[1]+2*np.pi /
                                  360*self.rel_step_angle, 2*np.pi),
                           np.mod(self.start_angle[2]+2*np.pi/360*self.rel_step_angle, 2*np.pi)]

        # the time step between which the agent is called
        self.timestep = 0.01
        # how much the angle should be incremented each timestep
        self.increment = np.pi

        # how high the robot should step

    def single_leg_state_machine(self, leg_number):

        # This block is redundant if the command velocity is always defined using [v,theta]
        # this would also make it possible ot combine self.last_vel and self.cmd_vel
        [rad_walking_dir, velocity] = cart2pol(
            self.cmd_vel[0], self.cmd_vel[1])
        walking_dir = rad_walking_dir/(2*np.pi)*360
        velocity = velocity/(2*np.pi)

        move_memory = self.angle-self.prev_step[leg_number]

        if self.current_state[leg_number] == 0:
            init_angle = self.foot_pos[leg_number][3]
            final_angle = init_angle+2*np.pi/360*self.rel_step_angle
            s = angdiff(self.angle, init_angle) / \
                angdiff(final_angle, init_angle)

            if s <= 1:
                self.current_state[leg_number] = 0
                self.prev_step[leg_number] = self.prev_step[leg_number]+2.5*0.01
            else:
                self.current_state[leg_number] = 1
                self.prev_step[leg_number] = self.angle

            est_trvl = 2*abs(angdiff(self.final_angle[leg_number], init_angle))
            final_pos = self.initial_pos[leg_number] + \
                self.swing_rad*np.array([np.cos(final_angle), np.sin(final_angle), 0]) + \
                rotz(self.last_vel[1]) @ \
                np.array([1, 0, 0])*est_trvl*self.last_vel[0]/2
            foot = targeted_step(
                s, self.height, self.foot_pos[leg_number][0:2], final_pos)
            # current_liftoff = foot_pos
            step_pos = self.initial_pos[leg_number] + rotz(
                self.last_vel[1])@np.array([1, 0, 0])*est_trvl*self.last_vel[0]/2

        elif self.current_state[leg_number] == 1:
            foot = self.step_pos[leg_number]+self.swing_rad*np.array([np.cos(self.angle), np.sin(
                self.angle), 0])-rotz(walking_dir)@np.array([1, 0, 0])*move_memory*velocity

            angle_in_range = (np.mod(self.angle, 2*np.pi) >= self.start_angle[leg_number]) and (
                np.mod(self.angle, 2*np.pi) <= self.stop_angle[leg_number])
            distance_travelled = np.linalg.norm(self.step_pos[leg_number]+self.swing_rad*np.array(
                [np.cos(self.angle), np.sin(self.angle), 0])-foot) >= 0.05
            if angle_in_range and distance_travelled:
                self.current_state[leg_number] = 0
            else:
                self.current_state[leg_number] = 1

            self.foot_pos[leg_number] = np.array([foot, self.angle])
            # self.prev_step[leg_number] = prev_step
            self.last_vel = np.array([velocity, walking_dir])

        # initialisation case to allow the simulation to settle
        elif self.current_state[leg_number] == 2:
            if self.angle >= 0:
                self.current_state[leg_number] = 1
            else:
                self.current_state[leg_number] = 2

            foot = self.initial_pos[leg_number]+(2*np.pi+self.angle)/(
                2*np.pi)*self.swing_rad*np.array([np.cos(0), np.sin(0), 0])
            self.foot_pos[leg_number] = np.array([foot, 0])
            #self.prev_step[leg_number] = prev_step

        return foot

    def move_robot(self):
        self.angle += self.increment*self.timestep
        return [self.single_leg_state_machine(0),
                self.single_leg_state_machine(1),
                self.single_leg_state_machine(2)]
