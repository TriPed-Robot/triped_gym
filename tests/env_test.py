import gym
import triped_gym
from triped_gym.utility import render_image
import time
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np

env = gym.make('TriPed-v0')

obs = env.reset()
print("The initial observation is {}".format(obs))

cam_dist = 2
cam_yaw = 0
render_width = 320
cam_pitch = -30
render_height = 240

fig = plt.figure()
base_pos, _ = env.robot.get_world_state()
camera_feed = plt.imshow(render_image(base_pos, cam_dist, cam_yaw,
                                      cam_pitch, render_width, render_height))


# Sample a random action from the entire action space
def update(i):
    random_action = env.action_space.sample()

    # # Take the action and get the new observation spacepi
    new_obs, _, _, _ = env.step(random_action)
    print("The new observation is {}".format(new_obs))

    base_pos, _ = env.robot.get_world_state()
    camera_feed.set_array(render_image(base_pos, cam_dist, cam_yaw,
                                       cam_pitch, render_width, render_height))
    return camera_feed


animation = FuncAnimation(fig, update, range(0, 20), blit=True)
plt.show()
