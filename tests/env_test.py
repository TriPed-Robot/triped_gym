import gym
import triped_gym
from triped_gym.utility import render_image
import time
env = gym.make('TriPed-v0')

obs = env.reset()
print("The initial observation is {}".format(obs))

cam_dist = 3
cam_yaw = 0
render_width = 320
cam_pitch = -30
render_height = 240

# Sample a random action from the entire action space
for i in range(1000):
    random_action = env.action_space.sample()

    # # Take the action and get the new observation spacepi
    new_obs, reward, done, info = env.step(random_action)
    print("The new observation is {}".format(new_obs))

    base_pos, _ = env.robot.get_world_state()
    rpg_img = render_image(base_pos, cam_dist, cam_yaw,
                           cam_pitch, render_width, render_height)
    time.sleep(1/240.)
