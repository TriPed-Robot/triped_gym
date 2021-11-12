import gym
import triped_gym
import time
env = gym.make('TriPed-v0')

obs = env.reset()
print("The initial observation is {}".format(obs))

# Sample a random action from the entire action space
for i in range(1000):
    random_action = env.action_space.sample()

    # # Take the action and get the new observation spacepi
    new_obs, reward, done, info = env.step(random_action)
    print("The new observation is {}".format(new_obs))

    time.sleep(1/240.)
