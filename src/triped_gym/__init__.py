from gym.envs.registration import register
from triped_gym.envs import *
register(id='TriPed-v0',
         entry_point='triped_gym.envs:PlaneEnvA',)
