from gym.envs.registration import register
register(id='TriPedTest-v0',
         entry_point='triped_gym.envs.plane_env_a:TestEnv',)
