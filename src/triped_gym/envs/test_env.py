import pybullet as p
import pybullet_data
from triped_gym.envs.position_controlled_env import PositionControlledEnv


class TestEnv(PositionControlledEnv):

    def __init__(self, rendering=False):
        """A environment for automatic testing
        """
        PositionControlledEnv.__init__(self, rendering)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.loadURDF("plane.urdf")

    def _get_reward(self):
        # placeholder reward
        return 0
