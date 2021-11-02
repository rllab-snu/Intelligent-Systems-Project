from gym.envs.registration import register
from rccar_env import Env

register(
    id='RCCar-v0',
    entry_point='env:Env',
    max_episode_steps=500,
    reward_threshold=1000.0,
)
