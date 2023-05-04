from gymnasium.envs.registration import register
from gym.envs.registration import register as register_gym

register(
    id='rover-arm-pick-v0',
    entry_point='rover_arm.envs:RoverArmEnv',
)

register_gym(
    id='rover-arm-pick-gym-v0',
    entry_point='rover_arm.envs:RoverArmEnvGym',
)

register(
    id='rover-arm-place-v0',
    entry_point='rover_arm.envs:RoverArmToPlaceEnv',
)
