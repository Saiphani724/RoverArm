from gym.envs.registration import register

register(
    id='rover-arm-pick-v0',
    entry_point='rover_arm.envs:RoverArmEnv',
)

register(
    id='rover-arm-place-v0',
    entry_point='rover_arm.envs:RoverArmToPlaceEnv',
)
