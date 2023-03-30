from gym.envs.registration import register

register(
    id='rover-arm-v0',
    entry_point='rover_arm.envs:RoverArmEnv',
)
