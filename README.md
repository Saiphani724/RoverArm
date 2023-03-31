A OpenAI Gym Env for Rover with Arm

Project Overview

https://docs.google.com/presentation/d/1NnCJ13qy9eBprIPJsYDFgFuNXPIBvIbRBtx6BVz_si8/edit#slide=id.p3

Task1 - Move the cart near the table and pick up the object in the tray

Reward = 1 (when the object is picked up)

Reward = 0 (else)


Code with Sample Actions
```
import rover_arm
import gym
env = gym.make('rover-arm-pick-v0', render_mode = 'rgb_array')

observation = env.reset()
done = False

while not done:
    action = env.action_space.sample()
    observation, reward, done, info = env.step(action)
    img = env.render()
    # print(img.shape)
    print(action, observation, reward)
    
print(reward, done, info)
```

You can try to explore the environment and action space by controlling the bot using Keyboard.


Keyboard Controls


Rover

Up, Down, Left, Right Arrows to steer the Rover.  


Arm

A, D -> Move the end-effector in X-axis

W, S -> Move the end-effector in Y-axis

Q, E -> Move the end-effector in Y-axis

-, + -> Open / Close the fingers of the robot arm

Note: W, S are also hot keys to adjust view in pybullet env (so ignore the changes or press again to undo the change.)


Code to control the bot using keyboard in human mode (needs to be run in local)


```
import rover_arm.keyboard_control as kc

import rover_arm
import gym
env = gym.make('rover-arm-pick-v0', render_mode = 'human')

keyboard_controller = kc.KeyboardAction()
keyboard_controller.start_listening()

observation = env.reset()
done = False

while not done:
    action = keyboard_controller.action
    observation, reward, done, info = env.step(action)
    # print(action, observation, reward)
# print(reward, done, info)
```


Task2: Pick the object from the closer tray and place it on the distant tray 

Use the env "rover-arm-place-v0" for task2

```

env = gym.make('rover-arm-place-v0')

```