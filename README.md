A OpenAI Gym Env for Rover with Arm

Project Overview - https://1drv.ms/p/s!Aswo8fN5BKr61klitElDnhZ-JARq

Task1 - Move the cart near the table and pick up the object in the tray

Reward = 1 (when the object is picked up)

(Else) If the bot gets close to the object it gets a small positive reward (say 0.001) and if it gets far it gets a negative reward (say -0.001).


Code with Sample Actions
```
import rover_arm
import gymnasium as gym
env = gym.make('rover-arm-pick-v0', render_mode = 'rgb_array')

observation = env.reset()
done = False

while not done:
    action = env.action_space.sample()
    observation, reward, terminated, truncated,  info = env.step(action)
    done = terminated or truncated
    img = env.render()
    # print(img.shape)
    print(action, observation, reward)
    
print(reward, done, info)
```

You can try to explore the environment and action space by controlling the bot using Keyboard.

You will have to install the dev version in local, and give keyboard access to terminal or IDE where code is being executed. To install in dev version you could do

```

pip install 'rover-arm[dev]'

```


Keyboard Controls


Rover

Up, Down, Left, Right Arrows to steer the Rover.  


Arm

A, D -> Move the end-effector in X-axis

W, S -> Move the end-effector in Y-axis

Q, E -> Move the end-effector in Z-axis

-, + -> Open / Close the fingers of the robot arm

Note: W, S are also hot keys to adjust view in pybullet env (so ignore the changes or press again to undo the change.)


Code to control the bot using keyboard in human mode (needs to be run in local)


```
import rover_arm
import gymnasium as gym
import rover_arm.keyboard_control as kc

env = gym.make('rover-arm-pick-v0', render_mode = 'human')

keyboard_controller = kc.KeyboardAction()
keyboard_controller.start_listening()


observation = env.reset()
done = False

while not done:
    action = keyboard_controller.action
    observation, reward, terminated, truncated,  info = env.step(action)
    done = terminated or truncated
    img = env.render()
    # print(img.shape)
    print(action, observation, reward)
    
print(reward, done, info)
```


Task2: Pick the object from the closer tray and place it on the distant tray 

Use the env "rover-arm-place-v0" for task2

```

env = gym.make('rover-arm-place-v0')

```

Note: Task2 Env is not upto date, it uses gym style instead of gymnasium, so expect done instead of truncated and terminated.