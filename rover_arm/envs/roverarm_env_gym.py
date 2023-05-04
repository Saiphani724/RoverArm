import gym 
from gym import error, spaces, utils
from gym.utils import seeding

import os
import pybullet as p
import pybullet_data
import math
import numpy as np
import random
import site


class RoverArmEnvGym(gym.Env):

    def __init__(self, render_mode = 'rgb_array', maxSteps=10 * 1000, isDiscrete=False, urdfRoot = pybullet_data.getDataPath(), 
    width = 480, height = 480):
        self.metadata = {'render.modes': ['human' , 'rgb_array']}
        self.render_mode = render_mode
        self._isDiscrete = isDiscrete
        self._timeStep = 1. / 240.
        self._urdfRoot = urdfRoot
        self._maxSteps = maxSteps
        self._width = width
        self._height = height
        if self.render_mode == 'human':
            cid = p.connect(p.SHARED_MEMORY)
            if (cid < 0):
                cid = p.connect(p.GUI)
            p.resetDebugVisualizerCamera(1.3, 180, -41, [0.52, -0.2, -0.33])
        else:
            p.connect(p.DIRECT)
        self._cam_dist = 3
        self._cam_yaw = -0.35
        self._cam_pitch = -35
        self._cam_target_p = [0.67, -0.35, 0.20]

        p.resetDebugVisualizerCamera(cameraDistance= self._cam_dist , cameraYaw= self._cam_yaw, cameraPitch= self._cam_pitch, cameraTargetPosition=self._cam_target_p)
        self.action_space = spaces.Box(np.array([-1]*6), np.array([1]*6))
        self.boundary = 5
        self.observation_space = spaces.Box(np.array([-self.boundary, -self.boundary, -self.boundary, -self.boundary, -1, 0,0 , -self.boundary, -self.boundary, -1]), np.array([self.boundary, self.boundary, self.boundary, self.boundary, 1, 0.07, 0.07, self.boundary, self.boundary, 1]))

        # Joint indices as found by p.getJointInfo()
        self.steering_joints = [0, 2]
        self.drive_joints = [1, 3, 4, 5]
        # Joint speed
        self.joint_speed = 0
        # Drag constants
        self.c_rolling = 0.3
        self.c_drag = 0.01
        # Throttle constant increases "speed" of the car
        self.c_throttle = 200
        
        self.MAX_SPEED = 20

    def reset(self, seed = None):
        random.seed(seed)
        np.random.seed(seed)

        self.step_counter = 0
        p.resetSimulation()
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,0) # we will enable rendering after we loaded everything
        
        p.setGravity(0,0,-10)

        planeUid = p.loadURDF(os.path.join(self._urdfRoot,"plane.urdf"), basePosition=[0,0,-0.65])

        rest_poses = [0,-0.215,0,-2.57,0,2.356,2.356,0.08,0.08]
        
        x_pos = np.random.choice([random.uniform(-1, -0.3), random.uniform(1.25,2)])
        y_pos = random.uniform(-1, 2.5)
        
        BASE_DIR = site.getsitepackages()[0] + "/rover_arm/data/"
        # BASE_DIR = "./rover_arm/data/"
        self.roverarmUid = p.loadURDF(BASE_DIR + "rover_arm.xml", basePosition=[ x_pos, y_pos ,-0.5])


        for i in range(7,14):
            p.resetJointState(self.roverarmUid,i, rest_poses[i - 7])
        p.resetJointState(self.roverarmUid, 16, 0.07)
        p.resetJointState(self.roverarmUid, 17, 0.07)
        
        tableUid = p.loadURDF(os.path.join(self._urdfRoot, "table/table.urdf"),basePosition=[0.5,0,-0.65], globalScaling = 0.5)
        trayUid = p.loadURDF(os.path.join(self._urdfRoot, "tray/traybox.urdf"),basePosition=[0.45,0,-0.335], globalScaling = 0.5)

        state_object= [random.uniform(0.4, 0.5), random.uniform(-0.05, 0.05), -0.2]
        self.objectUid = p.loadURDF(os.path.join(self._urdfRoot, "random_urdfs/000/000.urdf"), basePosition=state_object, globalScaling = 0.8)

        state_rover = p.getLinkState(self.roverarmUid, 0)[0][:2]
        state_arm = p.getLinkState(self.roverarmUid, 18)[0]
        state_fingers = (p.getJointState(self.roverarmUid,16)[0], p.getJointState(self.roverarmUid, 17)[0])
        state_object, _ = p.getBasePositionAndOrientation(self.objectUid)

        self.observation = state_rover + state_arm + state_fingers + tuple(state_object)
        
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,1)
        info = {'state_rover': state_rover, 'state_arm' : state_arm, 'state_fingers': state_fingers, 'state_object': state_object}
        return (np.array(self.observation).astype(np.float32), info)

    def step(self, action):
        # p.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING)
        orientation = p.getQuaternionFromEuler([0.,-math.pi,math.pi/2.])
        dv = 0.05
        dx_a, dy_a, dz_a  = [x * dv for x in action[2:5] ]
        fingers = action[5]
        fingers = np.interp(fingers, [-1,1], [0, 0.07])

        currentPose = p.getLinkState(self.roverarmUid, 18)
        
        currentPosition = currentPose[0]
        
        newPosition = [currentPosition[0] + dx_a,
                       currentPosition[1] + dy_a,
                       currentPosition[2] + dz_a]
        jointPoses = p.calculateInverseKinematics(self.roverarmUid,18,newPosition, orientation)
        jointPoses_rover, jointPoses_arm = jointPoses[:6], jointPoses[6:13]
        p.setJointMotorControlArray(self.roverarmUid, list(range(7,14))+[16,17], p.POSITION_CONTROL, list(jointPoses_arm)+2*[fingers])

        throttle, steering_angle = action[:2]

        # Clip throttle and steering angle to reasonable values
        
        throttle = min(max(throttle, -1), 1)
        steering_angle = np.interp(steering_angle, [-1,1], [-0.6, 0.6])
        

        # Set the steering joint positions
        p.setJointMotorControlArray(self.roverarmUid, self.steering_joints,
                                    controlMode=p.POSITION_CONTROL,
                                    targetPositions=[steering_angle] * 2)

        # Calculate drag / mechanical resistance ourselves
        # Using velocity control, as torque control requires precise models
        friction = -self.joint_speed * (self.joint_speed * self.c_drag +
                                        self.c_rolling)
        
        acceleration = self.c_throttle * throttle + friction
        
        # Each time step is 1/240 of a second
        self.joint_speed = self.joint_speed + 1 / 30 * acceleration
        
        self.joint_speed = min(max(self.joint_speed, -self.MAX_SPEED), self.MAX_SPEED)
        

        # Set the velocity of the wheel joints directly
        p.setJointMotorControlArray(
            bodyUniqueId=self.roverarmUid,
            jointIndices=self.drive_joints,
            controlMode=p.VELOCITY_CONTROL,
            targetVelocities=[self.joint_speed] * 4,
            forces=[10] * 4)
        
        state_object_prev, _ = p.getBasePositionAndOrientation(self.objectUid)

        p.stepSimulation()

        state_object, _ = p.getBasePositionAndOrientation(self.objectUid)
        state_rover = p.getLinkState(self.roverarmUid, 0)[0][:2]
        state_arm = p.getLinkState(self.roverarmUid, 18)[0]
        state_fingers = (p.getJointState(self.roverarmUid,16)[0], p.getJointState(self.roverarmUid, 17)[0])

        terminated , truncated = False, False
        if state_object[2] > 0:
            reward = 1
            terminated = True
        else:
            x0, y0, z0 = np.abs(np.array(currentPosition) - np.array(state_object_prev))
            x1, y1, z1 = np.abs(np.array(state_arm) - np.array(state_object))
            reward = x0 - x1 + y0 - y1 + z0 - z1
            if abs(reward) > 1e-3:
                reward = reward/ 10
            else:
                reward = 0

        self.step_counter += 1
        def inGame(state_rover):
            rx, ry = state_rover
            inBound = rx > -self.boundary and rx < self.boundary
            inBound = inBound and ry > -self.boundary and ry < self.boundary
            return inBound
        
        if not inGame(state_rover):
            reward = -1
            terminated = True

        if self.step_counter > self._maxSteps:
            reward = 0
            truncated = True

        self.observation = state_rover + state_arm + state_fingers + tuple(state_object)
        info = {'state_rover': state_rover, 'state_arm' : state_arm, 'state_fingers': state_fingers, 'state_object': state_object}

        done = terminated or truncated
        return np.array(self.observation).astype(np.float32), reward, done, info


    def render(self, width = None, height = None):
        # cam = p.getDebugVisualizerCamera()
        # xyz = cam[11]
        # x= float(xyz[0]) + 0.125
        # y = xyz[1]
        # z = xyz[2]
        # p.resetDebugVisualizerCamera(cameraYaw = cam[8], cameraPitch= cam[9],cameraDistance = cam[10],cameraTargetPosition=[x,y,z])
        if width == None or height == None:
            width = self._width
            height = self._height
        if self.render_mode != 'rgb_array':
            return None
        view_matrix1 = p.computeViewMatrixFromYawPitchRoll(cameraTargetPosition=self._cam_target_p,
                                                            distance=self._cam_dist,
                                                            yaw=self._cam_yaw,
                                                            pitch=self._cam_pitch,
                                                            roll=0,
                                                            upAxisIndex=2)
        view_matrix2 = p.computeViewMatrixFromYawPitchRoll(cameraTargetPosition=[0.7,0,0.05],
                                                            distance=.7,
                                                            yaw=90,
                                                            pitch=-70,
                                                            roll=0,
                                                            upAxisIndex=2)
        proj_matrix = p.computeProjectionMatrixFOV(fov=60,
                                                     aspect=float(width) /height,
                                                     nearVal=0.1,
                                                     farVal=100.0)
        
        (_, _, px1, _, _) = p.getCameraImage(width=width,
                                              height=height,
                                              viewMatrix=view_matrix1,
                                              projectionMatrix=proj_matrix,
                                              renderer=p.ER_BULLET_HARDWARE_OPENGL)
        
        (_, _, px2, _, _) = p.getCameraImage(width=width,
                                              height=height,
                                              viewMatrix=view_matrix2,
                                              projectionMatrix=proj_matrix,
                                              renderer=p.ER_BULLET_HARDWARE_OPENGL)

        rgb_array1 = np.array(px1, dtype=np.uint8)
        rgb_array1 = np.reshape(rgb_array1, (height,width, 4))[:, :, :3]
        
        rgb_array2 = np.array(px2, dtype=np.uint8)
        rgb_array2 = np.reshape(rgb_array2, (height,width, 4))[:, :, :3]

        
        rgb_array = np.concatenate((rgb_array1 , rgb_array2), axis = 2)
        
        return rgb_array
    
    def _get_state(self):
        return self.observation

    def close(self):
        p.disconnect()

