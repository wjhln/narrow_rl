#!/usr/bin/env python3

import gym
from gym import spaces
import numpy as np
import math
from geometry_msgs.msg import Twist
import rospy
from std_srvs.srv import Empty

class MyEnv(gym.Env):
    metadata = {"render_modes": ["human", "rgb_array"], "render_fps": 4}

    def __init__(self):

        self.reset_world_proxy = rospy.ServiceProxy('/gazebo/reset_world', Empty)
        self.reset_simulation_proxy = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
        self._cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.action_space = spaces.Discrete(6)
        self.observation_space = spaces.Box(low=np.float32(0.1), high=np.float32(8.0), shape=(361,), dtype=np.float32)

    def step(self, action):
        """
        执行动作
        """
        assert self.action_space.contains(action), "无效的动作"

        velocity = np.float32(1.5)
        steer = np.float32(math.pi / 3)
        angular_speed = 0
        linear_speed = 0
        if action == 0:  # FORWARD
            linear_speed = velocity
            angular_speed = 0.0
        elif action == 1:  # LEFT
            linear_speed = velocity
            angular_speed = steer
        elif action == 2:  # RIGHT
            linear_speed = velocity
            angular_speed = - steer
        elif action == 3:  # BACKWARD
            linear_speed = -velocity
            angular_speed = -0
        elif action == 4:  # LEFT BACK
            linear_speed = -velocity
            angular_speed = - steer
        elif action == 5:  # FIGHTBACK
            linear_speed = -velocity
            angular_speed = - steer



        reward = self._compute_reward(action)
        done = self._is_done()
        observation = 1
        info = {}


        return observation, reward, done, info
        # done = bool(
        #     self.state >= 10.0
        #     or self.state <= -10.0
        #     or self.step_count >= self.max_steps
        # )
        #
        # reward = 1.0 if self.state == 10.0 else -1.0 if self.state == -10.0 else 0.0
        #
        # info = {}
        #
        # return np.array([self.state], dtype=np.float32), reward, done, info




    def reset(self, seed=None, return_info=False, options=None):
        # self.gazebo.resetSim()
        # self.gazebo.resetWorld()
        self.reset_world_proxy()
        self.reset_simulation_proxy()
        # We need the following line to seed self.np_random
        # self.action_space.seed(42)

        # # Choose the agent's location uniformly at random
        # self._agent_location = np.random.random_integers(0, self.size, size=2, dtype=int)

        # # We will sample the target's location randomly until it does not coincide with the agent's location
        # self._target_location = self._agent_location
        # while np.array_equal(self._target_location, self._agent_location):
        #     self._target_location = np.random.random_integers(0, self.size, size=2, dtype=int)

        # observation = self._get_obs()
        # info = self._get_info()
        return 0


    def _compute_reward(self,action):
        if action == 0 or action == 1 or action == 2:
            reward = 1
        else:
            reward = -1
        return reward

    def _is_done(self):
        return False
    # # def close(self):
    #     if self.window is not None:
    #         pygame.display.quit()
    #         pygame.quit()
