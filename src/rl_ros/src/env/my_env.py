#!/usr/bin/env python3

import gym
from gym import spaces
import numpy as np
import math
from geometry_msgs.msg import Twist

class MyEnv(gym.Env):
    metadata = {"render_modes": ["human", "rgb_array"], "render_fps": 4}

    def __init__(self):
        self.action_space = spaces.Discrete(6)
        self.observation_space = spaces.Box(low=np.float32(0), high=np.float32(6), shape=(32,), dtype=np.float32)
        self.node = car.Node(x=1.5, y=0, yaw=math.pi / 2, v=0.0, direct=0)
        self.yaw_old = 0.0

        # self.size = size  # The size of the square grid
        # self.window_size = 512  # The size of the PyGame window
        #
        # # Observations are dictionaries with the agent's and the target's location.
        # # Each location is encoded as an element of {0, ..., `size`}^2, i.e. MultiDiscrete([size, size]).
        # self.observation_space = spaces.Dict(
        #     {
        #         "agent": spaces.Box(0, size - 1, shape=(2,), dtype=int),
        #         "target": spaces.Box(0, size - 1, shape=(2,), dtype=int),
        #     }
        # )
        #
        # # We have 4 actions, corresponding to "right", "up", "left", "down", "right"
        # self.action_space = spaces.Discrete(4)
        #
        # """
        # The following dictionary maps abstract actions from `self.action_space` to
        # the direction we will walk in if that action is taken.
        # I.e. 0 corresponds to "right", 1 to "up" etc.
        # """
        # self._action_to_direction = {
        #     0: np.array([1, 0]),
        #     1: np.array([0, 1]),
        #     2: np.array([-1, 0]),
        #     3: np.array([0, -1]),
        # }
        #
        # """
        # If human-rendering is used, `self.window` will be a reference
        # to the window that we draw to. `self.clock` will be a clock that is used
        # to ensure that the environment is rendered at the correct framerate in
        # human-mode. They will remain `None` until human-mode is used for the
        # first time.
        # """
        # self.window = None
        # self.clock = None

    def step(self, action):
        """
        执行动作
        """
        assert self.action_space.contains(action), "无效的动作"

        velocity = np.float32(1.5)
        steer = np.float32(math.pi / 3)
        car_delta = 0
        car_v = 0
        if action == 0:  # FORWARD
            car_v = velocity
            car_delta = 0.0
        elif action == 1:  # LEFT
            car_v = velocity
            car_delta = steer
        elif action == 2:  # RIGHT
            car_v = velocity
            car_delta = - steer
        elif action == 3:  # BACKWARD
            car_v = -velocity
            car_delta = -0
        elif action == 4:  # LEFT BACK
            car_v = -velocity
            car_delta = - steer
        elif action == 5:  # FIGHTBACK
            car_v = -velocity
            car_delta = - steer


        acceleration = 0
        delta = car_delta
        self.node.v = car_v
        self.node.update(acceleration, delta, 1)

        reward = self._compute_reward(action)
        done = self._is_done()
        return reward,done
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
        # We need the following line to seed self.np_random
        self.action_space.seed(42)

        # Choose the agent's location uniformly at random
        self._agent_location = np.random.random_integers(0, self.size, size=2, dtype=int)

        # We will sample the target's location randomly until it does not coincide with the agent's location
        self._target_location = self._agent_location
        while np.array_equal(self._target_location, self._agent_location):
            self._target_location = np.random.random_integers(0, self.size, size=2, dtype=int)

        observation = self._get_obs()
        info = self._get_info()
        return (observation, info) if return_info else observation

    def render(self,mode='plt'):
        dy = (self.node.yaw - self.yaw_old) / (0.01 * C.dt)
        steer = rs.pi_2_pi(-math.atan(C.WB * dy))
        car.draw_car(self.node.x, self.node.y, self.yaw_old, steer, C)
        self.yaw_old = self.node.yaw


        plt.plot(20, 20, color='gray', linewidth=2.0)
        plt.plot([0, 0], [0, 15])
        plt.plot([3, 3], [0, 15])
        plt.axis("equal")
        plt.title("v=" + str(self.node.v)[:4] + ";  yaw=" + str(self.node.yaw)[:4])

        plt.pause(0.0001)

    def _compute_reward(self,action):
        if action == 0 or action == 1 or action == 2:
            reward = 1
        else:
            reward = -1
        return reward

    def _is_done(self):
        if self.node.y > 10:
            return True
        else:
            return False
    # def close(self):
    #     if self.window is not None:
    #         pygame.display.quit()
    #         pygame.quit()
