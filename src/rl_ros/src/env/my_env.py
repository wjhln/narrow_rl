#!/usr/bin/env python3

import gym
from gym import spaces
import numpy as np
import math
from geometry_msgs.msg import Twist
import rospy
from std_srvs.srv import Empty
from geometry_msgs.msg import Vector3
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan
import time

class MyEnv(gym.Env):
    metadata = {"render_modes": ["human", "rgb_array"], "render_fps": 4}

    def __init__(self):
        rospy.init_node('env', anonymous=True)
        self.reset_world_proxy = rospy.ServiceProxy('/gazebo/reset_world', Empty)
        self.reset_simulation_proxy = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
        self.move_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber("/collision", Bool, self._collision_state_callback)
        rospy.Subscriber("/scan", LaserScan, self._laser_scan_callback)
        self.is_collision = False
        self.laser_scan = []
        self.action_space = spaces.Discrete(6)
        self.observation_space = spaces.Box(low=np.float32(0.1), high=np.float32(25.0), shape=(7,), dtype=np.float32)

    def step(self, action):
        assert self.action_space.contains(action), "无效的动作"

        velocity = np.float32(0.5)
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
            angular_speed = steer
        elif action == 5:  # FIGHTBACK
            linear_speed = -velocity
            angular_speed = - steer


        observation = self.laser_scan
        done = self.is_collision
        reward = self._compute_reward(observation, action, done)
        info = {}
        rate = rospy.Rate(10)
        self._move_base(linear_speed, angular_speed)

        rate.sleep()
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

        self._move_base(0, 0)
        self.reset_world_proxy()
        time.sleep(1)

        observation = self.laser_scan
        # self.reset_simulation_proxy()
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
        return observation


    def _compute_reward(self, observations, action, done):

        if not done:

            gap = observations
            gap.sort()
            decay1 = 0.9
            reward3 = 0
            for item in gap:
                reward3 += decay1 * math.log10(item)
                decay1 = decay1*decay1

            if action == 0 or action == 1 or action == 2:
                reward1 = observations[3]
            else:
                reward1 = -observations[3]

            reward2 = -abs(observations[1] - observations[5])
            reward = reward1 + 0.2 * reward2 + reward3
        else:
            reward = -100
        return reward

    def _is_done(self):
        return False
    # # def close(self):
    #     if self.window is not None:
    #         pygame.display.quit()
    #         pygame.quit()


    def _move_base(self, linear_speed, angular_speed):
        twist = Twist()
        twist.linear.x = linear_speed; twist.linear.y = 0.0; twist.linear.z = 0.0
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = angular_speed
        self.move_pub.publish(twist)
        # print(linear_speed, angular_speed)

    def _collision_state_callback(self,msg):
        self.is_collision = msg.data

    def _laser_scan_callback(self,msg):
        self.laser_scan = []
        ranges = msg.ranges  # 获取激光雷达的距离数据
        angle_min = msg.angle_min  # 激光雷达的最小角度
        angle_increment = msg.angle_increment  # 激光雷达的角度增量
        points = []  # 用来存储转换后的坐标点
        collision_points = [] # 用来存储碰撞点
        for i, r in enumerate(ranges):
            if r == float ('Inf') or np.isinf(r):
                self.laser_scan.append(msg.range_max)
            elif np.isnan(r):
                self.laser_scan.append(msg.range_min)
            else:
                self.laser_scan.append(round(r,3))        