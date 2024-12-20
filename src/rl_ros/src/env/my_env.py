#!/usr/bin/env python3

import gym
from gym import spaces
import numpy as np
import math
from geometry_msgs.msg import Twist
import rospy
from std_srvs.srv import Empty
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from scipy.spatial import distance
from colorama import Fore, Back, Style, init

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
        rospy.Subscriber("/sunny/state", Odometry, self._sunny_state_callback)
        self.is_collision = False
        self.laser_scan = []
        self.sunny_pose = (0,0)
        self.point_end = (9,7)
        self.distance = distance.euclidean(self.sunny_pose, self.point_end)
        self.action_space = spaces.Discrete(8)
        self.observation_space = spaces.Box(low=np.float32(0.1), high=np.float32(25.0), shape=(7,), dtype=np.float32)


    def step(self, action):
        assert self.action_space.contains(action), "无效的动作"

        velocity = np.float32(0.15)
        steer = np.float32(math.pi / 12)
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
        elif action == 6:  #Turin Left
            linear_speed = 0
            angular_speed = steer
        elif action == 7:  # Turin Right
            linear_speed = 0
            angular_speed = -steer


        rate = rospy.Rate(10)
        # self._move_base(0, 0)
        self._move_base(linear_speed, angular_speed)
        rate.sleep()

        observation = self.laser_scan
        done = self._is_done()
        reward = self._compute_reward(observation, action, done)
        info = {}

        return observation, reward, done, info

    def reset(self, seed=None, return_info=False, options=None):

        self._move_base(0, 0)
        self.reset_world_proxy()
        time.sleep(0.2)

        observation = self.laser_scan
       
        return observation


    def _compute_reward(self, observations, action, done):

        if not done:

            gap = []
            for item in observations:
                gap.append(item)
            gap.sort()
            decay1 = 0.9
            reward1 = 0
            for item in gap:
                reward1 += decay1 * math.log10(item)
                decay1 = decay1*decay1

            if action == 0 or action == 1 or action == 2:
                reward2 = observations[3]
            else:
                reward2 = -observations[3]

            reward3 = -abs(observations[1] - observations[5])

            dis_ = distance.euclidean(self.sunny_pose, self.point_end)
            reward4 = (self.distance - dis_) * 10
            self.distance = dis_

            

            reward = reward1 + reward2 + reward3 + reward4
            print("reward:".ljust(10), f"{round(reward, 3):<10} | {round(reward1, 3):<10} | {round(reward2, 3):<10} | {round(reward3, 3):<10} | {round(reward4, 3):<10}")

        else:
            if distance.euclidean(self.sunny_pose, self.point_end) < 0.36:
                reward = 100
                print(Fore.GREEN + "arrive")
            else:
                reward = -100
                print(Fore.RED + "collision")
            print("reward:".ljust(10), f"{reward:<10} | {0:<10} | {0:<10} | {0:<10} | {0:<10}")

        return reward

    def _is_done(self):
        if self.is_collision or distance.euclidean(self.sunny_pose, self.point_end) < 0.36:
            return True
        else:
            return False


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

    def _sunny_state_callback(self,msg):
        self.sunny_pose = (msg.pose.pose.position.x,msg.pose.pose.position.y)