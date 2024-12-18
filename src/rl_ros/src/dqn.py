#!/usr/bin/env python3
# coding:utf-8
from registration import StartRL_ROS_Environment
if __name__=="__main__":
    env = StartRL_ROS_Environment('MyEnv-v0')
    env.reset()
    env.step(0)
 
