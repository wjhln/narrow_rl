#!/usr/bin/env python3
# coding:utf-8
import time
from registration import StartRL_ROS_Environment
if __name__=="__main__":
    env = StartRL_ROS_Environment('MyEnv-v0')
    env.reset()
    time.sleep(2)
    state = 0
    while True:
        env.step(state)
        state +=1
        if(state > 5):
            state = 0
        # env.step(3)
    
