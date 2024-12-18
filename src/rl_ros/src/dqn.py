#!/usr/bin/env python3
# coding:utf-8
from registration import StartRL_ROS_Environment
from agent import Agent

if __name__=="__main__":
    env = StartRL_ROS_Environment('MyEnv-v0')
    agent = Agent(gamma=0.99, epsilon=0.5, lr=0.0001,
                  input_dims=(env.observation_space.shape),
                  n_actions=env.action_space.n, mem_size=50000, eps_min=0.05,
                  batch_size=32, replace=1000, eps_dec=8e-6,
                  chkpt_dir='')
    n_games = 5000

    for i in range(n_games):
        observation = env.reset()
        done = False
        sore = 0
        while not done:
            action = agent.choose_action(observation)
            observation_, reward, done, info = env.step(action)
            sore += reward
            print(reward,action)
            agent.store_transition(observation, action,
                                       reward, observation_, done)
            agent.learn()
            observation = observation_
        # state +=1
        # if(state > 5):
        #     state = 0
        # env.step(3)
    
