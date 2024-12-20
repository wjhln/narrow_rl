#!/usr/bin/env python3
# coding:utf-8
from registration import StartRL_ROS_Environment
from agent import Agent
from loguru import logger

if __name__=="__main__":

    with open('output.txt', 'w') as file:
        file.write("\n")

    env = StartRL_ROS_Environment('MyEnv-v0')
    agent = Agent(gamma=0.99, epsilon=0.5, lr=0.0001,
                  input_dims=(env.observation_space.shape),
                  n_actions=env.action_space.n, mem_size=50000, eps_min=0.05,
                  batch_size=32, replace=1000, eps_dec=8e-6,
                  chkpt_dir='model/')
    n_games = 500000
    for i in range(n_games):
        observation = env.reset()
        done = False
        sore = 0
        n_steps = 0
        while not done:
            action = agent.choose_action(observation)
            print("n_steps:".ljust(10), f"{n_steps:<10}", "action:".ljust(10), f"{action:<10}")
            observation_, reward, done, info = env.step(action)
            print(f"obs:".ljust(10), f"{observation_[0]:<10} | {observation_[1]:<10} | {observation_[2]:<10} | {observation_[3]:<10} | {observation_[4]:<10} | {observation_[5]:<10} | {observation_[6]:<10}")

            sore += reward
            agent.store_transition(observation, action,
                                       reward, observation_, done)
            agent.learn()
            observation = observation_
            n_steps += 1
            logger.warning(f"{i:<10} | {sore/n_steps:<10}")

        if(i % 100 == 0 and i != 0):
            agent.save_models(str(i))
        with open('output.txt', 'a') as file:
            file.write(str(i)+","+str(sore/n_steps)+"\n")
        # state +=1
        # if(state > 5):
        #     state = 0
        # env.step(3)
    
