import numpy as np
import tensorflow as tf
import operator
import random
import math
import os
import matplotlib.pyplot as plt


tar = [[0, 1, 2],
       [6, 5, 8],
       [8, 7, 4]]

a = [1, 2, 3]

b = [4, 5, 6]


import gym
env = gym.make('CartPole-v0')
for i in range(200):
    observation = env.reset()
    for t in range(100):
        env.render()
        print(observation)
        action = env.action_space.sample()
        observation, reward, done, info = env.step(action)
        if done:
            print("Episode finished after {} timesteps".format(t+1))
        break



pass
