import numpy as np
from sympy import *
import os


class TESTControl():
    def __init__(self, name, env, world, agent_index, arglist):
        print("L1 control init")
        self.name = name
        self.env = env
        self.world = world
        self.index = agent_index
        self.arglist = arglist
        self.obs_shape = env.observation_space[agent_index].shape
        self.action_shape = env.action_space[agent_index].shape
        self.pointA = (0, 0)
        self.pointB = (0, 0)
        self.pointL1= (0, 0)


    def MotionController(self, blablabla):

    def PathPlanner(self, blablabla):
