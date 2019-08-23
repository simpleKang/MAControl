import numpy as np
from sympy import *
import os


class TESTControl:
    def __init__(self, env, world, arglist):
        print("L1 control init")
        self.env = env
        self.world = world
        self.arglist = arglist
        self.obs_shape_n = [env.observation_space[i].shape for i in range(env.n)]
        self.actions_n = [world.dim_p * 2 + 1 for i in range(env.n)]
        self.n_features = self.obs_shape_n[0][0]
        self.pointA = (0, 0)
        self.pointB = (0, 0)


    def MotionController(self, blablabla):

    def PathPlanner(self, blablabla):