import numpy as np
from sympy import *
import os


class TESTControl():
    def __init__(self, name, env, world, agent_index, arglist):
        print("control init")
        self.name = name
        self.env = env
        self.world = world
        self.index = agent_index
        self.arglist = arglist

        self.vel = (0, 0)
        self.pos = (0, 0)
        self.pointA = (0, 0)
        self.pointB = (0, 0)

        self.L1_distance = 0
        self.nav_bearing = 0
        self.xtrack_vel = 0
        self.ltrack_vel = 0
        self.eta = 0
        self.lateral_accel = 0

        self.action = [0, 0, 0, 0, 0]

    def MotionController(self, obs, pointA, pointB):
        print("motion control")
        self.vel = (0, 0)
        self.lateral_accel = (0, 0)
        self.action = [0, 0, 0, 0, 0]
        return self.action

    def PathPlanner(self, obs):
        print("path plan")
        self.pointA = (0, 0)
        self.pointB = (0, 0)
        return self.pointA, self.pointB


