import numpy as np
import os


class TESTControl():
    def __init__(self, name, env, world, agent_index, arglist):
        print("control init")
        self.name = name
        self.env = env
        self.world = world
        self.index = agent_index
        self.arglist = arglist

        self.state = 0
        self.vel = (0, 0)
        self.pos = (0, 0)
        self.pointAi = (0, 0)
        self.pointBi = (0, 0)

        self.L1_distance = 0
        self.nav_bearing = 0
        self.eta = 0
        self.lateral_accel = 0

        self.waypoint_list = [[0 for i in range(3)] for j in range(256)]
        self.action = [0, 0, 0, 0, 0]

    def PathPlanner(self, obs):
        print("path plan")

        # TODO:判断是否修改当前航点状态

        # TODO:根据obs进行判断是否修改航点列表
        if():
            # TODO:按需求修改航点列表
            self.waypoint_listint

        # TODO:从航点列表中取出A点和B点
        self.pointAi = (0, 0)
        self.pointBi = (0, 0)
        return self.pointAi, self.pointBi

    def MotionController(self, obs, pointAi, pointBi):
        print("motion control")
        self.vel = obs[0:2]
        self.pos = obs[2:4]
        return self.action




