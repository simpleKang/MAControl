import numpy as np
import math
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
        self.vel = np.array([0, 0])
        self.pos = np.array([0, 0])
        self.pointAi = np.array([0, 0])
        self.pointBi = np.array([0, 0])

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
        self.pointAi = [0, 0]
        self.pointBi = [0, 0]
        return self.pointAi, self.pointBi

    def MotionController(self, obs, pointAi, pointBi):
        print("motion control")
        self.vel = np.array(obs[0:2])
        self.pos = np.array(obs[2:4])
        pointAi=np.array(pointAi)
        pointBi=np.array(pointBi)
        target_bearing = pointAi-self.pos

        groundspeed = np.sqrt(np.square(self.vel[0]) + np.square(self.vel[1]))
        groundspeed = max(0.1, groundspeed)
        L1_distance = groundspeed * 0.1  # L1 ratio

        vector_AB = pointBi-pointAi  # TODO: check AB点是否太近
        length_AB = np.sqrt(np.square(vector_AB[0]) + np.square(vector_AB[1]))
        vector_AB = vector_AB/length_AB

        vector_AP = self.pos-pointAi
        crosstrack_error = vector_AB % vector_AP
        distance_AP = np.sqrt(np.square(vector_AP[0]) + np.square(vector_AP[1]))
        alongTrackDist = vector_AP * vector_AB

        vector_BP = self.pos - pointBi
        length_BP = np.sqrt(np.square(vector_BP[0]) + np.square(vector_BP[1]))
        vector_BP_unit = vector_BP/length_BP

        AB_to_BP_bearing = math.acos(np.dot(vector_AB, vector_BP_unit))

        if distance_AP > L1_distance and alongTrackDist/max(distance_AP, 0.1) < -0.707:
            # calculate eta to fly to waypoint A
            vector_AP = vector_AP/distance_AP
            eta = math.acos(np.dot(-1 * vector_AP, self.vel)/groundspeed)
            nav_bearing = math.atan2(vector_AP[1], vector_AP[0])

        elif abs(AB_to_BP_bearing) < math.radians(100):
            # calculate eta to fly to waypoint B
            eta = math.acos(np.dot(-1 * vector_BP_unit, self.vel)/groundspeed)
            nav_bearing = math.atan2(vector_BP[1], vector_BP[0])

        else:
            # calculate eta to fly along between A and B
            a








        return self.action




