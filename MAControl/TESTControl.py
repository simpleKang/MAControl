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

        self.vel = (0, 0)
        self.pos = (0, 0)
        self.pointAi = (0, 0)
        self.pointBi = (0, 0)

        self.L1_distance = 0
        self.nav_bearing = 0
        self.eta = 0
        self.lateral_acc = 0

        self.action = [0, 0, 0, 0, 0]

        self.waypoint_finished = False
        self.arrive_flag = True
        self.pointB_index = 0
        # 256×3的航点列表，第3列为航点状态 [0: 无航点] [1: 未飞] [2: pointA] [3: pointB] [4: 已到达]
        self.waypoint_list = [[0 for i in range(3)] for j in range(256)]

    def PathPlanner(self, obs):
        print("path plan")
        self.pointAi = (0, 0)
        self.pointBi = (0.9, 0)
        return self.pointAi, self.pointBi

    def MotionController(self, obs, pointAi, pointBi):
        print("motion control")
        self.vel = np.array(obs[0:2])
        self.pos = np.array(obs[2:4])
        pointAi = np.array(pointAi)
        pointBi = np.array(pointBi)

        speed = np.sqrt(np.square(self.vel[0]) + np.square(self.vel[1]))
        speed = max(0.001, speed)
        L1_distance = speed * 0.1  # L1 ratio

        vector_AB = pointBi-pointAi
        dist_AB = np.sqrt(np.square(vector_AB[0]) + np.square(vector_AB[1]))
        vector_AB_unit = vector_AB/max(dist_AB, 0.001)

        vector_AP = self.pos-pointAi
        dist_AP = np.sqrt(np.square(vector_AP[0]) + np.square(vector_AP[1]))
        vector_AP_unit = vector_AP/max(dist_AP, 0.001)

        vector_BP = self.pos - pointBi
        dist_BP = np.sqrt(np.square(vector_BP[0]) + np.square(vector_BP[1]))
        vector_BP_unit = vector_BP/max(dist_BP, 0.001)

        alongTrackDist = np.dot(vector_AP, vector_AB_unit)
        AB_to_BP_bearing = math.acos(np.dot(vector_AB_unit, vector_BP_unit))

        if dist_AP > L1_distance and alongTrackDist/max(dist_AP, 0.001) < -0.707:
            # calculate eta to fly to waypoint A
            eta = math.acos(np.dot(-1 * vector_AP_unit, self.vel/speed))

        elif abs(AB_to_BP_bearing) < math.radians(100):
            # calculate eta to fly to waypoint B
            eta = math.acos(np.dot(-1 * vector_BP_unit, self.vel/speed))

        else:
            # calculate eta to fly along the line between A and B
            eta2 = math.acos(np.dot(vector_AB, self.vel/speed))
            beta = math.acos(np.dot(vector_AP_unit, vector_AB_unit))
            xtrackErr = dist_AP * math.sin(beta)
            sine_eta1 = xtrackErr / max(L1_distance, 0.001)
            #  TODO: (c++): sine_eta1 = math::constrain(sine_eta1,-0.7071,0.7071)
            eta1 = math.asin(sine_eta1)
            eta = eta1 + eta2

        #  TODO: (c++): eta = math::constrain(eta,-pi/2,pi/2)
        self.lateral_acc = speed * speed / L1_distance * math.sin(eta) * 0.02  # K_L1

        lateral = np.array([self.vel[1], -1*self.vel[0]])
        acc = lateral * self.lateral_acc
        self.action = [0, 0, 0, 0, 0]
        self.action[2] = acc[0]
        self.action[4] = acc[1]
        return self.action




