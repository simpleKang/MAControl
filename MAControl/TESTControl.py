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
        self.lateral_accel = 0

        self.action = [0, 0, 0, 0, 0]

        self.waypoint_finished = False
        self.arrive_flag = True
        self.pointB_index = 0
        # 256×3的航点列表，第3列为航点状态 [0: 无航点] [1: 未飞] [2: pointA] [3: pointB] [4: 已到达]
        self.waypoint_list = [[0 for i in range(3)] for j in range(256)]

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
        pointAi = np.array(pointAi)
        pointBi = np.array(pointBi)

        speed = np.sqrt(np.square(self.vel[0]) + np.square(self.vel[1]))
        speed = max(0.001, speed)
        L1_distance = speed * 0.1  # L1 ratio

        vector_AB = pointBi-pointAi  # TODO: check AB点是否太近
        dist_AB = np.sqrt(np.square(vector_AB[0]) + np.square(vector_AB[1]))
        vector_AB_unit = vector_AB/dist_AB

        vector_AP = self.pos-pointAi
        dist_AP = np.sqrt(np.square(vector_AP[0]) + np.square(vector_AP[1]))
        vector_AP_unit = vector_AP/dist_AP

        vector_BP = self.pos - pointBi
        dist_BP = np.sqrt(np.square(vector_BP[0]) + np.square(vector_BP[1]))
        vector_BP_unit = vector_BP/dist_BP

        alongTrackDist = np.dot(vector_AP, vector_AB_unit)
        AB_to_BP_bearing = math.acos(np.dot(vector_AB_unit, vector_BP_unit))

        if dist_AP > L1_distance and alongTrackDist/max(dist_AP, 0.001) < -0.707:
            # calculate eta to fly to waypoint A
            eta = math.acos(np.dot(-1 * vector_AP_unit, self.vel/speed))
            nav_bearing = math.atan2(vector_AP[1], vector_AP[0])

        elif abs(AB_to_BP_bearing) < math.radians(100):
            # calculate eta to fly to waypoint B
            eta = math.acos(np.dot(-1 * vector_BP_unit, self.vel/speed))
            nav_bearing = math.atan2(vector_BP[1], vector_BP[0])

        else:
            # calculate eta to fly along the line between A and B
            eta2 = math.acos(np.dot(vector_AB, self.vel/speed))
            beta = math.acos(np.dot(vector_AP_unit, vector_AB_unit))
            xtrackErr = dist_AP * math.sin(beta)
            eta1 =
            vector_AP =
            sin_eta1 = xtrackErr / max(L1_distance, 0.001)
            #  TODO: (c++): sin_eta1 = math::constrain(sin_eta1,-0.7071f,0.7071f)
            eta1 = math.asin(sin_eta1)
            eta = eta1 + eta2
            nav_bearing = math.atan2(vector_AB(1), vector_AB(0)) + eta1

        #  TODO: (c++): eta = math::constrain(eta,-pi/2,pi/2)
        KL1 = 0.02
        self.lateral_accel = KL1 * groundspeed * groundspeed / L1_distance * math.sin(eta)
        circle_mode = false
        bearing_error = eta

        accel_unit = np.array([self.vel[1], -1*self.vel[0]])
        accel_command = accel_unit * self.lateral_accel
        self.action[2] = accel_command[0]
        self.action[4] = accel_command[1]
        return self.action




