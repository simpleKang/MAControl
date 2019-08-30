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

        self.action = [0, 0, 0, 0, 0]

        self.waypoint_finished = False
        self.arrive_flag = False
        self.pointB_index = 0
        # 256×3的航点列表，第3列为航点状态 [0: 无航点] [1: 未飞] [2: pointA] [3: pointB] [4: 已到达]
        self.waypoint_list = [[0 for i in range(3)] for j in range(256)]

    def PathPlanner(self, obs):
        print("path plan")

        # TODO:根据obs进行判断是否修改航点列表
        # if True:
        #     self.WaypointUpdater(obs)

        # 初始时刻输出A、B坐标
        if self.pointB_index == 0:
            self.pointA = (obs[2], obs[3])
            self.pointB = (self.waypoint_list[self.pointB_index][0], self.waypoint_list[self.pointB_index][1])

        # 更改航点状态并输出A、B坐标
        if self.arrive_flag:
            if self.waypoint_list[self.pointB_index+1][2] != 0 and self.pointB_index < 255:
                if self.pointB_index > 0:
                    self.waypoint_list[self.pointB_index-1][2] = 4
                self.waypoint_list[self.pointB_index][2] = 2
                self.waypoint_list[self.pointB_index+1][2] = 3
                self.pointA = (self.waypoint_list[self.pointB_index][0], self.waypoint_list[self.pointB_index][1])
                self.pointB = (self.waypoint_list[self.pointB_index+1][0], self.waypoint_list[self.pointB_index+1][1])
                self.arrive_flag = False
                self.pointB_index += 1
            else:
                self.waypoint_list[self.pointB_index-1][2] = 4
                self.waypoint_list[self.pointB_index][2] = 4
                self.pointB_index = 0
                self.pointA = (self.waypoint_list[self.pointB_index][0], self.waypoint_list[self.pointB_index][1])
                self.pointB = (self.waypoint_list[self.pointB_index + 1][0], self.waypoint_list[self.pointB_index + 1][1])
                # self.waypoint_finished = True

        return self.pointA, self.pointB, self.waypoint_finished

    def MotionController(self, obs, pointAi, pointBi):
        print("motion control")
        self.vel = np.array(obs[0:2])
        self.pos = np.array(obs[2:4])
        pointAi = np.array(pointAi)
        pointBi = np.array(pointBi)

        # Set L1 Parameter
        L1_ratio = 0.1
        BP_range = 0.5
        Ex_speed = 7
        K_L1 = 0.02
        K_acct = 0.02

        speed = np.sqrt(np.square(self.vel[0]) + np.square(self.vel[1]))
        speed = speed
        L1_distance = speed * L1_ratio

        vector_AB = pointBi-pointAi
        dist_AB = np.sqrt(np.square(vector_AB[0]) + np.square(vector_AB[1]))
        vector_AB_unit = vector_AB/dist_AB

        vector_AP = self.pos-pointAi
        dist_AP = np.sqrt(np.square(vector_AP[0]) + np.square(vector_AP[1]))
        vector_AP_unit = vector_AP/dist_AP

        vector_BP = self.pos - pointBi
        dist_BP = np.sqrt(np.square(vector_BP[0]) + np.square(vector_BP[1]))
        self.arrive_flag = True if dist_BP < BP_range else False
        vector_BP_unit = vector_BP/dist_BP

        alongTrackDist = np.dot(vector_AP, vector_AB_unit)
        rrr1 = np.dot(vector_AB_unit, vector_BP_unit)
        rrr1 = np.piecewise(rrr1, [rrr1 < -1, -1 <= rrr1 <= 1, rrr1 >= 1], [-1, lambda rrr1:rrr1, 1])
        AB_to_BP_bearing = math.acos(rrr1)

        if dist_AP > L1_distance and alongTrackDist/dist_AP < -0.707:
            # calculate eta to fly to waypoint A
            rrr2 = np.dot(-1 * vector_AP_unit, self.vel/speed)
            rrr2 = np.piecewise(rrr2, [rrr2 < -1, -1 <= rrr2 <= 1, rrr2 >= 1], [-1, lambda rrr2: rrr2, 1])
            eta = math.acos(rrr2)
            print('scene1')

        elif abs(AB_to_BP_bearing) < math.radians(100):
            # calculate eta to fly to waypoint B
            eta = math.acos(np.dot(-1 * vector_BP_unit, self.vel/speed))
            print('scene2')

        else:
            # calculate eta to fly along the line between A and B
            rrr3 = np.dot(vector_AB_unit, self.vel/speed)
            rrr3 = np.piecewise(rrr3, [rrr3 < -1, -1 <= rrr3 <= 1, rrr3 >= 1], [-1, lambda rrr3: rrr3, 1])
            eta2 = math.acos(rrr3)
            rrr4 = np.dot(vector_AP_unit, vector_AB_unit)
            rrr4 = np.piecewise(rrr4, [rrr4 < -1, -1 <= rrr4 <= 1, rrr4 >= 1], [-1, lambda rrr4: rrr4, 1])
            beta = math.acos(rrr4)
            xtrackErr = dist_AP * math.sin(beta)
            sine_eta1 = xtrackErr / L1_distance
            #  TODO: (c++): sine_eta1 = math::constrain(sine_eta1,-0.7071,0.7071)
            sine_eta1 = np.piecewise(sine_eta1, [sine_eta1 < -1, -1 <= sine_eta1 <= 1, sine_eta1 >= 1],
                                     [-1, lambda sine_eta1: sine_eta1, 1])
            eta1 = math.asin(sine_eta1)
            eta = eta1 + eta2
            print('scene3')

        #  TODO: (c++): eta = math::constrain(eta,-pi/2,pi/2)
        lateral_acc_size = speed * speed / L1_distance * math.sin(eta) * K_L1

        lateral_acc_unit = np.array([self.vel[1], -1*self.vel[0]])/speed
        if np.dot(lateral_acc_unit, vector_AB_unit) > 0 or \
                np.dot(lateral_acc_unit, vector_AB_unit) == 0 < np.dot(lateral_acc_unit, -1 * vector_AP_unit):
            lateral_acc_unit = np.array([-1*self.vel[1], self.vel[0]])/speed

        tangent_acc_unit = self.vel/speed
        tangent_acc_size = (speed - Ex_speed) * K_acct
        tangent_acc = tangent_acc_unit * tangent_acc_size
        lateral_acc = lateral_acc_unit * lateral_acc_size

        acc = lateral_acc + tangent_acc
        self.action[2] = acc[0]
        self.action[4] = acc[1]
        return self.action




