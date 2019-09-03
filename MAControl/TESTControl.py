import numpy as np
import math
import os
import MAControl.util as U


class TESTControl():
    def __init__(self, name, env, world, agent_index, arglist):
        print("control init")
        self.name = name
        self.env = env
        self.world = world
        self.index = agent_index
        self.arglist = arglist
        self.dt = world.dt

        self.path_pace = 20
        self.motion_pace = 5
        self.inner_pace = 1

        self.pointAi = (0, 0)
        self.pointBi = (0, 0)

        self.STE_rate_error = 0
        self.throttle_integ_state = 0
        self.action = [0, 0, 0, 0, 0]

        self.waypoint_finished = False
        self.arrive_flag = False
        self.pointB_index = 0
        self.is_init = True
        # 256×3的航点列表，第3列为航点状态 [0: 无航点] [1: 未飞] [2: pointA] [3: pointB] [4: 已到达]
        self.waypoint_list = [[0 for i in range(3)] for j in range(256)]

    def PathPlanner(self, obs,step):
        # print("path plan")

        # TODO:根据obs进行判断是否修改航点列表
        # if True:
        #     self.WaypointUpdater(obs)

        # 初始时刻输出A、B坐标
        if self.pointB_index == 0 and self.is_init is True:
            self.pointA = (obs[2], obs[3])
            self.pointB = (self.waypoint_list[self.pointB_index][0], self.waypoint_list[self.pointB_index][1])
            self.is_init = False

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
                for i in range(self.pointB_index+1):
                    self.waypoint_list[i][2] = 1
                self.pointA = (self.waypoint_list[self.pointB_index][0], self.waypoint_list[self.pointB_index][1])
                self.pointB = (self.waypoint_list[0][0], self.waypoint_list[0][1])
                self.pointB_index = 0
                # self.waypoint_finished = True

        return self.pointA, self.pointB, self.waypoint_finished

    def MotionController(self, obs, pointAi, pointBi,step):
        # print("motion control")
        vel_vector = np.array(obs[0:2])
        pointPi = np.array(obs[2:4])
        pointAi = np.array(pointAi)
        pointBi = np.array(pointBi)
        print('pointAi', pointAi)
        print('pointBi', pointBi)

        # set L1 params
        L1_ratio = 0.1  # (当v=0.05则L1=0.005km=50m)
        BP_range = 0.1  # (0.1km=100m)
        K_L1 = 0.1  # (系数)

        # set tecs params
        TAS_setpoint = 0.05  # (km/s)
        throttle_c = 0  # (%)
        throttle_setpoint_max = 100  # (%)
        throttle_setpoint_min = 0  # (%)
        STE_rate_max = 0.025
        STE_rate_min = -0.025
        K_V = 1  # (系数)
        K_acct = 0.1  # (系数)

        # p-i-d
        Ki_STE = 0  # (系数)
        Kp_STE = 0.8  # (系数)
        Kd_STE = 0.1  # (系数)

        # # # # # tecs # # # # #
        # compute rate setpoints
        tas_state = speed = np.sqrt(np.square(vel_vector[0]) + np.square(vel_vector[1]))
        TAS_rate_setpoint = (TAS_setpoint - tas_state) * K_V
        STE_error = 0.5 * (TAS_setpoint * TAS_setpoint - tas_state * tas_state)
        STE_rate_setpoint = U.constrain(tas_state * TAS_rate_setpoint, STE_rate_min, STE_rate_max)
        print('speed', speed)

        # compute throttle_p
        if STE_rate_setpoint >= 0:
            throttle_p = throttle_c + STE_rate_setpoint / STE_rate_max * (throttle_setpoint_max - throttle_c)
        else:
            throttle_p = throttle_c + STE_rate_setpoint / STE_rate_min * (throttle_setpoint_min - throttle_c)

        # compute throttle_setpoint
        self.STE_rate_error = self.STE_rate_error * 0.8 + STE_rate_setpoint * 0.2
        self.throttle_integ_state = self.throttle_integ_state + STE_error * Ki_STE
        throttle_setpoint = throttle_p + (STE_error + self.STE_rate_error * Kd_STE) * Kp_STE + self.throttle_integ_state
        throttle_setpoint = U.constrain(throttle_setpoint, throttle_setpoint_min, throttle_setpoint_max)

        # # # # # L1 # # # # #
        # compute L1
        L1_distance = speed * L1_ratio

        # compute AB
        vector_AB = pointBi-pointAi
        dist_AB = np.sqrt(np.square(vector_AB[0]) + np.square(vector_AB[1]))
        dist_AB = max(dist_AB, 0.000000001)
        vector_AB_unit = vector_AB/dist_AB

        # compute AP
        vector_AP = pointPi-pointAi
        dist_AP = np.sqrt(np.square(vector_AP[0]) + np.square(vector_AP[1]))
        dist_AP = max(dist_AP, 0.000000001)
        vector_AP_unit = vector_AP/dist_AP

        # compute BP
        vector_BP = pointPi - pointBi
        dist_BP = np.sqrt(np.square(vector_BP[0]) + np.square(vector_BP[1]))
        dist_BP = max(dist_BP, 0.000000001)
        if dist_BP < BP_range:
            self.arrive_flag = True
            print('True', obs)
        else:
            self.arrive_flag = False
        vector_BP_unit = vector_BP/dist_BP

        # extra computation
        alongTrackDist = np.dot(vector_AP, vector_AB_unit)
        AB_to_BP_bearing = math.acos(U.constrain(np.dot(vector_AB_unit, vector_BP_unit), -1, 1))

        if dist_AP > L1_distance and alongTrackDist/dist_AP < -0.707:
            # calculate eta to fly to waypoint A
            eta = math.acos(U.constrain(np.dot(-1 * vector_AP_unit, vel_vector/speed), -1, 1))

        elif abs(AB_to_BP_bearing) < math.radians(100):
            # calculate eta to fly to waypoint B
            eta = math.acos(np.dot(-1 * vector_BP_unit, vel_vector/speed))

        else:
            # calculate eta to fly along the line between A and B
            eta2 = math.acos(U.constrain(np.dot(vector_AB_unit, vel_vector/speed), -1, 1))
            beta = math.acos(U.constrain(np.dot(vector_AP_unit, vector_AB_unit), -1, 1))
            xtrackErr = dist_AP * math.sin(beta)
            eta1 = math.asin(U.constrain(xtrackErr / L1_distance, -0.7071, 0.7071))
            eta = eta1 + eta2

        # eta
        eta = U.constrain(eta, -1.5708, 1.5708)
        lateral_acc_size = speed * speed / L1_distance * math.sin(eta) * K_L1

        # pointC
        vector_CB = np.dot(-1 * vector_BP, vector_AB_unit) * vector_AB_unit
        pointCi = pointBi - vector_CB
        vector_PC = pointCi - pointPi
        dist_PC = np.sqrt(np.square(vector_PC[0]) + np.square(vector_PC[1]))
        dist_PC = max(dist_PC, 0.000000001)
        vector_PC_unit = vector_PC / dist_PC

        # lateral_acc
        lateral_acc_unit = np.array([vel_vector[1], -1*vel_vector[0]])/speed
        if abs(np.dot(lateral_acc_unit, vector_AB)) > 0.99:  # acc // AB
            lateral_acc_unit = vector_AB_unit
        elif abs(np.dot(lateral_acc_unit, vector_AB)) < 0.01:  # acc _|_ AB
            lateral_acc_unit = vector_PC_unit
        elif np.dot(lateral_acc_unit, vector_PC) < -0.01:
            lateral_acc_unit = np.array([-1*vel_vector[1], vel_vector[0]])/speed

        lateral_acc = lateral_acc_unit * lateral_acc_size
        # TODO: lateral_acc smoothed by pid
        print('lateral_acc', lateral_acc)

        # tangent_acc
        tangent_acc_unit = vel_vector/speed
        tangent_acc_size = throttle_setpoint * K_acct
        tangent_acc = tangent_acc_unit * tangent_acc_size
        print('tangent_acc', tangent_acc)

        # action
        acc = lateral_acc + tangent_acc
        return acc

    def InnerController(self,obs,Exp_acc,step):
        print('innercontroller')
        acc = Exp_acc
        self.action[1] = acc[0]
        self.action[3] = acc[1]
        return self.action




