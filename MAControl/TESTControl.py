import numpy as np
import math
import os
import MAControl.util as U
import random


class TESTControl(object):

    Found_Target_Set = []
    Found_Target_Info = []
    Shared_UAV_state = []
    Shared_Big_Check = False

    def __init__(self, name, env, world, agent_index, arglist):
        # print("control init")
        self.name = name
        self.env = env
        self.world = world
        self.index = agent_index
        self.arglist = arglist
        self.dt = world.dt

        self.path_pace = 50
        self.motion_pace = 5
        self.inner_pace = 1

        self.pointAi = (0, 0)
        self.pointBi = (0, 0)
        self.tangent_acc = 0
        self.lateral_acc = 0

        self.STE_rate_error = 0
        self.throttle_integ_s = 0

        self.waypoint_finished = False
        self.arrive_flag = False
        self.pointB_index = 0
        self.is_init = True
        self.detect_dis = 50
        # 256×3的航点列表，第3列为航点状态 [0: 无航点] [1: 未飞] [2: pointA] [3: pointB] [4: 已到达]
        self.waypoint_list = [[0 for i in range(3)] for j in range(256)]

        self.ITerm = 0
        self.last_error = 0
        self.action = [0, 0, 0, 0, 0]

        self.BigCheck = False
        TESTControl.Shared_UAV_state.append(0)

    def add_new_target(self, obs, WorldTarget):
        TT_range = 0.05

        # COMPUTE selfview
        selfvel = np.array(obs[0:2])
        selfpos = np.array(obs[2:4])
        selfvelunit = selfvel / np.sqrt(np.dot(selfvel, selfvel))
        selfvelrightunit = np.array([selfvelunit[1], -1 * selfvelunit[0]])
        d1 = 0.2
        d2 = 0.4
        d3 = 0.3
        selfview1 = selfpos + selfvelunit * (d1+d2) - selfvelrightunit * d3/2
        selfview2 = selfpos + selfvelunit * (d1+d2) + selfvelrightunit * d3/2
        selfview3 = selfpos + selfvelunit * d1 + selfvelrightunit * d3/2
        selfview4 = selfpos + selfvelunit * d1 - selfvelrightunit * d3/2

        # GENERATE seen_target
        seen_target = []
        for target in WorldTarget:
            targetpos = np.array(target[1:3])
            if U.point_in_rec(selfview1, selfview2, selfview3, selfview4, targetpos):
                seen_target.append(target)

        # READ AND WRITE TESTControl.Found_Target_Set
        if TESTControl.Found_Target_Set == []:
            TESTControl.Found_Target_Set = seen_target
        elif seen_target != []:
            for target1 in seen_target:
                check = False
                for target2 in TESTControl.Found_Target_Set:
                    pos1 = np.array(target1[1:3])
                    pos2 = np.array(target2[1:3])
                    deltapos = np.sqrt(np.dot(pos1-pos2, pos1-pos2))
                    check = check | (deltapos <= TT_range)
                if not check:
                    TESTControl.Found_Target_Set.append(target1)

    def PolicyMaker(self, target, shared_info, auction_state, step, k):
        # print('make policy')

        self.BigCheck = True if random.random() > 0.95 else False

        if auction_state[k] != 0:
            if auction_state[len(shared_info)] != step:
                self.pointBi = (target[0], target[1])
            else:
                self.PathPlanner(shared_info[k], step)

        else:
            dist_TC = math.sqrt((shared_info[k][2]-target[0])**2 + (shared_info[k][3]-target[1])**2)
            if dist_TC <= self.detect_dis and target[2] < 0:
                winner = self.auction(shared_info, target)
                for i in winner:
                    auction_state[i] = 1
                target[2] = 1
                auction_state[len(shared_info)] = step

            else:
                self.PathPlanner(shared_info[k], step)

        return self.pointAi, self.pointBi, self.waypoint_finished, target, shared_info, auction_state

    def auction(self, shared_info, target):

        price = []
        for i in range(len(shared_info)):
            cal_price = [i]
            cal_price.append(random.random())
            price.append(cal_price)
        price = sorted(price, key=(lambda x: x[1]), reverse=True)

        winner = []
        for i in range(target[3]):
            winner.append(price[i][0])

        return winner

    def PathPlanner(self, obs, step):

        # 初始时刻输出A、B坐标
        if self.pointB_index == 0 and self.is_init is True:
            self.pointAi = (obs[2], obs[3])
            self.pointBi = (self.waypoint_list[self.pointB_index][0], self.waypoint_list[self.pointB_index][1])
            self.is_init = False

        # 更改航点状态并输出A、B坐标
        if self.arrive_flag:
            if self.waypoint_list[self.pointB_index+1][2] != 0 and self.pointB_index < 255:
                if self.pointB_index > 0:
                    self.waypoint_list[self.pointB_index-1][2] = 4
                self.waypoint_list[self.pointB_index][2] = 2
                self.waypoint_list[self.pointB_index+1][2] = 3
                self.pointAi = (self.waypoint_list[self.pointB_index][0], self.waypoint_list[self.pointB_index][1])
                self.pointBi = (self.waypoint_list[self.pointB_index+1][0], self.waypoint_list[self.pointB_index+1][1])
                self.arrive_flag = False
                self.pointB_index += 1
            else:
                for i in range(self.pointB_index+1):
                    self.waypoint_list[i][2] = 1
                self.pointAi = (self.waypoint_list[self.pointB_index][0], self.waypoint_list[self.pointB_index][1])
                self.pointBi = (self.waypoint_list[0][0], self.waypoint_list[0][1])
                self.pointB_index = 0
                # self.waypoint_finished = True

    def MotionController(self, obs, pointAi, pointBi, step):
        # print("motion control")
        vel_vector = np.array(obs[0:2])
        pointPi = np.array(obs[2:4])
        pointAi = np.array(pointAi)
        pointBi = np.array(pointBi)

        # set L1 params
        L1_ratio = 0.1  # (当v=0.05则L1=0.005km=50m)
        BP_range = 0.1  # (0.1km=100m)
        K_L1 = 0.1  # (系数)

        # set tecs params
        TAS_setpoint = 0.05  # (km/s)
        throttle_c = 0  # (%)
        throttle_setpoint_max = 100  # (%)
        throttle_setpoint_min = 1  # (%)
        STE_rate_max = 0.025
        STE_rate_min = -0.025
        K_V = 1  # (系数)
        K_acct = 0.01  # (系数)

        # p-i-d
        Ki_STE = 0.01  # (系数)
        Kp_STE = 0.1  # (系数)
        Kd_STE = 0.0  # (系数)

        # set motion_pace
        if step == 0 or step % self.motion_pace == 0:
            print('motion motion motion motion motion motion motion motion motion motion')

            # # # # # tecs # # # # #

            # compute rate setpoints
            tas_state = speed = np.sqrt(np.square(vel_vector[0]) + np.square(vel_vector[1]))
            TAS_rate_setpoint = (TAS_setpoint - tas_state) * K_V
            STE_error = 0.5 * (TAS_setpoint * TAS_setpoint - tas_state * tas_state)
            STE_rate_setpoint = U.constrain(tas_state * TAS_rate_setpoint, STE_rate_min, STE_rate_max)

            # compute throttle_p
            if STE_rate_setpoint >= 0:
                throttle_p = throttle_c + STE_rate_setpoint / STE_rate_max * (throttle_setpoint_max - throttle_c)
            else:
                throttle_p = throttle_c + STE_rate_setpoint / STE_rate_min * (throttle_setpoint_min - throttle_c)

            # compute throttle_setpoint
            self.STE_rate_error = self.STE_rate_error * 0.8 + STE_rate_setpoint * 0.2
            self.throttle_integ_s = self.throttle_integ_s + STE_error * Ki_STE
            throttle_setpoint = throttle_p + (STE_error + self.STE_rate_error * Kd_STE) * Kp_STE + self.throttle_integ_s
            throttle_setpoint = U.constrain(throttle_setpoint, throttle_setpoint_min, throttle_setpoint_max)

            # tangent_acc
            self.tangent_acc = throttle_setpoint * K_acct

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

            # lateral_acc
            lateral_acc_unit = np.array([vel_vector[1], -1*vel_vector[0]])/speed
            if -0.01 < np.dot(lateral_acc_unit, vector_PC) < 0.01:  # <a1,PC>直角
                lateral_acc_dir = np.sign(np.dot(lateral_acc_unit, vector_AB))
            else:
                lateral_acc_dir = np.sign(np.dot(lateral_acc_unit, vector_PC))
            self.lateral_acc = lateral_acc_size * lateral_acc_dir

        return self.tangent_acc, self.lateral_acc

    def InnerController(self, obs, tangent_acc, lateral_acc, step):
        # print('inner control')

        Exp_lateral_acc = lateral_acc
        True_lateral_acc = np.array(obs[5])
        delta_time = self.dt

        P_value = 0.9
        I_value = 0.01
        D_value = 0.0
        Iterm_window = 1

        error = Exp_lateral_acc - True_lateral_acc
        delta_error = error - self.last_error
        PTerm = error
        DTerm = delta_error / delta_time
        self.ITerm += error * delta_time
        if self.ITerm < -Iterm_window:
            self.ITerm = -Iterm_window
        elif self.ITerm > Iterm_window:
            self.ITerm = Iterm_window
        else:
            self.ITerm = self.ITerm
        self.last_error = error

        acct = tangent_acc
        accl = P_value * PTerm + I_value * self.ITerm + D_value * DTerm
        vel_vector = np.array(obs[0:2])
        speed = np.sqrt(np.square(vel_vector[0]) + np.square(vel_vector[1]))
        vel_right_unit = np.array([vel_vector[1], -1 * vel_vector[0]]) / speed
        acc = acct * vel_vector / speed + accl * vel_right_unit

        self.action[1] = acc[0]
        self.action[3] = acc[1]
        return self.action
