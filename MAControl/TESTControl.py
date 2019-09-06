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
    Select_list = []        # 选择拍卖者的列表，存储距离
    Auctioneer = -1         # 选出的拍卖者编号
    Target_index = -1       # 当前进行拍卖的目标编号
    is_sorted = False       # target_relist是否已进行排序
    target_relist = []      # 拍卖目标按优先级排序列表
    last_step = 0
    Trans_step = []         # 拍卖者发送出目标给竞拍者的延时step列表
    Price_list = []         # 选出的竞拍者发出的竞拍价格
    wait_step = 30          # 等待的时长
    Winner = []             # 最终选出来的优胜者列表
    Update_target_relist = False
    Update_step = 0

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
        self.detect_dis = 0.05
        self.comm_dis = 0.5
        self.close_area = []
        self.trans_step = 0
        # 256×3的航点列表，第3列为航点状态 [0: 无航点] [1: 未飞] [2: pointA] [3: pointB] [4: 已到达]
        self.waypoint_list = [[0 for i in range(3)] for j in range(256)]
        # 小飞机状态 [0: 搜索] [1: 未分配] [2: 已分配] [3: 正在执行]
        TESTControl.Shared_UAV_state.append(0)

        self.ITerm = 0
        self.last_error = 0
        self.action = [0, 0, 0, 0, 0]

        self.BigCheck = False
        TESTControl.Shared_UAV_state.append(0)

    def find_mate(self, obs_n, R):
        selfpos = np.array(obs_n[self.index][2:4])
        close_area = []
        for i in range(len(obs_n)):
            posi = obs_n[i][2:4]
            deltapos = np.sqrt(np.dot(selfpos - posi, selfpos - posi))
            if deltapos < R:
                close_area.append(i)
        self.close_area = close_area
        return self.close_area

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
            for i in range(len(seen_target)):
                TESTControl.Found_Target_Info.append(self.close_area)
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
                    TESTControl.Found_Target_Info.append(self.close_area)

        # COMMUNICATE TESTControl.Found_Target_Info
        for info in TESTControl.Found_Target_Info:
            check = False
            for num in self.close_area:
                check = check | num in info
            if check and (self.index not in info):
                info.append(self.index)

    def PolicyMaker(self, target, obs_n, step, k):

        # 更新小飞机的邻域列表
        self.close_area = self.find_mate(obs_n, self.comm_dis)

        if TESTControl.Shared_UAV_state[k] == 0:
            if TESTControl.Shared_Big_Check is True:
                TESTControl.Shared_UAV_state[k] = 1

                if TESTControl.is_sorted is False:
                    for i in range(len(TESTControl.Found_Target_Set)):
                        TESTControl.target_relist.append([i, TESTControl.Found_Target_Set[i][4], 0])
                    TESTControl.target_relist = sorted(TESTControl.target_relist, key=lambda x: x[1], reverse=True)
                    TESTControl.Target_index = TESTControl.target_relist[0][0]
                    TESTControl.is_sorted = True

                self.PathPlanner(obs_n[k], step)
            else:
                # TODO 进行各种条件的计算判断，输出单个小飞机的大判断计算结果
                self.BigCheck = True if random.random() > 0.95 else False
                self.add_new_target(obs_n[k], target)
                self.PathPlanner(obs_n[k], step)

        elif TESTControl.Shared_UAV_state[k] == 1:

            if (TESTControl.Update_step == step-1) and (TESTControl.Update_target_relist == True):
                if TESTControl.target_relist is not []:
                    TESTControl.Target_index = TESTControl.target_relist[0][0]
                    TESTControl.Update_target_relist = False

            if TESTControl.target_relist is not []:

                # 目标状态为0时进行拍卖者的选择，所有人都会进来
                if TESTControl.target_relist[TESTControl.Target_index][2] == 0:
                    # TODO 添加判断条件，正在执行的小飞机不能被选择
                    if k in TESTControl.Found_Target_Info[TESTControl.Target_index]:
                        TESTControl.Select_list.append([k,
                            math.sqrt((obs_n[k][2]-TESTControl.Found_Target_Set[TESTControl.Target_index][0])**2+
                                      (obs_n[k][3]-TESTControl.Found_Target_Set[TESTControl.Target_index][1])**2)])
                    if k == (len(obs_n)-1):
                        TESTControl.Select_list = sorted(TESTControl.Select_list, key=lambda x: x[1])
                        TESTControl.Auctioneer = TESTControl.Select_list[0][0]
                        TESTControl.target_relist[TESTControl.Target_index][2] = 1

                # 目标状态为1时进行拍卖者生成要发送的step延时,所有人都会进来，看看自己是不是拍卖者，是的话进行操作，确认竞拍者
                elif TESTControl.target_relist[TESTControl.Target_index][2] == 1:
                    if k == TESTControl.Auctioneer:
                        # TODO 添加判断条件，邻域内正在执行的小飞机不能成为竞拍者
                        for i in self.close_area:
                            # TODO 传输延时step个数的计算优化
                            TESTControl.Trans_step.append([i, round(math.sqrt((obs_n[k][2]-obs_n[i][2])**2+(obs_n[k][3]-obs_n[i][3])**2)/0.05)])
                        TESTControl.last_step = step
                        TESTControl.target_relist[TESTControl.Target_index][2] = 2

                # 目标状态为2时更改竞拍者状态为2，在下一个step进行更新
                # TODO 优化
                elif TESTControl.target_relist[TESTControl.Target_index][2] == 2:
                    if TESTControl.last_step == step-1:
                        for i in range(len(TESTControl.Trans_step)):
                            if k == TESTControl.Trans_step[i][0]:
                                TESTControl.Shared_UAV_state[k] = 2
                                self.trans_step = TESTControl.Trans_step[i][1]

            self.PathPlanner(obs_n[k], step)

        # 进入状态2的都是选出来的拍卖者和竞拍者
        elif TESTControl.Shared_UAV_state[k] == 2:

            if TESTControl.target_relist[TESTControl.Target_index][2] == 2:
                # 拍卖者进入判断
                if k == TESTControl.Auctioneer:
                    # 当拍卖者的等待时间完成时，根据价格选择优胜者
                    if TESTControl.wait_step == 0:
                        if TESTControl.Price_list is not []:
                            TESTControl.Price_list = sorted(TESTControl.Price_list, key=lambda x: x[1], reverse=True)
                            if len(TESTControl.Price_list) > TESTControl.Found_Target_Set[TESTControl.Target_index][5]:
                                for i in range(TESTControl.Found_Target_Set[TESTControl.Target_index[5]]):
                                    TESTControl.Winner.append([TESTControl.Price_list[i][0]])
                            else:
                                for i in range(len(TESTControl.Price_list)):
                                    TESTControl.Winner.append([TESTControl.Price_list[i][0]])
                            # 生成优胜者列表后目标状态置为3
                            TESTControl.target_relist[TESTControl.Target_index][2] = 3
                            TESTControl.last_step = step
                    else:
                        TESTControl.wait_step -= 1
                # 所有竞拍者（拍卖者也是竞拍者）进入
                if self.trans_step == 0 and TESTControl.wait_step > 0:
                    # TODO 目前是添加价格，为避免重复添加，添加完之后继续 -1 ，待优化
                    TESTControl.Price_list.append([k, self.auction(obs_n[k], TESTControl.Found_Target_Set)])
                    self.trans_step -= 1
                else:
                    self.trans_step -= 1

            if TESTControl.target_relist[TESTControl.Target_index][2] == 3:
                if TESTControl.last_step == step-1:
                    if TESTControl.Winner is not []:
                        if k in TESTControl.Winner:
                            TESTControl.Shared_UAV_state[k] = 3
                            self.pointBi = (TESTControl.Found_Target_Set[TESTControl.Target_index][0], TESTControl.Found_Target_Set[TESTControl.Target_index][1])
                            TESTControl.Winner.remove(k)
                        else:
                            TESTControl.Shared_UAV_state[k] = 1
                    else:
                        if TESTControl.Update_target_relist is False:
                            self.clearlist(step)

        elif TESTControl.Shared_UAV_state[k] == 3:
            print('Agent_%d is attacking.' % k)

        return self.pointAi, self.pointBi, self.waypoint_finished

    def clearlist(self, step):
        TESTControl.Select_list.clear()
        TESTControl.Trans_step.clear()
        TESTControl.Price_list.clear()
        TESTControl.is_sorted = False
        TESTControl.last_step = 0
        TESTControl.Auctioneer = -1
        TESTControl.target_relist.pop(0)
        TESTControl.Update_target_relist = True
        TESTControl.Update_step = step


    def auction(self, obs, target):
        # TODO 计算当前竞拍价格
        # price = []
        # for i in range(len(shared_info)):
        #     cal_price = [i]
        #     cal_price.append(random.random())
        #     price.append(cal_price)
        # price = sorted(price, key=(lambda x: x[1]), reverse=True)
        #
        # for i in range(target[3]):
        #     winner.append(price[i][0])
        price = random.random()

        return price

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
