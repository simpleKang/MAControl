from MAControl.Base.PolicyMaker import PolicyMaker
import random
import numpy as np
from MAControl.Util.PointInRec import point_in_rec
from MAControl.Util.Constrain import constrain
import math
import os
import random


class PolicyMaker_Auction(PolicyMaker):

    #                                       (Step2<=)&(<Step3)
    # 搜索目标[阶段] | 排序目标[步] | 选择目标[步] | 出价[阶段] | 统计价格[步] | 分道扬镳[步] | 重置[步] >>>> 搜索目标[阶段] ....
    #  <Step0         ==Step0       ==Step1         ^         ==Step3      ==Step4      ==Step5
    #                                                                        |
    #                                                                        |
    #                                                    InAttacking == True |
    #                                                                        |
    #                                                                     攻击[阶段]

    # 在搜索目标阶段(<Step0)操作，只增不减
    Found_Target_Set = []  # {target_pos, target_vel, target_value, target_defence, target_type, T_INDEX}
    Found_Target_Info = []  # {TARGET:UAV_INDEX}

    # 在重置步(==Step5)操作，只增不减
    Attacked_Target_Index = []  # {TARGET_INDEX}

    # 在分道步(==Step4)操作，只减不增
    Remain_UAV_Set = []  # {UAV_INDEX}

    # 在重置步(==Step5)重置
    Remain_Target_Set = []  # {target_pos, target_vel, target_value, target_defence, target_type, T_INDEX, TARGET_INDEX}
    Current_Target_Index = -1  # {TARGET_INDEX}
    Current_Price_Set = []   # {UAV X STEP}
    Current_Price_Result = []  # {UAV_INDEX,UAV_PRICE}


    def __init__(self, name, env, world, agent_index, arglist):
        super(PolicyMaker_Auction, self).__init__(name, env, world, agent_index, arglist)
        self.opt_index = 0
        self.x = 0
        self.y = 0
        self.InAttacking = False
        self.result = -1
        self.step_now = 0

        # 以下为一些阶段的初始设定步数
        # >> 未来步数点可修改，从而可以主动停留在某一阶段/步
        # >> 进入攻击阶段之后就跳出这部分逻辑而无所谓这些数值
        # >> 进入重置步的时候将所有步数点推到未来避免进入 >Step5 的无定义情况
        self.Step0 = 500
        self.Step1 = 501
        self.Step2 = 502
        self.Step3 = 520
        self.Step4 = 521
        self.Step5 = 522

        self.swarm_size = 0
        self.close_area = []
        self.task_sum = 1
        self.mission_success = 0
        self.over = 0

        self.self_task = []
        for tar in range(len(world.targets)):
            self.self_task.append([])

        self.targetbid = []
        for tar in range(len(world.targets)):
            self.targetbid.append([])

    def find_mate_communcation(self, obs_n, r=0.5):
        selfpos = np.array(obs_n[self.index][2:4])
        close_area = []
        for i in range(len(obs_n)):
            posi = obs_n[i][2:4]
            deltapos = np.sqrt(np.dot(selfpos - posi, selfpos - posi))
            if deltapos < r:
                close_area.append(i)
        return close_area


    def add_new_target(self, obs, WorldTarget, NewController, ttrange=0.05):

        # COMPUTE selfview
        selfvel = np.array(obs[0:2])
        selfpos = np.array(obs[2:4])
        selfvelunit = selfvel / np.sqrt(np.dot(selfvel, selfvel))
        selfdir = math.atan2(selfvel[1], selfvel[0])
        d1 = 0  # 轴向视场距离
        d2 = 0.2  # 轴向视场宽度
        d3 = 0.2  # 侧向视场宽度
        xx1 = -d3/2 * math.cos(selfdir) - d2/2 * math.sin(selfdir) * -1
        xx2 = -d3/2 * math.cos(selfdir) + d2/2 * math.sin(selfdir) * -1
        xx3 = d3/2 * math.cos(selfdir) + d2/2 * math.sin(selfdir) * -1
        xx4 = d3/2 * math.cos(selfdir) - d2/2 * math.sin(selfdir) * -1
        yy1 = -d3/2 * math.sin(selfdir) - d2/2 * math.cos(selfdir)
        yy2 = -d3/2 * math.sin(selfdir) + d2/2 * math.cos(selfdir)
        yy3 = d3/2 * math.sin(selfdir) + d2/2 * math.cos(selfdir)
        yy4 = d3/2 * math.sin(selfdir) - d2/2 * math.cos(selfdir)
        selfview1 = selfpos + selfvelunit * (d1+d2/2) + np.array([xx1, yy1])
        selfview2 = selfpos + selfvelunit * (d1+d2/2) + np.array([xx2, yy2])
        selfview3 = selfpos + selfvelunit * (d1+d2/2) + np.array([xx3, yy3])
        selfview4 = selfpos + selfvelunit * (d1+d2/2) + np.array([xx4, yy4])

        # GENERATE seen_target
        seen_target = []
        for target in WorldTarget:
            targetpos = np.array(target[0:2])
            if point_in_rec(selfview1, selfview2, selfview3, selfview4, targetpos):
                seen_target.append(target)
                truetype = target[-2]
                if truetype == 1:
                    gtype = np.random.choice([1, 2, 3], 1, p=self.arglist.p1)
                    if gtype == 2:
                        seen_target[-1][-4:-1] = [10, 1, 2]
                    elif gtype == 3:
                        seen_target[-1][-4:-1] = [5, 2, 3]
                elif truetype == 2:
                    gtype = np.random.choice([1, 2, 3], 1, p=self.arglist.p2)
                    if gtype == 3:
                        seen_target[-1][-4:-1] = [5, 2, 3]
                    elif gtype == 1:
                        seen_target[-1][-4:-1] = [2, 5, 1]
                elif truetype == 3:
                    gtype = np.random.choice([1, 2, 3], 1, p=self.arglist.p3)
                    if gtype == 1:
                        seen_target[-1][-4:-1] = [2, 5, 1]
                    elif gtype == 2:
                        seen_target[-1][-4:-1] = [10, 1, 2]
                # 在seen_target中，真序号是准确的（唯一标识），类型可能有误（相应的价值和防御能力都有误）


        #更新自身任务矩阵
        for i in range(len(seen_target)):
            if not self.self_task[seen_target[i][-1]]:
                self.self_task[seen_target[i][-1]].append(0)
                self.targetbid[seen_target[i][-1]].append(self.step_now)
                for num_number in range(int(seen_target[i][-3])):
                    self.targetbid[seen_target[i][-1]].append(0)

        #与通信范围内友方更新信息


        # for tar in range(len(WorldTarget)):
        #     num_need_tar = []
        #     friend_index = []
        #     list1 = []
        #     list2 = []
        #     if self.step_now == 400:
        #         print("gg")
        #     for friend in range(len(self.close_area)):
        #         if NewController[self.close_area[friend]][0].targetbid[tar]:
        #             if NewController[self.close_area[friend]][0].targetbid[tar][0] < 100000000:
        #                 num_need_tar.append(len(NewController[self.close_area[friend]][0].targetbid[tar]))
        #                 friend_index.append(self.close_area[friend])
        #     if not num_need_tar:
        #         continue
        #     print(num_need_tar)
        #     a = max(num_need_tar, key=num_need_tar.count)
        #     num_need_tar.remove(a)
        #     if num_need_tar:
        #         b = max(num_need_tar, key=num_need_tar.count)
        #         if a != b:
        #             continue
        #         with open(os.path.dirname(__file__) + '/check.txt', 'a') as f:
        #             f.write(str(a) + str(b) + '\n')
        #     for ii in range(a):
        #         list1.append(0)
        #     list2.extend(self.targetbid[tar])
        #     for iii in range(len(friend_index)):
        #         if not NewController[friend_index[iii]][0].targetbid[tar]:
        #             continue
        #         list2.extend(NewController[friend_index[iii]][0].targetbid[tar])
        #     sorted(set(list2), key=list2.index)
        #     self.targetbid[tar] = []
        #     for iiii in range(a):
        #         if a >8:
        #             print('gg')
        #         if iiii < len(list2):
        #             self.targetbid[tar].append(list2[iiii])
        #         else:
        #             self.targetbid[tar].append(0)
        #     if len(self.targetbid[tar]) > a:
        #         print("gg")



        for num in range(len(self.close_area)):
            for index_tar in range(len(WorldTarget)):
                list1 = list()
                gg = len(NewController[self.close_area[num]][0].targetbid[index_tar])
                ggg = NewController[self.close_area[num]][0].targetbid[index_tar]
                if len(NewController[self.close_area[num]][0].targetbid[index_tar]) != 0:
                    if self.targetbid[index_tar]:
                        if NewController[self.close_area[num]][0].targetbid[index_tar][0]:
                           if NewController[self.close_area[num]][0].targetbid[index_tar][0] <= self.targetbid[index_tar][0]:
                                for i in range(len(NewController[self.close_area[num]][0].targetbid[index_tar])):
                                    list1.append(NewController[self.close_area[num]][0].targetbid[index_tar][i])
                                list1.extend(self.targetbid[index_tar])
                                list1.remove(NewController[self.close_area[num]][0].targetbid[index_tar][0])
                                list1.remove(self.targetbid[index_tar][0])
                                a = len(list1)
                                while 0 in list1:
                                    list1.remove(0)
                                b = len(list1)
                                list1 = sorted(set(list1),key=list1.index)
                                for iii in range(a-b):
                                    list1.append(0)
                                list1.sort(reverse=True)
                                kk = len(NewController[self.close_area[num]][0].targetbid[index_tar]) - 1
                                k = NewController[self.close_area[num]][0].targetbid[index_tar][0]
                                self.targetbid[index_tar] = []
                                self.targetbid[index_tar].append(k)
                                for ii in range(kk):
                                    if ii < len(list1):
                                        self.targetbid[index_tar].append(list1[ii])
                                    else:
                                        self.targetbid[index_tar].append(0)
                                if not self.self_task[index_tar]:
                                    self.self_task[index_tar].append(0)
                           else:
                               for i in range(len(NewController[self.close_area[num]][0].targetbid[index_tar])):
                                   list1.append(NewController[self.close_area[num]][0].targetbid[index_tar][i])
                               list1.extend(self.targetbid[index_tar])
                               list1.remove(NewController[self.close_area[num]][0].targetbid[index_tar][0])
                               list1.remove(self.targetbid[index_tar][0])
                               # list1 = sorted(set(list1), key=list1.index)
                               a = len(list1)
                               while 0 in list1:
                                   list1.remove(0)
                               b = len(list1)
                               list1 = sorted(set(list1), key=list1.index)
                               for iii in range(a - b):
                                   list1.append(0)
                               list1.sort(reverse=True)
                               kk = len(self.targetbid[index_tar])-1
                               kkk = self.targetbid[index_tar][0]
                               self.targetbid[index_tar] = [kkk]
                               for ii in range(kk):
                                   if ii < len(list1):
                                       self.targetbid[index_tar].append(list1[ii])
                                   else:
                                       self.targetbid[index_tar].append(0)
                               if not self.self_task[index_tar]:
                                   self.self_task[index_tar].append(0)
                    else:
                        for i in range(len(NewController[self.close_area[num]][0].targetbid[index_tar])):
                            self.targetbid[index_tar].append(NewController[self.close_area[num]][0].targetbid[index_tar][i])
                        if not self.self_task[index_tar]:
                            self.self_task[index_tar].append(0)


    def bidding(self, obs, WorldTarget):

        # Pr = U - C (Pr为最终出价, U为收益, C为成本)

        # 收益U的相关参数
        x = 1         # 表示是否用于打击目标 0 or 1
        e1 = 0.5      # 我方小飞机优势系数
        e2 = 0.5      # 敌方目标战术价值系数  e1 + e2 = 1 (0 <= e1, e2 <= 1)
        pt = 0.8      # 小飞机单发杀伤概率

        #已发现目标的战术价值
        W = []
        for num in range(len(self.self_task)):
            if self.self_task[num]:
                W.append(WorldTarget[num][-4])
            else:
                W.append(0)

        sigma1 = 0.5  # 距离优势系数
        sigma2 = 0.5  # 角度优势系数
        D = 0.6       # 小飞机能够攻击目标的最大距离

        # 成本C的相关参数
        s = 0.5       # 小飞机的代价系数
        T = 0.5       # 敌方目标对小飞机的威胁程度
        pt_ = 0.6     # 目标的单发杀伤概率

        # 计算中间变量
        P = []
        for num in range(len(self.self_task)):
            if self.self_task[num]:
                dis = math.sqrt((WorldTarget[num][0] - obs[2])**2+
                                (WorldTarget[num][1] - obs[3])**2)
                v_unit = np.array([obs[0], obs[1]])/math.sqrt(obs[0]**2+obs[1]**2)
                t_unit = np.array([WorldTarget[num][0] - obs[2],
                                   WorldTarget[num][1] - obs[3]])/dis
                angle = math.acos(constrain(np.dot(v_unit, t_unit), -1, 1))
                Fd = math.exp(1 - dis / D)
                Fq = math.exp(1 - angle/math.pi)
                P_one = (sigma1 * Fd + sigma2 * Fq)*(self.step_now - 500)**1.01
                P.append(P_one)
            else:
                P.append(0)

        # 计算收益U
        U = []
        for num in range(len(P)):
            U_one = (e1 * P[num] + e2 * W[num]) * (1 - (1 - pt)**x)
            U.append(U_one)

        # 计算成本C
        C = s * T * pt_

        # 最终出价P
        Pr = []
        for num in range(len(U)):
            Pr_one = U[num] - C
            Pr.append([num, Pr_one])
        Pr.sort(reverse=True, key=lambda x:x[1] )

        return Pr

    def make_policy(self, WorldTarget, obs_n, step, NewController):

        if self.over == 0:

            dis = np.sqrt((WorldTarget[self.self_task.index(max(self.self_task))][0] - obs_n[self.index][2])**2
                          + (WorldTarget[self.self_task.index(max(self.self_task))][1] - obs_n[self.index][3])**2)
            if dis <= 0.1 and max(self.self_task) == [1]:
                self.mission_success = 1
            if self.mission_success and self.opt_index == 10:
                kkkkk = []
                a = self.self_task.index(max(self.self_task))
                for tar in range(len(WorldTarget)):
                    kkkkk.append([])
                kkkkk[a].append(10000000)
                kkkkk[a].append(random.randint(100000000, 1000000000000))
                self.targetbid = kkkkk.copy()
                self.over = 1

            else:
                self.step_now = step

                if step < self.Step0:
                    # print('UAV', self.index, 'searching')
                    self.close_area = self.find_mate_communcation(obs_n).copy()
                    self.add_new_target(obs_n[self.index], WorldTarget, NewController)
                    self.opt_index = 0

                #elif ((step % 2) == 0 and self.mission_success == 0) or ((step % 2) == 1 and max(self.self_task) == [0] and self.mission_success == 0):
                else:
                    if max(self.self_task) == [0]:
                        self.opt_index = 0
                    self.close_area = self.find_mate_communcation(obs_n).copy()
                    self.add_new_target(obs_n[self.index], WorldTarget, NewController)
                    self_bid = self.bidding(obs_n[self.index], WorldTarget)
                    for i in range(len(self_bid)):
                        if self.targetbid[self_bid[i][0]]:
                            kkk = self.targetbid[self_bid[i][0]].copy()
                            kkk.pop(0)
                            if min(kkk) < self_bid[i][1]:
                                a = self_bid[i]
                                self.targetbid[self_bid[i][0]][kkk.index(min(kkk))+1] = a[1]
                                for j in range(len(self.self_task)):
                                    if self.self_task[j]:
                                        self.self_task[j][0] = 0
                                self.self_task[self_bid[i][0]][0] = 1
                                break
                            else:
                                # print(self_bid[i][0])
                                # print(self.self_task)
                                if self.self_task[self_bid[i][0]]:
                                    self.self_task[self_bid[i][0]][0] = 0
                                else:
                                    self.self_task[self_bid[i][0]].append(0)
                    self.InAttacking = True
                    if max(self.self_task) == [1]:
                        self.x = WorldTarget[self.self_task.index(max(self.self_task))][0]
                        self.y = WorldTarget[self.self_task.index(max(self.self_task))][1]
                        self.result = WorldTarget[self.self_task.index(max(self.self_task))][7]
                        self.opt_index = 10

                    else:
                        self.InAttacking = False

                    if self.opt_index == 10 and max(self.self_task) == [0]:
                        self.opt_index = 1

                    # print(self_bid)
                    with open(os.path.dirname(__file__) + '/check.txt', 'a') as f:
                        f.write(str(self.step_now) + '\n' + str(self_bid) + '\n')
                # print(self.index)
                # print(self.self_task)
                # print(self.targetbid)
        with open(os.path.dirname(__file__) + '/check.txt', 'a') as f:
            f.write(str(step) + '\n' + str(self.index) + '\n' + str(self.self_task) + '\n'
                    + str(self.targetbid) + '\n' + str(self.close_area) + '\n')

        return [self.opt_index, [self.x, self.y, self.result, self.mission_success]]
