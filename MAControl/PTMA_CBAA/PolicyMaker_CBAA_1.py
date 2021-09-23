
# 在cbaa的基础上，优化了共识过程中的通信

from MAControl.Base.PolicyMaker import PolicyMaker
import numpy as np
from MAControl.Util.PointInRec import point_in_rec
import math
import random


class PolicyMaker_Auction(PolicyMaker):

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

        self.random_value = 0

        self.self_target = [[] for t in range(len(world.targets))]
        self.targetbid = [[] for t in range(len(world.targets))]
        self.self_bid = []

    def find_mate(self, obs_n, r=0.5):
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
                        seen_target[-1][-5:-1] = [3, 4, 0, 2]
                    elif gtype == 3:
                        seen_target[-1][-5:-1] = [9, 2, 3, 3]
                elif truetype == 2:
                    gtype = np.random.choice([1, 2, 3], 1, p=self.arglist.p2)
                    if gtype == 3:
                        seen_target[-1][-5:-1] = [9, 2, 3, 3]
                    elif gtype == 1:
                        seen_target[-1][-5:-1] = [7, 3, 2, 1]
                elif truetype == 3:
                    gtype = np.random.choice([1, 2, 3], 1, p=self.arglist.p3)
                    if gtype == 1:
                        seen_target[-1][-5:-1] = [7, 3, 2, 1]
                    elif gtype == 2:
                        seen_target[-1][-5:-1] = [3, 4, 0, 2]
                # 在 seen_target 中，真序号是准确的（唯一标识），类型可能有误（相应的价值和防御能力都有误）

        # 更新自身任务矩阵
        for i in range(len(seen_target)):
            if not self.self_target[seen_target[i][-1]]:
                # noinspection PyTypeChecker
                self.self_target[seen_target[i][-1]].append(0)
                # noinspection PyTypeChecker
                self.targetbid[seen_target[i][-1]].append(self.step_now)
                for nn in range(int(seen_target[i][3])+int(seen_target[i][4])):
                    self.targetbid[seen_target[i][-1]].append([0, 0, 0])

        for num in range(len(self.close_area)):
            for index_tar in range(len(WorldTarget)):
                list1 = list()
                ggg = NewController[self.close_area[num]][0].targetbid[index_tar]
                gg = len(ggg)  # possibly da+db+1
                # 如果友方个体目标参数处的出价不为空
                if gg != 0:
                    if self.targetbid[index_tar]:
                        if ggg[0]:
                            if ggg[0] <= self.targetbid[index_tar][0]:  # 时间 (self 不早于 mate )
                                for i in range(len(ggg)):
                                    list1.append(ggg[i])
                                list1.extend(self.targetbid[index_tar])
                                list1.remove(ggg[0])  # remove time from mate
                                list1.remove(self.targetbid[index_tar][0])  # remove time from self
                                a = len(list1)  # 除零前
                                while [0, 0, 0] in list1:
                                    list1.remove([0, 0, 0])
                                b = len(list1)  # 除零后
                                list1.sort(reverse=True, key=lambda x: x[1])
                                list2 = []
                                for unit_list1 in list1:
                                    if unit_list1 not in list2:
                                        list2.append(unit_list1)  # 不重复
                                list3 = []
                                for unit_list2 in list2:
                                    if not list3:
                                        list3.append(unit_list2)  # 空的时候直接加
                                    else:
                                        key = 0
                                        for unit in range(len(list3)):
                                            if unit_list2[0] == list3[unit][0]:
                                                key = 1
                                        if key == 0:  # 如果key仍然是0就加
                                            list3.append(unit_list2)
                                        for unit in range(len(list3)):
                                            if unit_list2[0] == list3[unit][0] and unit_list2[2] > list3[unit][2]:
                                                list3.remove(list3[unit])
                                                list3.append(unit_list2)
                                for di in range(a-b):
                                    list3.append([0, 0, 0])
                                list3.sort(reverse=True, key=lambda x: x[1])
                                kk = gg - 1  # possibly da+db
                                self.targetbid[index_tar] = []  # reset
                                self.targetbid[index_tar].append(ggg[0])   # add time
                                for ii in range(kk):
                                    if ii < len(list3):
                                        self.targetbid[index_tar].append(list3[ii])
                                    else:
                                        self.targetbid[index_tar].append([0, 0, 0])
                                if not self.self_target[index_tar]:
                                    self.self_target[index_tar].append(0)
                            else:  # 时间 (self 早于 mate)
                                for i in range(gg):
                                    list1.append(ggg[i])
                                list1.extend(self.targetbid[index_tar])
                                list1.remove(ggg[0])
                                list1.remove(self.targetbid[index_tar][0])
                                list2 = []
                                for unit_list1 in list1:
                                    if unit_list1 not in list2:
                                        list2.append(unit_list1)
                                a = len(list2)  # 除零前
                                while [0, 0, 0] in list2:
                                    list2.remove([0, 0, 0])
                                b = len(list2)  # 除零后
                                list3 = []
                                for unit_list2 in list2:
                                    if not list3:
                                        list3.append(unit_list2)
                                    else:
                                        key = 0
                                        for unit in range(len(list3)):
                                            if unit_list2[0] == list3[unit][0]:
                                                key = 1
                                        if key == 0:
                                            list3.append(unit_list2)
                                        for unit in range(len(list3)):
                                            if unit_list2[0] == list3[unit][0] and unit_list2[2] > list3[unit][2]:
                                                list3.remove(list3[unit])
                                                list3.append(unit_list2)
                                for di in range(a - b):
                                    list3.append([0, 0, 0])
                                list3.sort(reverse=True)  # ? no key?
                                kk = len(self.targetbid[index_tar]) - 1  # possibly da+db
                                time_k = self.targetbid[index_tar][0]  # time
                                self.targetbid[index_tar] = [time_k]
                                for ii in range(kk):
                                    if ii < len(list3):
                                        self.targetbid[index_tar].append(list3[ii])
                                    else:
                                        self.targetbid[index_tar].append([0, 0, 0])
                                if not self.self_target[index_tar]:
                                    self.self_target[index_tar].append(0)
                    else:
                        for i in range(len(ggg)):
                            self.targetbid[index_tar].append(ggg[i])
                        if not self.self_target[index_tar]:  # 0、[] >>> false # [0] >>> true
                            self.self_target[index_tar].append(0)

        # 将targetbid中的所有相同编号的变成一个
        list5 = []
        for i in range(len(self.targetbid)):
            for j in range(len(self.targetbid[i])):
                u = str([self.targetbid[i][j], i, j])
                u = u.replace('[', '')
                u = u.replace(']', '')
                a = list(eval(u))
                list5.append(a)
        list4 = []
        for unit_list5 in list5:
            signal = 0
            if not list4:
                list4.append(unit_list5)
            for unit_list4 in list4:
                if unit_list5[0] == unit_list4[0] and unit_list5[2] != unit_list4[2] \
                        and len(unit_list4) == 5 and len(unit_list5) == 5:  # 5? 怎么出来的5?
                    if unit_list5[2] > unit_list4[2]:
                        aa = unit_list4[-2]  # type
                        self.targetbid[aa].remove(unit_list4[0:3])
                        self.targetbid[aa].append([0, 0, 0])
                        signal = 1
                        list4.remove(unit_list4)
                    else:
                        aa = unit_list5[-2]
                        self.targetbid[aa].remove(unit_list5[0:3])
                        self.targetbid[aa].append([0, 0, 0])
                        signal = 2
                    break
            list4.append(unit_list5)
            if signal == 2:
                list4.remove(unit_list5)

    def bidding(self, obs, WorldTarget):

        # Pr = U - C (Pr为最终出价, U为收益, C为成本)

        # 收益U的相关参数

        e1 = 0.5      # 我方小飞机优势系数
        e2 = 0.5      # 敌方目标战术价值系数  e1 + e2 = 1 (0 <= e1, e2 <= 1)
        pt = 0.8      # 小飞机单发杀伤概率

        W = []
        for num in range(len(self.self_target)):  # 这肯定就是WorldTarget 即使有的项是[]
            if self.self_target[num]:
                W.append(WorldTarget[num][2])
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
        for num in range(len(self.self_target)):  # 这肯定就是WorldTarget 即使有的项是[]
            if self.self_target[num]:
                delta_pos = np.array(WorldTarget[num][0:2]) - np.array(obs[2:4])
                dis = math.sqrt(delta_pos[0]**2 + delta_pos[1]**2)
                v_dir = math.atan2(obs[1], obs[0])
                t_dir = math.atan2(delta_pos[1], delta_pos[0])
                angle = abs(v_dir - t_dir)
                Fd = math.exp(1 - dis / D)
                Fq = math.exp(1 - angle/math.pi)
                P_one = (sigma1 * Fd + sigma2 * Fq)
                P.append(P_one)
            else:
                P.append(0)

        # 计算收益U
        U = []
        for num in range(len(P)):  # 在W/P都是0的时候，U直接套公式就是0
            U_one = (e1 * P[num] + e2 * W[num]) * pt
            U.append(U_one)

        # 计算成本C
        C = s * T * pt_

        # 最终出价P
        Pr = []
        for num in range(len(U)):
            Pr_one = U[num] - C
            Pr.append([num, Pr_one])
        Pr.sort(reverse=True, key=lambda x: x[1])  # 根据Pr_one排序

        return Pr

    def make_policy(self, WorldTarget, obs_n, step, NewController):

        if self.over == 0:

            delta_pos = WorldTarget[self.self_target.index(max(self.self_target))][0:2] - obs_n[self.index][2:4]
            dis = math.sqrt(delta_pos[0] ** 2 + delta_pos[1] ** 2)

            if dis <= 0.1 and max(self.self_target) == [1]:
                self.mission_success = 1

            if self.mission_success and self.opt_index == 10:
                kkkkk = []
                a = self.self_target.index(max(self.self_target))
                for tar in range(len(WorldTarget)):
                    kkkkk.append([])
                kkkkk[a].append(10000000)
                kkkkk[a].append([self.index, random.randint(100000000, 1000000000000), 1000000])
                self.targetbid = kkkkk.copy()
                self.over = 1

            else:
                self.step_now = step

                if step < self.Step0:
                    self.close_area = self.find_mate(obs_n).copy()
                    self.add_new_target(obs_n[self.index], WorldTarget, NewController)
                    self.opt_index = 0

                elif step % 10 == 0:
                    if max(self.self_target) == [0]:
                        self.opt_index = 0
                    self.self_bid = self.bidding(obs_n[self.index], WorldTarget)  # 在此赋值

                else:
                    if max(self.self_target) == [0]:
                        self.opt_index = 0
                    self.close_area = self.find_mate(obs_n).copy()
                    self.add_new_target(obs_n[self.index], WorldTarget, NewController)
                    for i in range(len(self.self_bid)):  # 长度是len(WorldTarget)
                        # 比较self出价与竞标价格
                        if self.targetbid[self.self_bid[i][0]]:
                            kkk = self.targetbid[self.self_bid[i][0]].copy()
                            kkk_mirror = []
                            for unit in range(len(kkk)):
                                kkk_mirror.append(kkk[unit])
                            for vnit in range(len(kkk_mirror)-1):
                                if kkk_mirror[vnit+1][1] == 0:
                                    kkk.remove(kkk_mirror[vnit+1])
                            kkk.remove(self.targetbid[self.self_bid[i][0]][0])

                            # 查看self.targetbid中的index_tar是否已经有self.index
                            signal = 666
                            # 本个体是否已经在targetbid中，若不在，则signal=666
                            for unit in range(len(kkk)):
                                if kkk[unit][0] == self.index:
                                    signal = unit

                            if not kkk and self.self_bid[i][1] > 0:
                                a = self.self_bid[i]
                                self.targetbid[self.self_bid[i][0]][1] = [self.index, a[1], self.step_now]
                                for j in range(len(self.self_target)):
                                    if self.self_target[j]:
                                        self.self_target[j][0] = 0
                                # noinspection PyTypeChecker
                                self.self_target[a[0]][0] = 1
                                break

                            elif kkk:
                                min1_kkk = min(kkk, key=lambda x: x[1])
                                if kkk[kkk.index(min1_kkk)][1] < self.self_bid[i][1] and signal == 666:
                                    a = self.self_bid[i]
                                    self.targetbid[a[0]][kkk.index(min1_kkk) + 1] = [self.index, a[1], self.step_now]
                                    for j in range(len(self.self_target)):
                                        if self.self_target[j]:
                                            self.self_target[j][0] = 0
                                    # noinspection PyTypeChecker
                                    self.self_target[a[0]][0] = 1
                                    break

                                elif signal != 666:
                                    a = self.self_bid[i]
                                    self.targetbid[a[0]][signal+1] = [self.index, a[1], self.step_now]
                                    for j in range(len(self.self_target)):
                                        if self.self_target[j]:
                                            self.self_target[j][0] = 0
                                    # noinspection PyTypeChecker
                                    self.self_target[a[0]][0] = 1
                                    break

                                else:
                                    if self.self_target[self.self_bid[i][0]]:
                                        # noinspection PyTypeChecker
                                        self.self_target[self.self_bid[i][0]][0] = 0
                                    else:
                                        # noinspection PyTypeChecker
                                        self.self_target[self.self_bid[i][0]].append(0)

                            else:
                                if self.self_target[self.self_bid[i][0]]:
                                    # noinspection PyTypeChecker
                                    self.self_target[self.self_bid[i][0]][0] = 0
                                else:
                                    # noinspection PyTypeChecker
                                    self.self_target[self.self_bid[i][0]].append(0)

                        else:
                            pass

                    self.InAttacking = True
                    if max(self.self_target) == [1]:
                        self.x = WorldTarget[self.self_target.index([1])][0]
                        self.y = WorldTarget[self.self_target.index([1])][1]
                        self.result = WorldTarget[self.self_target.index([1])][7]
                        self.opt_index = 10

                    else:
                        self.InAttacking = False

                    if self.opt_index == 10 and max(self.self_target) == [0]:
                        self.opt_index = 1

        return [self.opt_index, [self.x, self.y, self.result, self.mission_success]]
