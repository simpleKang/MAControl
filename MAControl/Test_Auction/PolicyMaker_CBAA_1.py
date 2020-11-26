
# 在cbaa的基础上，优化了共识过程中的通信

from MAControl.Base.PolicyMaker import PolicyMaker
import numpy as np
from MAControl.Util.PointInRec import point_in_rec
from MAControl.Util.Constrain import constrain
import math
import os
import random
import numpy.matlib
import scipy.stats as stats

class PolicyMaker_Auction(PolicyMaker):

    #                                       (Step2<=)&(<Step3)
    # 搜索目标[阶段] | 排序目标[步] | 选择目标[步] | 出价[阶段] | 统计价格[步] | 分道扬镳[步] | 重置[步] >>>> 搜索目标[阶段] ....
    #  <Step0         ==Step0       ==Step1         ^         ==Step3      ==Step4      ==Step5
    #                                                                        |
    #                                                                        |
    #                                                    InAttacking == True |
    #                                                                        |
    #                                                                     攻击[阶段]

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

        #高斯分布时sigma = self.proportion * mu
        self.proportion = 1

        self.random_value = 0

        self.self_task = [[] for t in range(len(world.targets))]
        self.targetbid = [[] for t in range(len(world.targets))]
        self.self_bid = []

    def find_mate_communication(self, obs_n, r=0.5):
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


                            # seen_target = []
        # for target in WorldTarget:
        #     targetpos = np.array(target[0:2])
        #     if point_in_rec(selfview1, selfview2, selfview3, selfview4, targetpos):
        #         seen_target.append(target)
        #         truetype = target[-2]
        #         if truetype == 1:
        #             gtype = np.random.choice([1, 2, 3], 1, p=self.arglist.p1)
        #             if gtype == 2:
        #                 seen_target[-1][-4:-1] = [10, 1, 2]
        #             elif gtype == 3:
        #                 seen_target[-1][-4:-1] = [5, 2, 3]
        #         elif truetype == 2:
        #             gtype = np.random.choice([1, 2, 3], 1, p=self.arglist.p2)
        #             if gtype == 3:
        #                 seen_target[-1][-4:-1] = [5, 2, 3]
        #             elif gtype == 1:
        #                 seen_target[-1][-4:-1] = [2, 5, 1]
        #         elif truetype == 3:
        #             gtype = np.random.choice([1, 2, 3], 1, p=self.arglist.p3)
        #             if gtype == 1:
        #                 seen_target[-1][-4:-1] = [2, 5, 1]
        #             elif gtype == 2:
        #                 seen_target[-1][-4:-1] = [10, 1, 2]
                # 在seen_target中，真序号是准确的（唯一标识），类型可能有误（相应的价值和防御能力都有误）

        # 更新自身任务矩阵
        for i in range(len(seen_target)):
            if not self.self_task[seen_target[i][-1]]:
                self.self_task[seen_target[i][-1]].append(0)
                self.targetbid[seen_target[i][-1]].append(self.step_now)
                for num_number in range(int(seen_target[i][-3])):
                    self.targetbid[seen_target[i][-1]].append([0,0,0])

        for num in range(len(self.close_area)):
            for index_tar in range(len(WorldTarget)):
                list1 = list()
                gg = len(NewController[self.close_area[num]][0].targetbid[index_tar])
                ggg = NewController[self.close_area[num]][0].targetbid[index_tar]
                # 如果友方个体目标参数处的出价不为0
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
                                while [0, 0,0] in list1:
                                    list1.remove([0, 0,0])
                                b = len(list1)
                                list2 = []
                                for unit_list1 in list1:
                                    if unit_list1 not in list2:
                                        list2.append(unit_list1)
                                list3 = []
                                for unit_list2 in list2:
                                    if not list3:
                                        list3.append(unit_list2)
                                    else:
                                        key = 0
                                        for index_1 in range(len(list3)):
                                            if unit_list2[0] == list3[index_1][0]:
                                                key = 1
                                        if key == 0:
                                            list3.append(unit_list2)
                                        for index in range(len(list3)):
                                            if unit_list2[0] == list3[index][0] and unit_list2[2]>list3[index][2]:
                                                list3.remove(list3[index])
                                                list3.append(unit_list2)
                                # list1 = sorted(set(list1), key=list1.index)
                                for iii in range(a-b):
                                  list3.append([0, 0, 0])
                                list3.sort(reverse=True)
                                kk = len(NewController[self.close_area[num]][0].targetbid[index_tar]) - 1
                                k = NewController[self.close_area[num]][0].targetbid[index_tar][0]
                                self.targetbid[index_tar] = []
                                self.targetbid[index_tar].append(k)
                                for ii in range(kk):
                                    if ii < len(list3):
                                        self.targetbid[index_tar].append(list3[ii])
                                    else:
                                        self.targetbid[index_tar].append([0,0,0])
                                if not self.self_task[index_tar]:
                                    self.self_task[index_tar].append(0)
                                with open(os.path.dirname(__file__) + '/check.txt', 'a') as f:
                                    f.write(str(index_tar)+str('list3>>>>>>>>>>>>>>>>>>>>>') + '\n'+str(list3) + '\n'+
                                            str('>>>>>>>>>>>>>>>>>>>>>') + '\n')
                           else:
                               for i in range(len(NewController[self.close_area[num]][0].targetbid[index_tar])):
                                   list1.append(NewController[self.close_area[num]][0].targetbid[index_tar][i])
                               list1.extend(self.targetbid[index_tar])
                               list1.remove(NewController[self.close_area[num]][0].targetbid[index_tar][0])
                               list1.remove(self.targetbid[index_tar][0])
                               list2 = []
                               for unit_list1 in list1:
                                   if unit_list1 not in list2:
                                       list2.append(unit_list1)
                               # list1 = sorted(set(list1), key=list1.index)
                               a = len(list2)
                               while [0,0,0] in list2:
                                   list2.remove([0,0,0])
                               b = len(list2)
                               #list1 = sorted(set(list1), key=list1.index)
                               list3 = []
                               for unit_list2 in list2:
                                   if not list3:
                                       list3.append(unit_list2)
                                   else:
                                       key = 0
                                       for index_1 in range(len(list3)):
                                           if unit_list2[0] == list3[index_1][0]:
                                               key = 1
                                       if key == 0:
                                           list3.append(unit_list2)
                                       for index in range(len(list3)):
                                           if unit_list2[0] == list3[index][0] and unit_list2[2] > list3[index][2]:
                                               list3.remove(list3[index])
                                               list3.append(unit_list2)
                               for iii in range(a - b):
                                   list3.append([0,0,0])
                               list3.sort(reverse=True)
                               kk = len(self.targetbid[index_tar])-1
                               kkk = self.targetbid[index_tar][0]
                               self.targetbid[index_tar] = [kkk]
                               for ii in range(kk):
                                   if ii < len(list3):
                                       self.targetbid[index_tar].append(list3[ii])
                                   else:
                                       self.targetbid[index_tar].append([0,0,0])
                               if not self.self_task[index_tar]:
                                   self.self_task[index_tar].append(0)
                    else:
                        for i in range(len(NewController[self.close_area[num]][0].targetbid[index_tar])):
                            self.targetbid[index_tar].append(NewController[self.close_area[num]][0].targetbid[index_tar][i])
                        if not self.self_task[index_tar]:
                            self.self_task[index_tar].append(0)

        #将targetbid中的所有相同编号的变成一个
        with open(os.path.dirname(__file__) + '/check.txt', 'a') as f:
            f.write('>>>>>>>>>>>>>>>>>>' + '\n' +
                    str(self.targetbid) + '\n')
        list5 = []
        for i in range(len(self.targetbid)):
            for j in range(len(self.targetbid[i])):
                u = str([self.targetbid[i][j], i, j])
                u = u.replace('[', '')
                u = u.replace(']', '')
                a = list(eval(u))
                list5.append(a)
        list4 = []
        if self.step_now == 500:
            print("gg")
        for unit_list3 in list5:
            signal = 0
            if not list4:
                list4.append(unit_list3)
            for index2 in list4:
                if unit_list3[0] == index2[0] and unit_list3[2] != index2[2] and len(index2) == 5 and len(unit_list3) == 5:

                    if unit_list3[2]>index2[2]:
                        aa = index2[-2]
                        self.targetbid[aa].remove(index2[0:3])
                        self.targetbid[aa].append([0, 0, 0])
                        signal = 1
                    else:
                        aa = unit_list3[-2]
                        self.targetbid[aa].remove(unit_list3[0:3])
                        self.targetbid[aa].append([0, 0, 0])
                        signal =2
                    break
            list4.append(unit_list3)
            if signal == 1:
                list4.remove(index2)
            if signal == 2:
                list4.remove(unit_list3)




            # for ii in range(len(list4)):
            #     if unit_list3[0] == list4[ii][0] and unit_list3[0] != 0 and unit_list3[1] != list4[ii][1]:
            #         if unit_list3[-3] < list4[ii][-3]:
            #             aa = unit_list3[-2]
            #             self.targetbid[aa].remove(unit_list3[0:-2])
            #             self.targetbid[aa].append([0, 0, 0])
            #         elif unit_list3[-3] > list4[ii][-3]:
            #              aa = list4[ii][-2]
            #              self.targetbid[aa].remove(list4[0:-2])
            #              self.targetbid[aa].append([0, 0, 0])
        with open(os.path.dirname(__file__) + '/check.txt', 'a') as f:
            f.write('>>>>>>>>>>>>>>>>>>' + '\n' +
                    str(self.targetbid) + '\n' + str(list5) + '\n' + str(list4) + '\n')






    def bidding(self, obs, WorldTarget):

        # Pr = U - C (Pr为最终出价, U为收益, C为成本)

        # 收益U的相关参数
        x = 1         # 表示是否用于打击目标 0 or 1
        e1 = 0.5      # 我方小飞机优势系数
        e2 = 0.5      # 敌方目标战术价值系数  e1 + e2 = 1 (0 <= e1, e2 <= 1)
        pt = 0.8      # 小飞机单发杀伤概率

        #已发现目标的战术价值
        # W = []
        # for num in range(len(self.self_task)):
        #     if self.self_task[num]:
        #         W.append(WorldTarget[num][-4])
        #     else:
        #         W.append(0)

        # 已发现目标的战术价值并使用了高斯分布求均值
        # mu 为p(真实） sigma为self.proportion * mu
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
                P_one = (sigma1 * Fd + sigma2 * Fq)
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
                kkkkk[a].append([self.index, random.randint(100000000, 1000000000000), 1000000])
                self.targetbid = kkkkk.copy()
                self.over = 1

            else:
                self.step_now = step

                if step < self.Step0:
                    # print('UAV', self.index, 'searching')
                    self.close_area = self.find_mate_communication(obs_n).copy()
                    self.add_new_target(obs_n[self.index], WorldTarget, NewController)
                    self.opt_index = 0

                #elif ((step % 2) == 0 and self.mission_success == 0) or ((step % 2) == 1 and max(self.self_task) == [0] and self.mission_success == 0):
                elif step % 10 == 0:
                    if max(self.self_task) == [0]:
                        self.opt_index = 0

                    self.self_bid = self.bidding(obs_n[self.index], WorldTarget)
                else:
                    if max(self.self_task) == [0]:
                        self.opt_index = 0
                    self.close_area = self.find_mate_communication(obs_n).copy()
                    self.add_new_target(obs_n[self.index], WorldTarget, NewController)
                    with open(os.path.dirname(__file__) + '/check.txt', 'a') as f:
                        f.write(str(self.targetbid) + '\n')
                    for i in range(len(self.self_bid)):
                        # 比较self出价与竞标价格
                        if self.targetbid[self.self_bid[i][0]]:
                            kkk = self.targetbid[self.self_bid[i][0]].copy()
                            kkk_mirror = []
                            for index_mirror in range(len(kkk)):
                                kkk_mirror.append(kkk[index_mirror])
                            for index_t in range(len(kkk_mirror)-1):
                                if kkk_mirror[index_t+1][1] == 0:
                                    kkk.remove(kkk_mirror[index_t+1])
                            kkk.remove(self.targetbid[self.self_bid[i][0]][0])

                            #查看self.targetbid中的index_tar是否已经有self.index
                            signal = 666
                            # 本个体是否已经在targetbid中，若不在，则signal=666
                            for index_kkk in range(len(kkk)):
                                if kkk[index_kkk][0] == self.index:
                                    signal = index_kkk

                            if not kkk and self.self_bid[i][1]>0:
                                a = self.self_bid[i]
                                self.targetbid[self.self_bid[i][0]][1] = [self.index, a[1], self.step_now]
                                for j in range(len(self.self_task)):
                                    if self.self_task[j]:
                                        self.self_task[j][0] = 0
                                self.self_task[self.self_bid[i][0]][0] = 1
                                break

                            elif kkk[kkk.index(min(kkk, key=lambda kkk: kkk[1]))][1] < self.self_bid[i][1] and signal == 666:
                                a = self.self_bid[i]
                                self.targetbid[self.self_bid[i][0]][kkk.index(min(kkk))+1] = [self.index, a[1], self.step_now]
                                for j in range(len(self.self_task)):
                                    if self.self_task[j]:
                                        self.self_task[j][0] = 0
                                self.self_task[self.self_bid[i][0]][0] = 1
                                break
                            elif signal != 666:
                                a = self.self_bid[i]
                                self.targetbid[self.self_bid[i][0]][signal+1] = [self.index, a[1], self.step_now]
                                for j in range(len(self.self_task)):
                                    if self.self_task[j]:
                                        self.self_task[j][0] = 0
                                self.self_task[self.self_bid[i][0]][0] = 1
                                with open(os.path.dirname(__file__) + '/check.txt', 'a') as f:
                                    f.write(str(self.self_bid[i][0]) + str('更新区域') +str(signal+1) + '\n')
                                break

                            else:
                                # print(self_bid[i][0])
                                # print(self.self_task)
                                if self.self_task[self.self_bid[i][0]]:
                                    self.self_task[self.self_bid[i][0]][0] = 0
                                else:
                                    self.self_task[self.self_bid[i][0]].append(0)
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
                        f.write(str(self.step_now) + '\n' + str(self.self_bid) + '\n')
                # print(self.index)
                # print(self.self_task)
                # print(self.targetbid)
        with open(os.path.dirname(__file__) + '/check.txt', 'a') as f:
            f.write(str(step) + '\n' + str(self.index) + '\n' + str(self.self_task) + '\n'
                    + str(self.targetbid) + '\n' + str(self.close_area) + '\n')

        return [self.opt_index, [self.x, self.y, self.result, self.mission_success]]
