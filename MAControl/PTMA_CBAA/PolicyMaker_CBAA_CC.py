
# 在cbaa的基础上，优化了共识过程中的通信

from MAControl.Base.PolicyMaker import PolicyMaker
import numpy as np
from MAControl.Util.PointInRec import point_in_rec
import math
import random


class PolicyMaker_Probability(PolicyMaker):

    def __init__(self, name, env, world, agent_index, arglist):
        super(PolicyMaker_Probability, self).__init__(name, env, world, agent_index, arglist)
        self.opt_index = 0
        self.x = 0
        self.y = 0
        self.InAttacking = False
        self.result = []  # 缺省 or 一整条目标信息
        self.attack_type = '0'
        self.attack_time = 0
        self.close_area = []
        self.assigned = 0  # 是/否决定了去向且已抵达
        self.print = False

        self.step_now = 0
        self.Step0 = 500  # 决策起始点
        self.over = 0

        self.self_target = [[] for _ in range(len(world.targets))]
        self.targetbid = [[] for _ in range(len(world.targets))]
        self.self_bid = []

        # 高斯用在CBAA上，记忆的友方ID不会导致本个体让出当前目标
        self.memory_friendID = [[] for _ in range(len(world.targets))]
        self.gaussian = [0.3015, 0.3355, 0.3565, 0.3711, 0.3820, 0.3905, 0.3974, 0.4031]
        self.targetbid_old = []

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
            si = seen_target[i][-1]
            if not self.self_target[si]:
                # noinspection PyTypeChecker
                self.self_target[si].append(0)  # 在空的情况下，首先填个0
                # noinspection PyTypeChecker
                self.targetbid[si].append(self.step_now)
                for nn in range(int(seen_target[i][3])+int(seen_target[i][4])):
                    self.targetbid[si].append([0, 0, 0])  # 在空的情况下，填进 step_now 然后 [0, 0, 0]

        for num in range(len(self.close_area)):
            for index_tar in range(len(WorldTarget)):  # 双循环 for mate & for tar
                ggg = NewController[self.close_area[num]][0].targetbid[index_tar]
                sgg = self.targetbid[index_tar]
                # 如果友方个体目标参数处的出价不为空
                if ggg:
                    if sgg:
                        if ggg[0]:
                            list1 = [ggg[i] for i in range(len(ggg))]
                            list1.extend(sgg)
                            list1.remove(ggg[0])  # remove time from mate
                            list1.remove(sgg[0])  # remove time from self
                            list1.sort(reverse=True, key=lambda x: x[1])
                            if ggg[0] <= sgg[0]:  # 时间 (self 不早于 mate)
                                a = len(list1)  # 除零前
                                while [0, 0, 0] in list1:
                                    list1.remove([0, 0, 0])
                                b = len(list1)  # 除零后
                                list2 = []
                                for unit_list1 in list1:
                                    if unit_list1 not in list2:
                                        list2.append(unit_list1)  # 不重复
                            else:  # 时间 (self 早于 mate)
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
                            list3.sort(reverse=True, key=lambda x: x[1])
                            kk = len(ggg) - 1 if ggg[0] <= sgg[0] else len(sgg) - 1  # possibly da+db
                            time_k = ggg[0] if ggg[0] <= sgg[0] else sgg[0]
                            self.targetbid[index_tar] = [time_k]  # reset & add time
                            for ii in range(kk):
                                if ii < len(list3):
                                    self.targetbid[index_tar].append(list3[ii])
                                else:
                                    self.targetbid[index_tar].append([0, 0, 0])
                            if not self.self_target[index_tar]:
                                self.self_target[index_tar].append(0)
                    else:
                        self.targetbid[index_tar] = [ggg[i] for i in range(len(ggg))]
                        if not self.self_target[index_tar]:  # 0、[] >>> false # [0] >>> true
                            self.self_target[index_tar].append(0)
                else:
                    pass

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
        for unit5 in list5:
            list4.append(unit5)
            for unit4 in list4:  # [0]=bid_i_j, [1]=i, [2]=[j]
                if unit5[0] == unit4[0] and unit5[2] != unit4[2] and len(unit4) == len(unit5) == 5:
                    if unit5[2] > unit4[2]:
                        aa = unit4[-2]  # i
                        self.targetbid[aa].remove(unit4[0:3])
                        self.targetbid[aa].append([0, 0, 0])
                        list4.remove(unit4)
                    else:
                        aa = unit5[-2]
                        self.targetbid[aa].remove(unit5[0:3])
                        self.targetbid[aa].append([0, 0, 0])
                        list4.remove(unit5)
                    break  # break from for-loop

    def bidding(self, obs, WorldTarget):

        # Pr = U - C (Pr为最终出价, U为收益, C为成本)

        # 收益U的相关参数

        e1 = 0.5      # 我方小飞机优势系数
        e2 = 0.5      # 敌方目标战术价值系数  e1 + e2 = 1 (0 <= e1, e2 <= 1)
        pt = 0.8      # 小飞机单发杀伤概率

        W = []
        for num in range(len(self.self_target)):  # 这肯定就是len(WorldTarget) 即使有的项是[]
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

        self.targetbid_old = self.targetbid.copy()

        if self.over == 0:

            delta_pos = WorldTarget[self.self_target.index(max(self.self_target))][0:2] - obs_n[self.index][2:4]
            dis = math.sqrt(delta_pos[0] ** 2 + delta_pos[1] ** 2)

            self.assigned = 1 if dis <= 0.1 and max(self.self_target) == [1] else self.assigned
            # 双条件 ·离近咯 ·[1] in self.self_target

            if self.assigned and self.opt_index == 10:  # 上面的双条件，还有个10型opt
                kkkkk = [[] for _ in range(len(WorldTarget))]
                aa = self.self_target.index([1])
                kkkkk[aa].append(10000000)
                kkkkk[aa].append([self.index, random.randint(100000000, 1000000000000), 1000000])
                self.targetbid = kkkkk.copy()  # 只有·aa这里是上述取值 ·其他地方是[]
                self.over = 1
                if not self.print:
                    print('Step ', self.attack_time, 'UAV', self.index, 'to attack',
                          'target', self.result[-1], self.attack_type, '-type')
                    self.print = True

            else:
                self.step_now = step

                if step < self.Step0:
                    self.close_area = self.find_mate(obs_n).copy()
                    self.add_new_target(obs_n[self.index], WorldTarget, NewController)
                    self.opt_index = 0

                elif step % 10 == 0:
                    self.opt_index = 0 if max(self.self_target) == [0] else self.opt_index
                    self.self_bid = self.bidding(obs_n[self.index], WorldTarget)  # 在此赋值

                else:  # step>=Step0 且不是10的倍数
                    self.opt_index = 0 if max(self.self_target) == [0] else self.opt_index
                    self.close_area = self.find_mate(obs_n).copy()
                    self.add_new_target(obs_n[self.index], WorldTarget, NewController)

                    for i in range(len(self.self_bid)):  # 长度是len(WorldTarget)
                        a = self.self_bid[i]   # 是根据·bid降序排列的·index
                        # 比较self出价与竞标价格
                        if self.targetbid[a[0]]:  # 按照顺序一个一个看下来(如果非空)
                            kkk = self.targetbid[a[0]].copy()
                            kkk_mirror = [kkk[unit] for unit in range(len(kkk))]
                            for vnit in range(len(kkk_mirror)-1):
                                if kkk_mirror[vnit+1][1] == 0:  # 在那个三元组里的中间如果是0，就整个扔掉
                                    kkk.remove(kkk_mirror[vnit+1])
                            kkk.remove(self.targetbid[a[0]][0])  # remove time

                            # 查看self.targetbid中的index_tar是否已经有self.index
                            signal = 666
                            # 本个体是否已经在targetbid中，若不在，则signal=666
                            for unit in range(len(kkk)):
                                if kkk[unit][0] == self.index:
                                    signal = unit

                            if not kkk and a[1] > 0:  # 双条件，kkk==[] 且 bid值是有的
                                self.targetbid[a[0]][1] = [self.index, a[1], self.step_now]  # # 把a[1]给[a[0]][1]
                                for j in range(len(self.self_target)):
                                    if self.self_target[j]:
                                        self.self_target[j][0] = 0
                                # noinspection PyTypeChecker
                                self.self_target[a[0]][0] = 1
                                self.result = WorldTarget[self.self_target.index([1])]
                                self.attack_type = 'A' if 1 < self.result[3] else 'B'
                                break

                            elif kkk and self.self_target[i]:  # 双条件，kkk!==[] 且 target值是有的
                                min1_kkk = min(kkk, key=lambda x: x[1])
                                # 内圈：双条件，self里边的大，且，signal是666
                                if min1_kkk[1] < a[1] and signal == 666:  # # 把a[1]给[a[0]][kkk.index(min1_kkk)+1]
                                    self.targetbid[a[0]][kkk.index(min1_kkk)+1] = [self.index, a[1], self.step_now]
                                    for j in range(len(self.self_target)):
                                        if self.self_target[j]:
                                            self.self_target[j][0] = 0
                                    # noinspection PyTypeChecker
                                    self.self_target[a[0]][0] = 1
                                    self.result = WorldTarget[self.self_target.index([1])]
                                    self.attack_type = 'A' if kkk.index(min1_kkk)+1 < self.result[3] else 'B'
                                    break

                                # 内圈：双条件，[1] in self.self_target，且，signal是666
                                elif self.self_target[a[0]][0] == 1 and signal == 666:
                                    list0 = []
                                    beckon = 0
                                    stbid_a0_copy = self.targetbid[a[0]].copy()
                                    stbid_a0_copy.remove(stbid_a0_copy[0])  # remove time

                                    # 将上一步中的目标报价与这步中的目标报价进行比较，获得其中新添加的个体ID
                                    for bid_one in stbid_a0_copy:
                                        if bid_one[1] != 0 and bid_one not in self.targetbid_old[a[0]]:
                                            list0.append(bid_one[0])
                                    # 当前目标报价的个体ID是否在记忆中覆盖的友方ID中
                                    for bid_two in stbid_a0_copy:
                                        if bid_two[0] in self.memory_friendID[a[0]]:
                                            beckon = 1

                                    if beckon == 0:
                                        for u0 in list0:
                                            if random.random() < self.gaussian[(len(obs_n)*2-20) % 10]:
                                                for j in range(len(self.self_target)):
                                                    if self.self_target[j]:
                                                        self.self_target[j][0] = 0
                                                # noinspection PyTypeChecker
                                                self.self_target[a[0]][0] = 1
                                                self.result = WorldTarget[self.self_target.index([1])]
                                                self.attack_type = 'A' if random.random() < 0.5 else 'B'
                                                self.memory_friendID[a[0]].append(u0)
                                                break
                                            # noinspection PyTypeChecker
                                            self.self_target[a[0]][0] = 0
                                    else:
                                        for j in range(len(self.self_target)):
                                            if self.self_target[j]:
                                                self.self_target[j][0] = 0
                                        # noinspection PyTypeChecker
                                        self.self_target[a[0]][0] = 1
                                        self.result = WorldTarget[self.self_target.index([1])]
                                        self.attack_type = 'A' if random.random() < 0.5 else 'B'

                                    break

                                elif signal != 666:  # 内圈：右条件破坏了 # # 把a[1]给[a[0]][signal+1]
                                    self.targetbid[a[0]][signal+1] = [self.index, a[1], self.step_now]
                                    for j in range(len(self.self_target)):
                                        if self.self_target[j]:
                                            self.self_target[j][0] = 0
                                    # noinspection PyTypeChecker
                                    self.self_target[a[0]][0] = 1
                                    self.result = WorldTarget[self.self_target.index([1])]
                                    self.attack_type = 'A' if signal+1 < self.result[3] else 'B'
                                    break

                                else:  # 内圈：signal是666，但任意左条件都不满足
                                    if self.self_target[a[0]]:
                                        # noinspection PyTypeChecker
                                        self.self_target[a[0]][0] = 0
                                    else:
                                        # noinspection PyTypeChecker
                                        self.self_target[a[0]].append(0)

                            else:  # 余下所有情形
                                if self.self_target[a[0]]:
                                    # noinspection PyTypeChecker
                                    self.self_target[a[0]][0] = 0
                                else:
                                    # noinspection PyTypeChecker
                                    self.self_target[a[0]].append(0)

                        else:
                            pass

                    if max(self.self_target) == [1]:
                        self.x = self.result[0]
                        self.y = self.result[1]
                        self.opt_index = 10
                        self.attack_time = step
                        self.InAttacking = True
                    else:
                        self.InAttacking = False
                        self.result = []
                        self.attack_type = '0'
                        self.attack_time = 0

                    self.opt_index = 1 if self.opt_index == 10 and max(self.self_target) == [0] else self.opt_index

        else:
            pass

        fate = self.result[-1] if self.result else self.result  # self.result指目标的整条属性 or 缺省
        return self.opt_index, [self.x, self.y, fate, self.assigned, self.attack_type, self.attack_time]
