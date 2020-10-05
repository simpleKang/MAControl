from MAControl.Base.PolicyMaker import PolicyMaker
from MAControl.Util.PointInRec import point_in_rec
from MAControl.Util.viewfield import viewfield
import random
import numpy as np
from MAControl.Util.Constrain import constrain
import math


class PolicyMaker_Probability(PolicyMaker):

    #                                       (Step2<=)&(<Step3)
    # 搜索目标[阶段] | 排序目标[步] | 选择目标[步] | 出价[阶段] | 统计价格[步] | 分道扬镳[步] | 重置[步] >>>> 搜索目标[阶段] ....
    #  <Step0         ==Step0       ==Step1         ^         ==Step3      ==Step4      ==Step5
    #                                                                        |
    #                                                                        |
    #                                                    InAttacking == True |
    #                                                                        |
    #                                                                     攻击[阶段]

    def __init__(self, name, env, world, agent_index, arglist):
        super(PolicyMaker_Probability, self).__init__(name, env, world, agent_index, arglist)
        self.opt_index = 0
        self.x = 0
        self.y = 0
        self.InAttacking = False
        self.result = -1
        self.seen_targets = []
        self.attacked_targets = []

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

    def find_mate(self, obs_n, r=0.5):
        selfpos = np.array(obs_n[self.index][0], obs_n[self.index][15], obs_n[self.index][16])  # alt lat lon
        close_area = []
        for i in range(len(obs_n)):
            posi = np.array(obs_n[i][0], obs_n[i][15], obs_n[i][16])
            deltapos = abs(posi - selfpos)
            delta = np.sqrt(deltapos[0] * deltapos[0] + deltapos[1] * deltapos[1] * 0.4
                            + deltapos[2] * deltapos[2] * 0.4)
            if delta < r:
                close_area.append(i)
        return close_area

    def add_new_target(self, obs, WorldTarget, ttrange=0.05):

        # COMPUTE selfview
        selfproj = viewfield(obs[17], obs[18], obs[0] - 58.809239, obs[1], obs[2], obs[3], 0.1, 0.1)

        # GENERATE _seen_targets
        _seen_targets = []
        for target in WorldTarget:
            if point_in_rec(selfproj[0], selfproj[1], selfproj[2], selfproj[3], np.array(target[0:2])):
                _seen_targets.append(target)
                truetype = target[-2]
                if truetype == 1:
                    gtype = np.random.choice([1, 2, 3], 1, p=self.arglist.p1)
                    if gtype == 2:
                        _seen_targets[-1][-4:-1] = [10, 1, 2]
                    elif gtype == 3:
                        _seen_targets[-1][-4:-1] = [5, 2, 3]
                elif truetype == 2:
                    gtype = np.random.choice([1, 2, 3], 1, p=self.arglist.p2)
                    if gtype == 3:
                        _seen_targets[-1][-4:-1] = [5, 2, 3]
                    elif gtype == 1:
                        _seen_targets[-1][-4:-1] = [2, 5, 1]
                elif truetype == 3:
                    gtype = np.random.choice([1, 2, 3], 1, p=self.arglist.p3)
                    if gtype == 1:
                        _seen_targets[-1][-4:-1] = [2, 5, 1]
                    elif gtype == 2:
                        _seen_targets[-1][-4:-1] = [10, 1, 2]
                # 在 _seen_targets 中，真序号是准确的（唯一标识），类型可能有误（相应的价值和防御能力都有误）

        # UPDATE self.seen_targets
        if not self.seen_targets:
            self.seen_targets = _seen_targets
        elif _seen_targets:
            for target1 in _seen_targets:
                check = False
                for target2 in self.seen_targets:
                    target_link = np.array(target1[0:2]) - np.array(target2[0:2])
                    deltapos = np.sqrt(np.dot(target_link, target_link))
                    check = check | (deltapos <= ttrange)
                if not check:
                    self.seen_targets.append(target1)

    def operate_step(self, operate_index, step, waitstep=5):

        if operate_index == 0:
            # wait one more step
            self.Step0 += 1
            self.Step1 += 1
            self.Step2 += 1
            self.Step3 += 1
            self.Step4 += 1
            self.Step5 += 1

        if operate_index == 1:
            # set next cycle
            self.Step0 = step + 10
            self.Step1 = self.Step0 + 1
            self.Step2 = self.Step0 + 2
            self.Step3 = self.Step0 + 20
            self.Step4 = self.Step0 + 21
            self.Step5 = self.Step0 + 22

        if operate_index == 2:
            #  finish searching immediately, start resorting at next step
            self.Step0 = step + 1
            self.Step1 = self.Step0 + 1
            self.Step2 = self.Step0 + 2
            self.Step3 = self.Step0 + 20
            self.Step4 = self.Step0 + 21
            self.Step5 = self.Step0 + 22

        if operate_index == 3:
            # wait [waitstep] more steps
            self.Step0 += waitstep
            self.Step1 += waitstep
            self.Step2 += waitstep
            self.Step3 += waitstep
            self.Step4 += waitstep
            self.Step5 += waitstep

    def bidding(self, obs, Tndx):

        # Pr = U - C (Pr为最终出价, U为收益, C为成本)

        # 收益U的相关参数
        x = 1         # 表示是否用于打击目标 0 or 1
        e1 = 0.5      # 我方小飞机优势系数
        e2 = 0.5      # 敌方目标战术价值系数  e1 + e2 = 1 (0 <= e1, e2 <= 1)
        pt = 0.8      # 小飞机单发杀伤概率
        W = self.seen_targets[Tndx][5]    # 目标的战术价值
        sigma1 = 0.5  # 距离优势系数
        sigma2 = 0.5  # 角度优势系数
        D = 0.6       # 小飞机能够攻击目标的最大距离

        # 成本C的相关参数
        s = 0.5       # 小飞机的代价系数
        T = 0.5       # 敌方目标对小飞机的威胁程度
        pt_ = 0.6     # 目标的单发杀伤概率

        # 计算中间变量
        dis = math.sqrt((self.seen_targets[Tndx][0] - obs[2])**2 +
                        (self.seen_targets[Tndx][1] - obs[3])**2)
        v_unit = np.array([obs[0], obs[1]])/math.sqrt(obs[0]**2+obs[1]**2)
        t_unit = np.array([self.seen_targets[Tndx][0] - obs[2],
                           self.seen_targets[Tndx][1] - obs[3]])/dis
        angle = math.acos(constrain(np.dot(v_unit, t_unit), -1, 1))
        Fd = math.exp(1 - dis / D)
        Fq = math.exp(1 - angle/math.pi)
        P = sigma1 * Fd + sigma2 * Fq

        # 计算收益U
        U = (e1 * P + e2 * W) * (1 - (1 - pt)**x)

        # 计算成本C
        C = s * T * pt_

        # 最终出价P
        Pr = U - C

        return Pr

    def make_policy(self, WorldTarget, obs_n, step):

        if self.InAttacking:
            self.opt_index = 10
            # print('UAV', self.index, 'ATTACKING ATTACKING')

        else:

            if step < self.Step0:
                # print('UAV', self.index, 'searching')
                self.close_area = self.find_mate(obs_n)
                self.add_new_target(obs_n[self.index], WorldTarget)
                self.opt_index = 0

                if self.searching_is_good_enough(step):
                    self.operate_step(2, step)
                elif step == (self.Step0 - 1):
                    self.operate_step(3, step, waitstep=10)

            elif step == self.Step0:
                # print('UAV', self.index, 'resorting')

                if self.index == max(PolicyMaker_Probability.Remain_UAV_Set):

                    for i in range(len(PolicyMaker_Probability.Found_Target_Set)):
                        if i not in PolicyMaker_Probability.Attacked_Target_Index:
                            PolicyMaker_Probability.Remain_Target_Set.append(PolicyMaker_Probability.Found_Target_Set[i]+[i])

                    PolicyMaker_Probability.Remain_Target_Set = sorted(PolicyMaker_Probability.Remain_Target_Set, key=lambda x: x[5], reverse=True)
                    # print('Found_Target_Set: ', PolicyMaker_Probability.Found_Target_Set)
                    # print('Remain_Target_Set: ', PolicyMaker_Probability.Remain_Target_Set)

            elif step == self.Step1:
                # print('UAV', self.index, 'choosing')

                if self.index == max(PolicyMaker_Probability.Remain_UAV_Set):
                    PolicyMaker_Probability.Current_Target_Index = PolicyMaker_Probability.Remain_Target_Set[0][-1]
                    # print('Current_Target_Index: ', PolicyMaker_Probability.Current_Target_Index)
                    PolicyMaker_Probability.Current_Price_Set = [[0 for j in range(len(PolicyMaker_Probability.Remain_UAV_Set))] for k in range(18)]

            elif self.Step2 <= step < self.Step3:
                # print('UAV', self.index, 'pricing')

                # TODO 是否出价如何判断
                if random.random() > 0.5:
                    k = PolicyMaker_Probability.Remain_UAV_Set.index(self.index)
                    PolicyMaker_Probability.Current_Price_Set[step - self.Step2][k] = self.bidding(obs_n[self.index])
                    # Current_Price_Set 是根据 Remain_UAV_Set 生成的，从后者选取编号

            elif step == self.Step3:
                # print('UAV', self.index, 'priced')
                self.swarm_size = len(PolicyMaker_Probability.Remain_UAV_Set)

                if self.index == max(PolicyMaker_Probability.Remain_UAV_Set):

                    results = sum(np.array(PolicyMaker_Probability.Current_Price_Set))
                    for k in range(len(PolicyMaker_Probability.Remain_UAV_Set)):
                        PolicyMaker_Probability.Current_Price_Result.append([PolicyMaker_Probability.Remain_UAV_Set[k], results[k]])

                    PolicyMaker_Probability.Current_Price_Result = \
                        sorted(PolicyMaker_Probability.Current_Price_Result, key=lambda x: x[1], reverse=True)
                    # print('Current_Price_Result: ', PolicyMaker_Probability.Current_Price_Result)

            elif step == self.Step4:
                # 下述计算会在各个UAV本地重复进行，确认自己是否具有攻击资格，部分UAV即将进入攻击阶段

                # 根据当前目标的类型估计，重新讨论目标的类型（含有随机性），进而确定需要的UAV个数
                DEMANDED_UAV_NUM = 0
                if PolicyMaker_Probability.Found_Target_Set[PolicyMaker_Probability.Current_Target_Index][6] == 5:
                    DEMANDED_UAV_NUM = np.random.choice([5, 1, 2], 1, p=self.arglist.q1)[0]
                elif PolicyMaker_Probability.Found_Target_Set[PolicyMaker_Probability.Current_Target_Index][6] == 1:
                    DEMANDED_UAV_NUM = np.random.choice([5, 1, 2], 1, p=self.arglist.q2)[0]
                elif PolicyMaker_Probability.Found_Target_Set[PolicyMaker_Probability.Current_Target_Index][6] == 2:
                    DEMANDED_UAV_NUM = np.random.choice([5, 1, 2], 1, p=self.arglist.q3)[0]

                if DEMANDED_UAV_NUM > self.swarm_size:
                    # print('WARNING: HARD TARGET ', PolicyMaker_Probability.Current_Target_Index)
                    CHOSEN_UAV_NUM = self.swarm_size
                else:
                    CHOSEN_UAV_NUM = DEMANDED_UAV_NUM

                CHOSEN_UAV_SET = []
                for k in range(CHOSEN_UAV_NUM):
                    CHOSEN_UAV_SET.append(PolicyMaker_Probability.Current_Price_Result[k][0])

                if self.index in CHOSEN_UAV_SET:
                    # print('UAV', self.index, 'to attack')
                    self.InAttacking = True
                    self.x = PolicyMaker_Probability.Found_Target_Set[PolicyMaker_Probability.Current_Target_Index][0]
                    self.y = PolicyMaker_Probability.Found_Target_Set[PolicyMaker_Probability.Current_Target_Index][1]
                    self.result = PolicyMaker_Probability.Found_Target_Set[PolicyMaker_Probability.Current_Target_Index][8]
                    PolicyMaker_Probability.Remain_UAV_Set.remove(self.index)
                else:
                    pass
                    # print('UAV', self.index, 'not to attack')

            elif step == self.Step5:
                # print('UAV', self.index, 'recycling')
                self.operate_step(1, step)

                if len(PolicyMaker_Probability.Remain_Target_Set) == 1:
                    self.opt_index = 5
                    self.x = len(PolicyMaker_Probability.Remain_UAV_Set)
                    self.y = PolicyMaker_Probability.Remain_UAV_Set.index(self.index)

                if self.index == max(PolicyMaker_Probability.Remain_UAV_Set):
                    PolicyMaker_Probability.Attacked_Target_Index.append(PolicyMaker_Probability.Current_Target_Index)
                    PolicyMaker_Probability.Remain_Target_Set = []
                    PolicyMaker_Probability.Current_Target_Index = -1
                    PolicyMaker_Probability.Current_Price_Set = []
                    PolicyMaker_Probability.Current_Price_Result = []

            else:
                raise Exception('Wrong Wrong Wrong')

        return [self.opt_index, [self.x, self.y, self.result]]
