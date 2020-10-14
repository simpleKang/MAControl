from MAControl.Base.PolicyMaker import PolicyMaker
from MAControl.Util.PointInRec import point_in_rec
from MAControl.Util.viewfield import viewfield
from collections import Counter
import numpy as np
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

    SEEN_TARGETS = []
    RESULT = []
    Prices = []

    def __init__(self, name, env, world, agent_index, arglist):
        super(PolicyMaker_Probability, self).__init__(name, env, world, agent_index, arglist)
        self.opt_index = 0
        self.x = 0
        self.y = 0
        self.InAttacking = False
        self.result = -1
        self.seen_targets = []

        # 以下为一些阶段的初始设定步数
        # >> 未来步数点可修改，从而可以主动停留在某一阶段/步
        # >> 进入攻击阶段之后就跳出这部分逻辑而无所谓这些数值
        # >> 进入重置步的时候将所有步数点推到未来避免进入 >Step5 的无定义情况
        self.Step0 = 500
        self.Step1 = 501
        self.Step2 = 502
        self.Step3 = 503
        self.Step4 = 504
        self.Step5 = 505

        self.mission_swarm = []
        self.close_area = []
        self.price = 0
        self.rank = 0

    def find_mate(self, obs_n, r=0.5):
        selfpos = np.array([obs_n[self.index][0], obs_n[self.index][15], obs_n[self.index][16]])  # alt lat lon
        close_area = []
        for i in range(len(obs_n)):
            posi = np.array([obs_n[i][0], obs_n[i][15], obs_n[i][16]])
            Dpos = abs(posi - selfpos)
            delta = np.sqrt(Dpos[0] * Dpos[0] + Dpos[1] * Dpos[1] * 0.4 + Dpos[2] * Dpos[2] * 0.4)
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
            self.Step3 = self.Step0 + 3
            self.Step4 = self.Step0 + 4
            self.Step5 = self.Step0 + 5

        if operate_index == 2:
            #  finish searching immediately, start resorting at next step
            self.Step0 = step + 1
            self.Step1 = self.Step0 + 1
            self.Step2 = self.Step0 + 2
            self.Step3 = self.Step0 + 3
            self.Step4 = self.Step0 + 4
            self.Step5 = self.Step0 + 5

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
        delta_lla = np.array(self.seen_targets[Tndx][2:5]) - np.array([obs[16], obs[15], obs[0]])
        dis = math.sqrt(0.01*delta_lla[0]**2 + 0.01*delta_lla[1]**2 + 0.1*delta_lla[2]**2)
        v_dir = obs[3]
        t_dir = math.atan2(delta_lla[1], delta_lla[0])
        angle = abs(v_dir - t_dir)
        Fd = math.exp(1 - dis / D)
        Fq = math.exp(1 - angle/math.pi)
        P = sigma1 * Fd + sigma2 * Fq

        # 计算收益U
        U = (e1 * P + e2 * W) * pt

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

            elif step == self.Step0:
                # print('UAV', self.index, 'resort, then store for communication')
                self.seen_targets = sorted(self.seen_targets, key=lambda x: x[5], reverse=True)
                PolicyMaker_Probability.SEEN_TARGETS.append(self.seen_targets)

            elif step == self.Step1:
                # print('UAV', self.index, 'communicate locally')
                HIGHEST_TARGETS = []
                for i in self.close_area:
                    HIGHEST_TARGETS.append(PolicyMaker_Probability.SEEN_TARGETS[i][0])
                HIGHEST_TARGETS = sorted(HIGHEST_TARGETS, key=lambda x: x[5], reverse=True)
                self.result = HIGHEST_TARGETS[0]
                PolicyMaker_Probability.RESULT.append(self.result[-1])

            elif step == self.Step2:
                # print('UAV', self.index, 'choose target, then generate mission-swarm accordingly, then bid price')
                Counter_k = Counter(PolicyMaker_Probability.RESULT).most_common(1)
                target_index = Counter_k[0][0]
                for i, result in enumerate(PolicyMaker_Probability.RESULT):
                    if result == target_index:
                        self.mission_swarm.append(i)
                    else:
                        pass
                if self.index in self.mission_swarm:
                    self.price = self.bidding(obs_n[self.index], 0)
                else:
                    self.price = 0
                PolicyMaker_Probability.Prices.append(self.price)

            elif step == self.Step3:
                # print('UAV', self.index, 'sort price, then determine own rank')
                PolicyMaker_Probability.Prices = sorted(PolicyMaker_Probability.Prices, reverse=True)
                self.rank = PolicyMaker_Probability.Prices.index(self.price)

            elif step == self.Step4:
                # 下述计算会在各个UAV本地重复进行，确认自己是否具有攻击资格，部分UAV即将进入攻击阶段

                # 根据当前目标的类型估计，重新讨论目标的类型（含有随机性），进而确定需要的UAV个数
                DEMANDED_UAV_NUM = 0
                if self.result[6] == 5:
                    DEMANDED_UAV_NUM = np.random.choice([5, 1, 2], 1, p=self.arglist.q1)[0]
                elif self.result[6] == 1:
                    DEMANDED_UAV_NUM = np.random.choice([5, 1, 2], 1, p=self.arglist.q2)[0]
                elif self.result[6] == 2:
                    DEMANDED_UAV_NUM = np.random.choice([5, 1, 2], 1, p=self.arglist.q3)[0]

                if self.rank < DEMANDED_UAV_NUM:
                    # print('UAV', self.index, 'to attack')
                    self.opt_index = 10
                    self.InAttacking = True
                    self.x = self.result[0]
                    self.y = self.result[1]
                else:
                    pass
                    # print('UAV', self.index, 'not to attack')

            elif step == self.Step5:
                # print('UAV', self.index, 'recycling')
                self.operate_step(1, step)
                PolicyMaker_Probability.SEEN_TARGETS = []
                PolicyMaker_Probability.RESULT = []
                PolicyMaker_Probability.Prices = []

            else:
                raise Exception('Wrong Wrong Wrong')

        return self.opt_index, [self.result, [self.x, self.y]]
