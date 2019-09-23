from MAControl.Base.PolicyMaker import PolicyMaker
import random
import numpy as np
from MAControl.Util.PointInRec import point_in_rec
from MAControl.Util.Constrain import constrain
import math


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
        selfpos = np.array(obs_n[self.index][2:4])
        close_area = []
        for i in range(len(obs_n)):
            posi = obs_n[i][2:4]
            deltapos = np.sqrt(np.dot(selfpos - posi, selfpos - posi))
            if deltapos < r:
                close_area.append(i)
        return close_area

    def add_new_target(self, obs, WorldTarget, ttrange=0.05):

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
                    gtype = np.random.choice([1, 2, 3], 1, p=[0.2, 0.4, 0.4])
                    if gtype == 2:
                        seen_target[-1][-4:-1] = [10, 1, 2]
                    elif gtype == 3:
                        seen_target[-1][-4:-1] = [5, 2, 3]
                elif truetype == 2:
                    gtype = np.random.choice([1, 2, 3], 1, p=[0.4, 0.2, 0.4])
                    if gtype == 3:
                        seen_target[-1][-4:-1] = [5, 2, 3]
                    elif gtype == 1:
                        seen_target[-1][-4:-1] = [2, 5, 1]
                elif truetype == 3:
                    gtype = np.random.choice([1, 2, 3], 1, p=[0.4, 0.4, 0.2])
                    if gtype == 1:
                        seen_target[-1][-4:-1] = [2, 5, 1]
                    elif gtype == 2:
                        seen_target[-1][-4:-1] = [10, 1, 2]
                # 在seen_target中，真序号是准确的（唯一标识），类型可能有误（相应的价值和防御能力都有误）

        # READ AND WRITE TESTControl.Found_Target_Set
        if not PolicyMaker_Auction.Found_Target_Set:
            PolicyMaker_Auction.Found_Target_Set = seen_target
            for i in range(len(seen_target)):
                PolicyMaker_Auction.Found_Target_Info.append(self.close_area)
        elif seen_target:
            for target1 in seen_target:
                check = False
                for target2 in PolicyMaker_Auction.Found_Target_Set:
                    pos1 = np.array(target1[0:2])
                    pos2 = np.array(target2[0:2])
                    deltapos = np.sqrt(np.dot(pos1 - pos2, pos1 - pos2))
                    check = check | (deltapos <= ttrange)
                if not check:
                    PolicyMaker_Auction.Found_Target_Set.append(target1)
                    PolicyMaker_Auction.Found_Target_Info.append(self.close_area)

        # COMMUNICATE TESTControl.Found_Target_Info
        for info in PolicyMaker_Auction.Found_Target_Info:
            check = False
            for num in self.close_area:
                check = check | num in info
            if check and (self.index not in info):
                info.append(self.index)

    def searching_is_good_enough(self, step):
        check1 = ((PolicyMaker_Auction.Found_Target_Set) != [])
        check2 = (len(PolicyMaker_Auction.Attacked_Target_Index) != len(PolicyMaker_Auction.Found_Target_Set))

        # 实际情况
        check3a = 0 if len(PolicyMaker_Auction.Found_Target_Set) == 0 \
            else (np.sum(PolicyMaker_Auction.Found_Target_Set, axis=0))[4]/len(PolicyMaker_Auction.Remain_UAV_Set)
        # 阈值随时间减少
        check3b = 1000/self.env.n*20/(step+1)
        check3 = (check3a > check3b)

        if check1 and check2 and check3:
            output = True
        else:
            output = False
        return output

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

    def bidding(self, obs):

        # Pr = U - C (Pr为最终出价, U为收益, C为成本)

        # 收益U的相关参数
        x = 1         # 表示是否用于打击目标 0 or 1
        e1 = 0.5      # 我方小飞机优势系数
        e2 = 0.5      # 敌方目标战术价值系数  e1 + e2 = 1 (0 <= e1, e2 <= 1)
        pt = 0.8      # 小飞机单发杀伤概率
        W = PolicyMaker_Auction.Found_Target_Set[PolicyMaker_Auction.Current_Target_Index][4]        # 目标的战术价值
        sigma1 = 0.5  # 距离优势系数
        sigma2 = 0.5  # 角度优势系数
        D = 0.6       # 小飞机能够攻击目标的最大距离

        # 成本C的相关参数
        s = 0.5       # 小飞机的代价系数
        T = 0.5       # 敌方目标对小飞机的威胁程度
        pt_ = 0.6     # 目标的单发杀伤概率

        # 计算中间变量
        dis = math.sqrt((PolicyMaker_Auction.Found_Target_Set[PolicyMaker_Auction.Current_Target_Index][0] - obs[2])**2+
                        (PolicyMaker_Auction.Found_Target_Set[PolicyMaker_Auction.Current_Target_Index][1] - obs[3])**2)
        v_unit = np.array([obs[0], obs[1]])/math.sqrt(obs[0]**2+obs[1]**2)
        t_unit = np.array([PolicyMaker_Auction.Found_Target_Set[PolicyMaker_Auction.Current_Target_Index][0] - obs[2],
                           PolicyMaker_Auction.Found_Target_Set[PolicyMaker_Auction.Current_Target_Index][1] - obs[3]])/dis
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

                if self.index == max(PolicyMaker_Auction.Remain_UAV_Set):

                    for i in range(len(PolicyMaker_Auction.Found_Target_Set)):
                        if i not in PolicyMaker_Auction.Attacked_Target_Index:
                            PolicyMaker_Auction.Remain_Target_Set.append(PolicyMaker_Auction.Found_Target_Set[i]+[i])

                    PolicyMaker_Auction.Remain_Target_Set = sorted(PolicyMaker_Auction.Remain_Target_Set, key=lambda x: x[4], reverse=True)
                    # print('Found_Target_Set: ', PolicyMaker_Auction.Found_Target_Set)
                    # print('Remain_Target_Set: ', PolicyMaker_Auction.Remain_Target_Set)

            elif step == self.Step1:
                # print('UAV', self.index, 'choosing')

                if self.index == max(PolicyMaker_Auction.Remain_UAV_Set):
                    PolicyMaker_Auction.Current_Target_Index = PolicyMaker_Auction.Remain_Target_Set[0][-1]
                    # print('Current_Target_Index: ', PolicyMaker_Auction.Current_Target_Index)
                    PolicyMaker_Auction.Current_Price_Set = [[0 for j in range(len(PolicyMaker_Auction.Remain_UAV_Set))] for k in range(18)]

            elif self.Step2 <= step < self.Step3:
                # print('UAV', self.index, 'pricing')

                # TODO 是否出价如何判断
                if random.random() > 0.5:
                    k = PolicyMaker_Auction.Remain_UAV_Set.index(self.index)
                    PolicyMaker_Auction.Current_Price_Set[step - self.Step2][k] = self.bidding(obs_n[self.index])
                    # Current_Price_Set 是根据 Remain_UAV_Set 生成的，从后者选取编号

            elif step == self.Step3:
                # print('UAV', self.index, 'priced')
                self.swarm_size = len(PolicyMaker_Auction.Remain_UAV_Set)

                if self.index == max(PolicyMaker_Auction.Remain_UAV_Set):

                    results = sum(np.array(PolicyMaker_Auction.Current_Price_Set))
                    for k in range(len(PolicyMaker_Auction.Remain_UAV_Set)):
                        PolicyMaker_Auction.Current_Price_Result.append([PolicyMaker_Auction.Remain_UAV_Set[k], results[k]])

                    PolicyMaker_Auction.Current_Price_Result = \
                        sorted(PolicyMaker_Auction.Current_Price_Result, key=lambda x: x[1], reverse=True)
                    # print('Current_Price_Result: ', PolicyMaker_Auction.Current_Price_Result)

            elif step == self.Step4:
                # 下述计算会在各个UAV本地重复进行，确认自己是否具有攻击资格，部分UAV即将进入攻击阶段

                # 根据当前目标的类型估计，重新讨论目标的类型（含有随机性），进而确定需要的UAV个数
                DEMANDED_UAV_NUM = 0
                if PolicyMaker_Auction.Found_Target_Set[PolicyMaker_Auction.Current_Target_Index][5] == 5:
                    DEMANDED_UAV_NUM = np.random.choice([5, 1, 2], 1, p=[0.0896, 0.0467, 0.8637])[0]
                elif PolicyMaker_Auction.Found_Target_Set[PolicyMaker_Auction.Current_Target_Index][5] == 1:
                    DEMANDED_UAV_NUM = np.random.choice([5, 1, 2], 1, p=[0.1313, 0.3319, 0.5368])[0]
                elif PolicyMaker_Auction.Found_Target_Set[PolicyMaker_Auction.Current_Target_Index][5] == 2:
                    DEMANDED_UAV_NUM = np.random.choice([5, 1, 2], 1, p=[0.0384, 0.3925, 0.5691])[0]

                if DEMANDED_UAV_NUM > self.swarm_size:
                    # print('WARNING: HARD TARGET ', PolicyMaker_Auction.Current_Target_Index)
                    CHOSEN_UAV_NUM = self.swarm_size
                else:
                    CHOSEN_UAV_NUM = DEMANDED_UAV_NUM

                CHOSEN_UAV_SET = []
                for k in range(CHOSEN_UAV_NUM):
                    CHOSEN_UAV_SET.append(PolicyMaker_Auction.Current_Price_Result[k][0])

                if self.index in CHOSEN_UAV_SET:
                    # print('UAV', self.index, 'to attack')
                    self.InAttacking = True
                    self.x = PolicyMaker_Auction.Found_Target_Set[PolicyMaker_Auction.Current_Target_Index][0]
                    self.y = PolicyMaker_Auction.Found_Target_Set[PolicyMaker_Auction.Current_Target_Index][1]
                    self.result = PolicyMaker_Auction.Found_Target_Set[PolicyMaker_Auction.Current_Target_Index][7]
                    PolicyMaker_Auction.Remain_UAV_Set.remove(self.index)
                else:
                    pass
                    # print('UAV', self.index, 'not to attack')

            elif step == self.Step5:
                # print('UAV', self.index, 'recycling')
                self.operate_step(1, step)

                if len(PolicyMaker_Auction.Remain_Target_Set) == 1:
                    self.opt_index = 5
                    self.x = len(PolicyMaker_Auction.Remain_UAV_Set)
                    self.y = PolicyMaker_Auction.Remain_UAV_Set.index(self.index)

                if self.index == max(PolicyMaker_Auction.Remain_UAV_Set):
                    PolicyMaker_Auction.Attacked_Target_Index.append(PolicyMaker_Auction.Current_Target_Index)
                    PolicyMaker_Auction.Remain_Target_Set = []
                    PolicyMaker_Auction.Current_Target_Index = -1
                    PolicyMaker_Auction.Current_Price_Set = []
                    PolicyMaker_Auction.Current_Price_Result = []

            else:
                raise Exception('Wrong Wrong Wrong')

        return [self.opt_index, [self.x, self.y, self.result]]
