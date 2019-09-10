from MAControl.Base.PolicyMaker import PolicyMaker
import random
import numpy as np
from MAControl.Util.PointInRec import point_in_rec


class PolicyMaker_Auciton(PolicyMaker):

    #                                       (Step2<=)&(<Step3)
    # 搜索目标[阶段] | 排序目标[步] | 选择目标[步] | 出价[阶段] | 统计价格[步] | 分道扬镳[步] | 重置[步] >>>> 搜索目标[阶段] ....
    #  <Step0         ==Step0       ==Step1         ^         ==Step3      ==Step4      ==Step5
    #                                                                        |
    #                                                                        |
    #                                                    InAttacking == True |
    #                                                                        |
    #                                                                     攻击[阶段]

    # 在搜索目标阶段(<Step0)操作，只增不减
    Found_Target_Set = []  # {target_pos, target_vel, target_value, target_defence}
    Found_Target_Info = []  # {TARGET:UAV_INDEX}

    # 在重置步(==Step5)操作，只增不减
    Attacked_Target_Index = []  # {TARGET_INDEX}

    # 在分道步(==Step4)操作，只减不增
    Remain_UAV_Set = []  # {UAV_INDEX}

    # 在重置步(==Step5)重置
    Remain_Target_Set = []  # {target_pos, target_vel, target_value, target_defence, TARGET_INDEX}
    Current_Target_Index = -1  # {TARGET_INDEX}
    Current_Price_Set = []   # {UAV X STEP}
    Current_Price_Result = []  # {UAV_INDEX,UAV_PRICE}

    def __init__(self, name, env, world, agent_index, arglist):
        super(PolicyMaker_Auciton, self).__init__(name, env, world, agent_index, arglist)
        self.opt_index = 0
        self.x = 0
        self.y = 0
        self.InAttacking = False

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
        PolicyMaker_Auciton.Remain_UAV_Set.append(self.index)

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
        selfvelrightunit = np.array([selfvelunit[1], -1 * selfvelunit[0]])
        d1 = 0
        d2 = 0.5
        d3 = 0.5
        selfview1 = selfpos + selfvelunit * (d1 + d2) - selfvelrightunit * d3 / 2
        selfview2 = selfpos + selfvelunit * (d1 + d2) + selfvelrightunit * d3 / 2
        selfview3 = selfpos + selfvelunit * d1 + selfvelrightunit * d3 / 2
        selfview4 = selfpos + selfvelunit * d1 - selfvelrightunit * d3 / 2

        # GENERATE seen_target
        seen_target = []
        for target in WorldTarget:
            targetpos = np.array(target[1:3])
            if point_in_rec(selfview1, selfview2, selfview3, selfview4, targetpos):
                seen_target.append(target)

        # READ AND WRITE TESTControl.Found_Target_Set
        if not PolicyMaker_Auciton.Found_Target_Set:
            PolicyMaker_Auciton.Found_Target_Set = seen_target
            for i in range(len(seen_target)):
                PolicyMaker_Auciton.Found_Target_Info.append(self.close_area)
        elif seen_target:
            for target1 in seen_target:
                check = False
                for target2 in PolicyMaker_Auciton.Found_Target_Set:
                    pos1 = np.array(target1[1:3])
                    pos2 = np.array(target2[1:3])
                    deltapos = np.sqrt(np.dot(pos1 - pos2, pos1 - pos2))
                    check = check | (deltapos <= ttrange)
                if not check:
                    PolicyMaker_Auciton.Found_Target_Set.append(target1)
                    PolicyMaker_Auciton.Found_Target_Info.append(self.close_area)

        # COMMUNICATE TESTControl.Found_Target_Info
        for info in PolicyMaker_Auciton.Found_Target_Info:
            check = False
            for num in self.close_area:
                check = check | num in info
            if check and (self.index not in info):
                info.append(self.index)

    def operate_step(self, operate_index, step):

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

    def make_policy(self, WorldTarget, obs_n, step):

        if self.InAttacking:
            self.opt_index = 5
            print('UAV', self.index, 'ATTACKING ATTACKING')

        else:

            if step < self.Step0:
                print('UAV', self.index, 'searching')
                self.close_area = self.find_mate(obs_n)
                self.add_new_target(obs_n[self.index], WorldTarget)

                if (step == (self.Step0 - 1)) and (not PolicyMaker_Auciton.Found_Target_Set) \
                        and (len(PolicyMaker_Auciton.Attacked_Target_Index)!=len(PolicyMaker_Auciton.Found_Target_Set)):
                    self.operate_step(0, step)

            elif step == self.Step0:
                print('UAV', self.index, 'resorting')

                if self.index == max(PolicyMaker_Auciton.Remain_UAV_Set):

                    for i in range(len(PolicyMaker_Auciton.Found_Target_Set)):
                        if i not in PolicyMaker_Auciton.Attacked_Target_Index:
                            PolicyMaker_Auciton.Remain_Target_Set.append(PolicyMaker_Auciton.Found_Target_Set[i]+[i])

                    PolicyMaker_Auciton.Remain_Target_Set = sorted(PolicyMaker_Auciton.Remain_Target_Set, key=lambda x: x[4], reverse=True)
                    print('Remain_Target_Set: ', PolicyMaker_Auciton.Remain_Target_Set)

            elif step == self.Step1:
                print('UAV', self.index, 'choosing')

                if self.index == max(PolicyMaker_Auciton.Remain_UAV_Set):
                    PolicyMaker_Auciton.Current_Target_Index = PolicyMaker_Auciton.Remain_Target_Set[0][-1]
                    print('Current_Target_Index: ', PolicyMaker_Auciton.Current_Target_Index)
                    PolicyMaker_Auciton.Current_Price_Set = [[0 for j in range(len(PolicyMaker_Auciton.Remain_UAV_Set))] for k in range(18)]

            elif self.Step2 <= step < self.Step3:
                print('UAV', self.index, 'pricing')

                if random.random() > 0.5:
                    PolicyMaker_Auciton.Current_Price_Set[step - self.Step2][self.index] = random.random()

            elif step == self.Step3:
                print('UAV', self.index, 'priced')
                self.swarm_size = len(PolicyMaker_Auciton.Remain_UAV_Set)

                if self.index == max(PolicyMaker_Auciton.Remain_UAV_Set):

                    results = sum(np.array(PolicyMaker_Auciton.Current_Price_Set))
                    for k in range(len(PolicyMaker_Auciton.Remain_UAV_Set)):
                        PolicyMaker_Auciton.Current_Price_Result.append([PolicyMaker_Auciton.Remain_UAV_Set[k], results[k]])

                    PolicyMaker_Auciton.Current_Price_Result = \
                        sorted(PolicyMaker_Auciton.Current_Price_Result, key=lambda x: x[1], reverse=True)
                    print('Current_Price_Result: ', PolicyMaker_Auciton.Current_Price_Result)

            elif step == self.Step4:
                # 下述计算会在各个UAV本地重复进行，确认自己是否具有攻击资格，部分UAV即将进入攻击阶段
                # 因为基于共享量进行计算，中间变量的计算结果将相同

                DEMANDED_UAV_NUM = PolicyMaker_Auciton.Found_Target_Set[PolicyMaker_Auciton.Current_Target_Index][5]
                if DEMANDED_UAV_NUM > self.swarm_size:
                    print('WARNING: HARD TARGET ', PolicyMaker_Auciton.Current_Target_Index)
                    CHOSEN_UAV_NUM = self.swarm_size
                else:
                    CHOSEN_UAV_NUM = DEMANDED_UAV_NUM

                CHOSEN_UAV_SET = []
                for k in range(CHOSEN_UAV_NUM):
                    CHOSEN_UAV_SET.append(PolicyMaker_Auciton.Current_Price_Result[k][0])

                if self.index in CHOSEN_UAV_SET:
                    print('UAV', self.index, 'to attack')
                    self.InAttacking = True
                    self.x = PolicyMaker_Auciton.Found_Target_Set[PolicyMaker_Auciton.Current_Target_Index][0]
                    self.y = PolicyMaker_Auciton.Found_Target_Set[PolicyMaker_Auciton.Current_Target_Index][1]
                    PolicyMaker_Auciton.Remain_UAV_Set.remove(self.index)
                else:
                    print('UAV', self.index, 'not to attack')

            elif step == self.Step5:
                print('UAV', self.index, 'recycling')
                self.operate_step(1, step)

                if self.index == max(PolicyMaker_Auciton.Remain_UAV_Set):
                    PolicyMaker_Auciton.Attacked_Target_Index.append(PolicyMaker_Auciton.Current_Target_Index)
                    PolicyMaker_Auciton.Remain_Target_Set = []
                    PolicyMaker_Auciton.Current_Target_Index = -1
                    PolicyMaker_Auciton.Current_Price_Set = []
                    PolicyMaker_Auciton.Current_Price_Result = []

            else:
                raise Exception('Wrong Wrong Wrong')

        return [self.opt_index, self.x, self.y]
