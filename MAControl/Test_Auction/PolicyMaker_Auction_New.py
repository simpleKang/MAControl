from MAControl.Base.PolicyMaker import PolicyMaker
import math
import random
import numpy as np
from MAControl.Util.PointInRec import point_in_rec


class PolicyMaker_Auciton(PolicyMaker):

    Found_Target_Set = []  # {target_pos, target_vel, target_value, target_defence}
    Found_Target_Info = []  # {TARGET:UAV_INDEX}
    Remain_Target_Set = []  # {target_pos, target_vel, target_value, target_defence, TARGET_STATE}
    Remain_UAV_Set = []  # {UAV_STATE}

    def __init__(self, name, env, world, agent_index, arglist):
        super(PolicyMaker_Auciton, self).__init__(name, env, world, agent_index, arglist)
        self.opt_index = 0
        self.x = 0
        self.y = 0

        # 以下为一些阶段的初始设定步数，如果某一步需要用比预设更多的时间以达成某些要求，可以直接修改步数，从而延迟进入下一步
        self.Step0 = 1000  # 首次进入决策过程，对目标进行排序
        self.Step1 = 1001  # 紧接上一步，选择一个目标作为打击对象
        self.Step2 = 1002  # 紧接上一步，各UAV开始出价
        self.Step3 = 1012  # 经过一段时间的出价，各UAV统计出价结果
        self.Step4 = 1013  # 紧接上一步，各UAV进入攻击状态
        self.Step5 = 1014  # 紧接上一步，统计剩余目标和剩余UAV数量

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

    def make_policy(self, WorldTarget, obs_n, step):

        if step < self.Step0:
            print('<step0')
            pass

        elif step < self.Step1:
            print('step0')
            pass

        elif step < self.Step2:
            print('step1')
            pass

        elif step < self.Step3:
            print('step2')
            pass

        elif step < self.Step4:
            print('step3')
            pass

        elif step < self.Step5:
            print('step4')
            pass

        else:
            pass

        if random.random() > 0.999999:
            self.opt_index = 5
            self.x = random.random()
            self.y = random.random()
            print(self.index, 'attacking')

        return [self.opt_index, self.x, self.y]
