from MAControl.Base.PolicyMaker import PolicyMaker
from MAControl.Util.PointInRec import point_in_rec
import numpy as np
import math


class PolicyMaker_SelfOrganization(PolicyMaker):

    def __init__(self, name, env, world, agent_index, arglist):
        super(PolicyMaker_SelfOrganization, self).__init__(name, env, world, agent_index, arglist)
        self.opt_index = 0                    # 操作数, 目前有0 / 1 / 10
        self.decision = 0                     # 决策内容
        self.seen_uavs = list()               # 当前视野中uav
        self.seen_targets = list()            # 当前视野中target
        self.wait_step = -1                   # 决定开始决策前的等待步长
        self.init_step = 300                  # 初始前XX步内不进行任何决策
        self.after_decision_step = 100        # 决策后XX步内不进行任何决策
        self.uav_num = arglist.uav_num        # 小瓜子数量
        self.is_decision = False
        self.state = np.zeros(self.uav_num*2)
        self.action_index = None

    def find_objects_in_sight(self, obs):

        # 计算视场区域
        selfvel = np.array(obs[self.index][0:2])
        selfpos = np.array(obs[self.index][2:4])
        selfvelunit = selfvel / np.sqrt(np.dot(selfvel, selfvel))
        selfdir = math.atan2(selfvel[1], selfvel[0])
        d1 = 0.1  # 轴向视场距离
        d2 = 0.2  # 轴向视场宽度
        d3 = 0.2  # 侧向视场宽度
        xx1 = -d3 / 2 * math.cos(selfdir) - d2 / 2 * math.sin(selfdir) * -1
        xx2 = -d3 / 2 * math.cos(selfdir) + d2 / 2 * math.sin(selfdir) * -1
        xx3 = d3 / 2 * math.cos(selfdir) + d2 / 2 * math.sin(selfdir) * -1
        xx4 = d3 / 2 * math.cos(selfdir) - d2 / 2 * math.sin(selfdir) * -1
        yy1 = -d3 / 2 * math.sin(selfdir) - d2 / 2 * math.cos(selfdir)
        yy2 = -d3 / 2 * math.sin(selfdir) + d2 / 2 * math.cos(selfdir)
        yy3 = d3 / 2 * math.sin(selfdir) + d2 / 2 * math.cos(selfdir)
        yy4 = d3 / 2 * math.sin(selfdir) - d2 / 2 * math.cos(selfdir)
        selfview1 = selfpos + selfvelunit * (d1 + d2 / 2) + np.array([xx1, yy1])
        selfview2 = selfpos + selfvelunit * (d1 + d2 / 2) + np.array([xx2, yy2])
        selfview3 = selfpos + selfvelunit * (d1 + d2 / 2) + np.array([xx3, yy3])
        selfview4 = selfpos + selfvelunit * (d1 + d2 / 2) + np.array([xx4, yy4])

        self.seen_uavs.clear()
        self.seen_targets.clear()

        # 寻找视场内uav
        for i in range(self.uav_num):
            uav_pos = np.array(obs[i][2:4])
            if point_in_rec(selfview1, selfview2, selfview3, selfview4, uav_pos):
                self.seen_uavs.append(i)

        # 寻找视场内target
        for i in range(obs.__len__() - self.uav_num):
            target_pos = np.array(obs[i+self.uav_num][2:4])
            if point_in_rec(selfview1, selfview2, selfview3, selfview4, target_pos):
                self.seen_targets.append(i)

        # TODO  尚未考虑视线遮挡

    def make_policy(self, WorldTarget, obs_n, step):

        self.opt_index = 0

        opt = [self.opt_index, self.decision]

        return opt
