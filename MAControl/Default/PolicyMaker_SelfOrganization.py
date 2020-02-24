from MAControl.Base.PolicyMaker import PolicyMaker
from MAControl.Util.PointInRec import point_in_rec
import numpy as np
import math


class PolicyMaker_SelfOrganization(PolicyMaker):
    seen_targets = list()
    seen_uavs = list()
    pheromonal = list()

    def __init__(self, name, env, world, agent_index, arglist):
        super(PolicyMaker_SelfOrganization, self).__init__(name, env, world, agent_index, arglist)
        self.opt_index = 0                    # 操作数, 目前有0 / 1 / 10
        self.decision = 0                     # 决策内容
        PolicyMaker_SelfOrganization.seen_uavs.append(())           # 个体视野中uav
        PolicyMaker_SelfOrganization.seen_targets.append(())        # 个体视野中target
        PolicyMaker_SelfOrganization.pheromonal.append(-1)     # agent 会将它更新为非负数. # 一直是 -1 表示自己是个target.
        self.known_uavs = list()              # 视野+通信到的uav
        self.known_targets = list()           # 视野+通信到的target
        self.communication_range = 1
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

        _seen_uavs = list()
        _seen_targets = list()

        # 寻找视场内uav
        for i in range(self.uav_num):
            uav_pos = np.array(obs[i][2:4])
            if point_in_rec(selfview1, selfview2, selfview3, selfview4, uav_pos):
                _seen_uavs.append(i)

        # 寻找视场内target
        for i in range(obs.__len__() - self.uav_num):
            target_pos = np.array(obs[i+self.uav_num][2:4])
            if point_in_rec(selfview1, selfview2, selfview3, selfview4, target_pos):
                _seen_targets.append(i+self.uav_num)

        if (_seen_targets.__len__() != 0) and (self.index < self.uav_num):
            PolicyMaker_SelfOrganization.pheromonal[self.index] = 1

        # TODO  尚未考虑视线遮挡 [p98-(5.5)]

        PolicyMaker_SelfOrganization.seen_uavs[self.index] = _seen_uavs
        PolicyMaker_SelfOrganization.seen_targets[self.index] = _seen_targets

    def find_communication_mates(self, obs):
        _COMMates = list()

        self_pos = np.array(obs[self.index][2:4])
        for i in range(self.uav_num):
            uav_pos = np.array(obs[i][2:4])
            distance = np.linalg.norm(uav_pos-self_pos)
            if distance < self.communication_range:
                _COMMates.append(i)

        _COMMates.remove(self.index)
        return _COMMates

    def extend_known_objects(self, obs):
        _known_uavs = PolicyMaker_SelfOrganization.seen_uavs[self.index]
        _known_targets = PolicyMaker_SelfOrganization.seen_targets[self.index]
        _neighbor_pheromone = list()

        for k in self.find_communication_mates(obs):
            if k not in _known_uavs:
                _known_uavs.append(k)
            else:
                pass

            targets_k = PolicyMaker_SelfOrganization.seen_targets[k]
            _neighbor_pheromone.append(PolicyMaker_SelfOrganization.pheromonal[k])

            for t in targets_k:
                if t not in _known_targets:
                    _known_targets.append(t)
                else:
                    pass

        self.known_uavs = _known_uavs
        self.known_targets = _known_targets

        if PolicyMaker_SelfOrganization.pheromonal[self.index] != 1:
            _neighbor_pheromone.append(0)
            result = max(_neighbor_pheromone)/2
            result = 0 if result < 0.001 else result
            if self.index < self.uav_num:
                PolicyMaker_SelfOrganization.pheromonal[self.index] = result
            else:
                pass
        else:
            pass

    def make_policy(self, obstacles, obs_n, step):

        self.opt_index = 0

        opt = [self.opt_index, self.decision]

        return opt
