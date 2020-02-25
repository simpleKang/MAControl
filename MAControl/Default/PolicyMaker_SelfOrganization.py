from MAControl.Base.PolicyMaker import PolicyMaker
from MAControl.Util.PointInRec import point_in_rec
import numpy as np
import math
import MAControl.Default.Behavior_Archetype as BA


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
                if i != self.index:
                    _seen_uavs.append(i)

        # 寻找视场内target
        for i in range(obs.__len__() - self.uav_num):
            target_pos = np.array(obs[i+self.uav_num][2:4])
            if point_in_rec(selfview1, selfview2, selfview3, selfview4, target_pos):
                if i != (self.index - self.uav_num):
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

    def get_UAV_density(self, obs):
        item = list()

        self_pos = np.array(obs[self.index][2:4])
        for i in self.known_uavs:
            uav_pos = np.array(obs[i][2:4])
            distance = np.linalg.norm(uav_pos-self_pos)
            item.append(1/distance)

        _density = sum(item)
        return _density

    def rule_summation(self, ba_index, obs_n):

        W = BA.BA[ba_index][2:]
        W.append(2)
        W.append(2)

        UR = list()
        UR.append(np.array(self.rule1(obs_n)))
        UR.append(np.array(self.rule2(obs_n)))
        UR.append(np.array(self.rule3(obs_n)))
        UR.append(np.array(self.rule4(obs_n)))
        UR.append(np.array(self.rule5(obs_n)))
        UR.append(np.array(self.rule6(obs_n)))
        UR.append(np.array(self.rule7(obs_n)))
        UR.append(np.array(self.rule8(obs_n)))
        UR.append(np.array(self.rule9(obs_n)))
        UR.append(np.array(self.rule10(obs_n)))

        URLength = [np.linalg.norm(UR[i]) for i in range(10)]
        threshold = sum(URLength) * 0.01

        UD = np.array([0, 0])

        for i in range(10):
            if URLength[i] > threshold:
                UD = UD + W[i] * UR[i] / URLength[i]
            else:
                pass

        return UD

    def make_policy(self, obstacles, obs_n, step):

        if self.index < self.uav_num:  # uav policy
            if step % 10 == 9:
                self.find_objects_in_sight(obs_n)
            elif step % 10 == 0:
                if step != 0:
                    self.extend_known_objects(obs_n)
            elif step % 10 == 1:
                pheromonal = PolicyMaker_SelfOrganization.pheromonal[self.index]
                density = self.get_UAV_density(obs_n)
                BEHAVIOR = [-10, -10]
                for k, ba_k in enumerate(BA.BA):
                    BA_k = pheromonal * ba_k[0] + density * ba_k[1]
                    BEHAVIOR = [BA_k, k] if BEHAVIOR[0] < BA_k else BEHAVIOR
                self.rule_summation(BEHAVIOR[1], obs_n)
            else:
                pass

        else:  # target policy
            pass

        opt = [self.opt_index, self.decision]

        return opt

    # 每个 rule 是一个单独的函数 利于融合代码
    # 输出为二维速度 输入可以修改

    # @XJ >>>> Alignment
    def rule1(self, obs):
        UD = obs[self.index][0:2]
        return UD

    # @XJ >>>> Target Orbit
    def rule2(self, obs):
        UD = obs[self.index][0:2]
        return UD

    # @XJ >>>> Cohesion
    def rule3(self, obs):
        UD = obs[self.index][0:2]
        return UD

    # @KSB >>>> Separation
    def rule4(self, obs):
        UD = obs[self.index][0:2]
        return UD

    # @WZQ >>>> Weighted Target Attraction
    def rule5(self, obs):
        UD = obs[self.index][0:2]
        return UD

    # @WZQ >>>> Flat Target Repulsion
    def rule6(self, obs):
        UD = obs[self.index][0:2]
        return UD

    # @WZQ >>>> Weighted Target Repulsion
    def rule7(self, obs):
        UD = obs[self.index][0:2]
        return UD

    # @WZQ >>>> Flat Attraction
    def rule8(self, obs):
        UD = obs[self.index][0:2]
        return UD

    # @KSB >>>> Evasion
    def rule9(self, obs):
        UD = obs[self.index][0:2]
        return UD

    # @XJ >>>> Obstacle Avoidance
    def rule10(self, obs):
        UD = obs[self.index][0:2]
        return UD
