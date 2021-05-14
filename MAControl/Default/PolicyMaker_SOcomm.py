from MAControl.Base.PolicyMaker import PolicyMaker
from MAControl.Util.PointInRec import point_in_rec
import MAEnv.scenarios.TargetProfile as T
import numpy as np
import math


class PolicyMaker_SelfOrganization(PolicyMaker):
    seen_targets = list()
    seen_uavs = list()
    pheromone = list()
    target_in_sight = list()

    def __init__(self, name, env, world, agent_index, arglist):
        super(PolicyMaker_SelfOrganization, self).__init__(name, env, world, agent_index, arglist)
        self.UD = [0, 0]                      # 存储决策(rule->BA)得出的速度期望
        PolicyMaker_SelfOrganization.seen_uavs.append(())          # 个体视野中uav
        PolicyMaker_SelfOrganization.seen_targets.append(())       # 个体视野中target
        PolicyMaker_SelfOrganization.pheromone.append(-1)          # agent 会将它更新为非负数. # 一直是 -1 表示自己是个target.
        PolicyMaker_SelfOrganization.target_in_sight.append([])
        self.known_uavs = list()              # 视野+通信到的uav
        self.known_targets = list()           # 视野+通信到的target
        self.communication_range = 0.6
        self.uav_sensor_range = 0.6
        self.uav_engagement_range = 0.4
        self.target_sensor_range = 0.2
        self.target_engagement_range = 0.2
        self.size = 0.03
        self.uav_num = arglist.uav_num        # 小瓜子数量
        self.decision_frequency = 50
        self.current_behavior = None
        self.target_trap = 0

    def get_limited_vision(self, obs):  # 速度正常观测 # 位置未知>>>仅知方位
        limited_obs = list()
        for i in range(obs.__len__()):
            if i != self.index:
                relative_pos = np.array(obs[i][2:4])-np.array(obs[self.index][2:4])
                agent_bearing = relative_pos/np.linalg.norm(relative_pos)
            else:
                agent_bearing = np.array([0, 0])
            limited_ob = np.concatenate([obs[i][0:2]]+[agent_bearing]+[obs[i][4:]])
            limited_obs.append(limited_ob)
        return limited_obs

    def find_objects_in_sight(self, obs, world):

        # 计算视场区域
        selfvel = np.array(obs[self.index][0:2])
        selfpos = np.array(obs[self.index][2:4])
        selfvelunit = selfvel / np.sqrt(np.dot(selfvel, selfvel))
        selfdir = math.atan2(selfvel[1], selfvel[0])
        # d1 = 0.1  # 轴向视场距离
        # d2 = 0.2  # 轴向视场宽度
        # d3 = 0.2  # 侧向视场宽度
        d1 = self.uav_sensor_range*(-1)
        d2 = self.uav_sensor_range*2
        d3 = self.uav_sensor_range*2
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
            if world.agents[i].H > 0:
                uav_pos = np.array(obs[i][2:4])
                if point_in_rec(selfview1, selfview2, selfview3, selfview4, uav_pos):
                    if i != self.index:
                        _seen_uavs.append(i)

        # 寻找视场内target
        for i in range(obs.__len__() - self.uav_num):
            if world.agents[i+self.uav_num].H > 0:
                target_pos = np.array(obs[i+self.uav_num][2:4])
                if point_in_rec(selfview1, selfview2, selfview3, selfview4, target_pos):
                    if i != (self.index - self.uav_num):
                        _seen_targets.append(i+self.uav_num)

        if (_seen_targets.__len__() != 0) and (self.index < self.uav_num):
            PolicyMaker_SelfOrganization.pheromone[self.index] = 1

        # TO DO  尚未考虑视线遮挡 [p98-(5.5)]

        PolicyMaker_SelfOrganization.seen_uavs[self.index] = _seen_uavs
        PolicyMaker_SelfOrganization.seen_targets[self.index] = _seen_targets

    def find_communication_mates(self, obs, world):
        _COMMates = list()

        self_pos = np.array(obs[self.index][2:4])
        for i in range(self.uav_num):
            if world.agents[i].H > 0:
                uav_pos = np.array(obs[i][2:4])
                distance = np.linalg.norm(uav_pos-self_pos)
                if distance < self.communication_range:
                    _COMMates.append(i)

        _COMMates.remove(self.index)
        return _COMMates

    def extend_known_objects(self, obs, world):
        _known_uavs = PolicyMaker_SelfOrganization.seen_uavs[self.index]
        _known_targets = PolicyMaker_SelfOrganization.seen_targets[self.index]
        _neighbor_pheromone = list()

        for k in self.find_communication_mates(obs, world):
            if k not in _known_uavs:
                _known_uavs.append(k)
            else:
                pass

            targets_k = PolicyMaker_SelfOrganization.seen_targets[k]
            _neighbor_pheromone.append(PolicyMaker_SelfOrganization.pheromone[k])

            for t in targets_k:
                if t not in _known_targets:
                    _known_targets.append(t)
                else:
                    pass

        self.known_uavs = _known_uavs
        self.known_targets = _known_targets

        if PolicyMaker_SelfOrganization.pheromone[self.index] != 1:
            _neighbor_pheromone.append(0)
            result = max(_neighbor_pheromone)/2
            result = 0 if result < 0.001 else result
            if self.index < self.uav_num:
                PolicyMaker_SelfOrganization.pheromone[self.index] = result
            else:
                pass
        else:
            pass

    def get_UAV_density(self, obs):

        # item = list()
        # self_pos = np.array(obs[self.index][2:4])
        # for i in self.known_uavs:
        #     uav_pos = np.array(obs[i][2:4])
        #     distance = np.linalg.norm(uav_pos-self_pos)
        #     item.append(1/distance)
        # _density = sum(item)

        # _density = len(self.known_uavs) / (self.uav_num - 1)
        _density = len(self.known_uavs)

        return _density

    def get_pheromone(self, world):

        if self.known_targets:
            # _pheromone = world.agents[self.known_targets[0]].H / T.target_H[self.known_targets[0] - self.uav_num]
            _pheromone = T.target_H[self.known_targets[0] - self.uav_num]
        else:
            _pheromone = 0

        # _pheromone = PolicyMaker_SelfOrganization.pheromone[self.index]

        return _pheromone

    def rule_summation(self, archetype, obs_n, obstacles):

        W = archetype[2:10]
        W.append(2)
        W.append(2)

        r1 = archetype[-3]
        r2 = archetype[-2]
        r3 = archetype[-1]

        UR = list()
        UR.append(np.array(self.rule1(obs_n)))
        UR.append(np.array(self.rule2(obs_n)))
        UR.append(np.array(self.rule3(obs_n, r1)))
        UR.append(np.array(self.rule4(obs_n, r2)))
        UR.append(np.array(self.rule5(obs_n)))
        UR.append(np.array(self.rule6(obs_n)))
        UR.append(np.array(self.rule7(obs_n, r3)))
        UR.append(np.array(self.rule8(obs_n)))
        UR.append(np.array(self.rule9(obs_n)))
        UR.append(np.array(self.rule10(obs_n, obstacles)))

        URLength = [np.linalg.norm(UR[i]) for i in range(10)]
        threshold = sum(URLength) * 0.01

        UD = np.array([0, 0])

        for i in range(10):
            if URLength[i] > threshold:
                UD = UD + W[i] * UR[i] / URLength[i]
            else:
                pass

        # 至此完成(5.31)
        # 放弃实现(5.32) 如想限制速度上界 建议在 scenario6_AFIT.py 中写入约束
        self.UD = UD

    def make_policy(self, world, obstacles, obs_n, behavior_archetypes, step):

        _opt_index = 0

        if (self.index < self.uav_num) and (world.agents[self.index].H > 0):  # uav policy

            if step % self.decision_frequency == self.decision_frequency-1:
                self.find_objects_in_sight(obs_n, world)

            elif (step % self.decision_frequency == 0) and (step != 0):
                self.extend_known_objects(obs_n, world)

            elif (step % self.decision_frequency == 1) and (step > self.decision_frequency):

                pheromone = self.get_pheromone(world)
                density = self.get_UAV_density(obs_n)

                BEHAVIOR = [-10, -10]
                for k, ba_k in enumerate(behavior_archetypes):
                    BA_k = pheromone * ba_k[0] + density * ba_k[1]
                    BEHAVIOR = [BA_k, ba_k, k] if BEHAVIOR[0] < BA_k else BEHAVIOR

                # if self.known_targets:
                #     if density < pheromone:
                #         self.target_trap += 1
                #     else:
                #         self.target_trap -= 1
                #     if self.target_trap > 0:
                #         BEHAVIOR = [None, behavior_archetypes[2], 2]
                #     else:
                #         BEHAVIOR = [None, behavior_archetypes[1], 1]
                #
                # else:
                #     BEHAVIOR = [None, behavior_archetypes[0], 0]

                if self.known_targets:
                    PolicyMaker_SelfOrganization.target_in_sight[self.index].append(1)
                else:
                    PolicyMaker_SelfOrganization.target_in_sight[self.index].append(0)

                self.rule_summation(BEHAVIOR[1], obs_n, obstacles)
                self.current_behavior = BEHAVIOR[2]
                _opt_index = 1

                # self.UD = self.rule5(obs_n)
                # if self.known_targets:
                #     _opt_index = 10
                #     self.UD = [self.known_targets[0], obs_n[self.known_targets[0]][2:4]]

            else:
                pass

        else:  # target policy
            pass

        opt = [_opt_index, self.UD]
        return opt, self.current_behavior

    # 每个 rule 是一个单独的函数 利于融合代码
    # 输出为二维速度 输入可以修改

    # @KSB >>>> Alignment
    def rule1(self, obs):
        R1_list = list()
        self_pos = obs[self.index][2:4]
        for uav in self.known_uavs:
            uav_pos = obs[uav][2:4]
            uav_vel = obs[uav][0:2]
            dist = np.linalg.norm(self_pos - uav_pos)
            R1_list.append(uav_vel / dist)
        if R1_list:
            R1 = sum(R1_list) / len(R1_list)
        else:
            R1 = 0
        return R1

    # @WZQ >>>> Target Orbit
    def rule2(self, obs):
        R2_list = list()
        self_pos = obs[self.index][2:4]
        self_vel = obs[self.index][0:2]
        d1 = math.pi/2
        d2 = math.pi*3/2
        if self.known_targets:
            for tar in self.known_targets:
                tar_pos = obs[tar][2:4]
                dist = np.linalg.norm(tar_pos - self_pos)
                if dist >= 0.3*self.uav_sensor_range:
                    pos_vec = tar_pos - self_pos
                    d1_vec = np.array([pos_vec[0] * math.cos(d1) + pos_vec[1] * math.sin(d1),
                                       pos_vec[1] * math.cos(d1) - pos_vec[0] * math.sin(d1)])
                    d2_vec = np.array([pos_vec[0] * math.cos(d2) + pos_vec[1] * math.sin(d2),
                                       pos_vec[1] * math.cos(d2) - pos_vec[0] * math.sin(d2)])
                    dot_d1 = np.vdot(self_vel, d1_vec)
                    dot_d2 = np.vdot(self_vel, d2_vec)
                    if dot_d1 >= dot_d2:
                        R2_list.append(d1_vec)
                    else:
                        R2_list.append(d2_vec)
            if R2_list:
                R2 = sum(R2_list) / len(self.known_targets)
            else:
                R2 = 0
        else:
            R2 = 0
        return R2

    # @KSB >>>> Cohesion
    def rule3(self, obs, r1):
        R3_list = list()
        self_pos = obs[self.index][2:4]
        if self.known_uavs:
            for uav in self.known_uavs:
                uav_pos = obs[uav][2:4]
                dist = np.linalg.norm(self_pos - uav_pos)
                if dist > self.uav_sensor_range * r1:
                    R3_list.append((uav_pos - self_pos) * (dist - self.uav_sensor_range * r1))
            if R3_list:
                R3 = sum(R3_list) / len(self.known_uavs)
            else:
                R3 = 0
        else:
            R3 = 0
        return R3

    # @KSB >>>> Separation
    def rule4(self, obs, r2):
        R4_list = list()
        self_pos = obs[self.index][2:4]
        if self.known_uavs:
            for uav in self.known_uavs:
                uav_pos = obs[uav][2:4]
                dist = np.linalg.norm(self_pos - uav_pos)
                if dist < self.uav_sensor_range * r2:
                    R4_list.append((self_pos - uav_pos) * (self.uav_sensor_range * r2 - dist))
            if R4_list:
                R4 = sum(R4_list) / len(self.known_uavs)
            else:
                R4 = 0
        else:
            R4 = 0
        return R4

    # @WZQ >>>> Weighted Target Attraction
    def rule5(self, obs):
        R5_list = list()
        self_pos = obs[self.index][2:4]
        if self.known_targets:
            for tar in self.known_targets:
                tar_pos = obs[tar][2:4]
                dist = np.linalg.norm(tar_pos - self_pos)
                R5_list.append((tar_pos - self_pos) / dist**5)
            R5 = sum(R5_list) / len(self.known_targets)
        elif self.known_uavs:
            for uav in self.known_uavs:
                if PolicyMaker_SelfOrganization.pheromone[uav] > 0:
                    uav_pos = obs[uav][2:4]
                    dist = np.linalg.norm(uav_pos - self_pos)
                    R5_list.append(PolicyMaker_SelfOrganization.pheromone[uav] * (uav_pos - self_pos) / dist)
            if R5_list:
                R5 = sum(R5_list) / len(self.known_uavs)
            else:
                R5 = 0
        else:
            R5 = 0
        return R5

    # @WZQ >>>> Flat Target Repulsion
    def rule6(self, obs):
        R6_list = list()
        self_pos = obs[self.index][2:4]
        if self.known_targets:
            for tar in self.known_targets:
                tar_pos = obs[tar][2:4]
                dist = np.linalg.norm(tar_pos - self_pos)
                if (dist < 0.9 * self.uav_sensor_range) or (dist < self.target_sensor_range):
                    R6_list.append(self_pos - tar_pos)
            if R6_list:
                R6 = sum(R6_list) / len(self.known_targets)
            else:
                R6 = 0
        else:
            R6 = 0
        return R6

    # @WZQ >>>> Weighted Target Repulsion
    def rule7(self, obs, r3):
        R7_list = list()
        self_pos = obs[self.index][2:4]
        if self.known_targets:
            for tar in self.known_targets:
                tar_pos = obs[tar][2:4]
                dist = np.linalg.norm(tar_pos - self_pos)
                if (dist < r3 * self.uav_sensor_range) and \
                   (r3 * self.uav_sensor_range > self.target_engagement_range):
                    R7_list.append((self_pos - tar_pos) / 2 * (r3 * self.uav_sensor_range - dist))
                elif dist < self.target_engagement_range:
                    R7_list.append((self_pos - tar_pos) / 2 * (self.target_engagement_range - dist))
            if R7_list:
                R7 = sum(R7_list) / len(self.known_targets)
            else:
                R7 = 0
        else:
            R7 = 0
        return R7

    # @WZQ >>>> Flat Target Attraction
    def rule8(self, obs):
        R8_list = list()
        self_pos = obs[self.index][2:4]
        if self.known_targets:
            for tar in self.known_targets:
                tar_pos = obs[tar][2:4]
                dist = np.linalg.norm(tar_pos - self_pos)
                if dist >= 0.8*self.uav_sensor_range:
                    R8_list.append(tar_pos - self_pos)
            if R8_list:
                R8 = sum(R8_list)
            else:
                R8 = 0
        elif self.known_uavs:
            for uav in self.known_uavs:
                uav_pos = obs[uav][2:4]
                R8_list.append(uav_pos - self_pos)
            R8 = sum(R8_list)
        else:
            R8 = 0
        return R8

    # @KSB >>>> Evasion
    def rule9(self, obs):
        R9_list = list()
        min_dist = 0.5
        self_pos = obs[self.index][2:4]
        self_vel = obs[self.index][0:2]
        f_self_pos = self_pos + self_vel
        if self.known_uavs:
            for uav in self.known_uavs:
                uav_pos = obs[uav][2:4]
                uav_vel = obs[uav][0:2]
                dist = np.linalg.norm(self_pos - uav_pos)
                if dist > min_dist:
                    n_dist = dist
                else:
                    n_dist = min_dist
                f_uav_pos = uav_pos + uav_vel
                f_dist = np.linalg.norm(f_self_pos - f_uav_pos)
                if (f_dist < 3 * self.size) and (f_dist < n_dist):
                    R9_list.append(n_dist * (self_pos - uav_pos) / (3 * self.size))
            if R9_list:
                R9 = sum(R9_list) / len(self.known_uavs)
            else:
                R9 = 0
        else:
            R9 = 0
        return R9

    # @XJ >>>> Obstacle Avoidance
    def rule10(self, obs, obstacles):

        if obstacles:
            dUO = list()
            R10part2 = list()
            self_pos = np.array(obs[self.index][2:4])
            for item in obstacles:
                distance = np.linalg.norm(np.array(item[0:2])-self_pos)
                d = self.uav_sensor_range + item[2] - distance
                d = d if d > 0.000001 else 0.000001
                dUO.append(d)
                if (distance - item[2]) < self.uav_sensor_range/2:
                    R10part2.append(np.array(item[0:2])-self_pos)
                    R10part2[-1] = R10part2[-1] / distance * (distance - item[2])
                    R10part2[-1] = R10part2[-1] * dUO[-1] / self.uav_sensor_range * (-1)
                else:
                    R10part2.append([0, 0])
            sumdUO = sum(dUO)

            R10f = list()
            R10part1 = list()
            self_vel = np.array(obs[self.index][0:2])
            for k, item in enumerate(obstacles):
                vector1 = np.array(item[0:2])-self_pos
                cos1 = np.dot(vector1, self_vel)/np.linalg.norm(vector1)/np.linalg.norm(self_vel)
                if cos1 > 0:
                    vector2 = np.array([vector1[1], -1*vector1[0]])
                    vector3 = np.array([-1*vector1[1], vector1[0]])
                    cos2 = np.dot(vector2, self_vel)/np.linalg.norm(vector2)/np.linalg.norm(self_vel)
                    cos3 = np.dot(vector3, self_vel)/np.linalg.norm(vector3)/np.linalg.norm(self_vel)
                    if cos2 > cos3:
                        R10part1.append(vector2 * math.acos(cos1) * 2 / math.pi / np.linalg.norm(vector2))
                    else:
                        R10part1.append(vector3 * math.acos(cos1) * 2 / math.pi / np.linalg.norm(vector3))
                else:
                    R10part1.append([0, 0])
                if np.linalg.norm(vector1) < self.uav_sensor_range:
                    r10f = (np.array(R10part1[k]) + 0.1 * np.array(R10part2[k])) * dUO[k]
                else:
                    r10f = np.array([0, 0])
                R10f.append(r10f)
            sumR10f = sum(R10f)

            R10 = sumR10f / sumdUO
            if np.linalg.norm(R10) == 0:
                R10 = self_vel

        else:
            R10 = obs[self.index][0:2]
        return R10
