from MAControl.Base.PolicyMaker import PolicyMaker
import numpy as np
import math
import MAControl.Default.Behavior_Archetype as BA


class PolicyMaker_SelfOrganization(PolicyMaker):

    def __init__(self, name, env, world, agent_index, arglist):
        super(PolicyMaker_SelfOrganization, self).__init__(name, env, world, agent_index, arglist)
        self.UD = [0, 0]                      # 存储决策(rule->BA)得出的速度期望
        self.seen_uavs = list()               # 个体视野中uav
        self.seen_targets = list()            # 个体视野中target
        self.pheromone = -1                   # uav 会将它更新为非负数. # 一直是 -1 表示自己是个target.
        self.uav_num = arglist.uav_num        # 小瓜子数量
        self.decision_frequency = 50
        self.rule_act = [0, 0]                # 记录目标相关规则的触发与否，1-触发，0-未触发，[吸引,排斥]
        self.target_sense = 0                 # 0 是默认状况 # 被吸引过就会变成1 # 被排斥过就会变回0 # 其它时候保持记忆

    def get_objects_in_sight(self, obs):

        _seen_uavs = list()
        _seen_targets = list()

        length = obs[self.index].__len__()
        num = int((length-6)/2)

        # 从环境里拿到的 observation 是 [bearing + index] 的形式
        # bearing 是真正的观测所得，而距离是未知的
        # index 作为接口来读取 agent 性质(target?UAV?)及其它认为可观测的量(vel)

        for i in range(num):
            bearing = obs[self.index][6+i*2]
            index = int(obs[self.index][7+i*2])
            if index < self.uav_num:
                _seen_uavs.append([bearing, obs[index][0], obs[index][1]])
            else:
                _seen_targets.append([bearing, obs[index][0], obs[index][1], self.world.agents[index].H])

        if self.index < self.uav_num:
            if _seen_targets.__len__() != 0:
                self.pheromone = 1
            else:
                self.pheromone = 0
        else:
            pass

        self.seen_uavs = _seen_uavs
        self.seen_targets = _seen_targets

    def get_UAV_density(self, obs):

        # item = list()
        # for uav in self.seen_uavs:
        #    item.append(1/uav[0])  # further bearing -> lesser density
        # _density = sum(item)

        _density = len(self.seen_uavs)

        return _density

    def rule_summation(self, archetype, obs_n):

        W = archetype[2:]
        W.append(2)
        W.append(2)

        UR = list()
        UR.append(np.array(self.rule1(obs_n)))
        UR.append(np.array(self.rule2(obs_n)))
        UR.append(np.array(self.rule3(obs_n)))
        UR.append(np.array(self.rule4(obs_n)))
        UR.append(np.array(self.rule5(obs_n)))
        UR.append(np.array(self.rule6(obs_n)))
        UR.append(np.array([0, 0]))
        UR.append(np.array(self.rule8(obs_n)))
        UR.append(np.array(self.rule9(obs_n)))
        UR.append(np.array([0, 0]))

        URLength = [np.linalg.norm(UR[i]) for i in range(10)]
        threshold = sum(URLength) * 0.01

        UD = np.array([0, 0])

        for i in range(10):
            if URLength[i] > threshold:
                UD = UD + W[i] * UR[i] / URLength[i]
            else:
                pass

        self.UD = UD

    def make_policy(self, obstacles, obs_n, step):

        _opt_index = 0

        if self.index < self.uav_num:  # uav policy
            if step % self.decision_frequency == self.decision_frequency-1:
                self.get_objects_in_sight(obs_n)
            elif (step % self.decision_frequency == 1) and (step > self.decision_frequency):
                pheromone = self.pheromone
                density = self.get_UAV_density(obs_n)

                # BEHAVIOR = [-10, -10]
                # for k, ba_k in enumerate(BA.BA):
                #     BA_k = pheromone * ba_k[0] + density * ba_k[1]
                #     BEHAVIOR = [BA_k, k] if BEHAVIOR[0] < BA_k else BEHAVIOR
                # self.rule_summation(BA.BA[BEHAVIOR[1]], obs_n)

                # rule 1+3+4+9
                behavior = [0.88, 0.88, 1.00, 0.00, 1.00, 0.50, 0.00, 0.00, 0.00, 0.00]
                # rule 1+3+4+9 8+6+2
                behavior = [0.88, 0.88, 1.00, 0.95, 1.00, 0.50, 0.00, 0.55, 0.00, 0.95]
                # rule 1+3+4+9 8+6+5
                behavior = [0.88, 0.88, 1.00, 0.00, 1.00, 0.20, 0.95, 0.15, 0.00, 0.95]
                self.rule_summation(behavior, obs_n)

                _opt_index = 1
            else:
                pass

        else:  # target policy
            pass

        opt = [_opt_index, self.UD, self.rule_act]
        return opt

    # 每个 rule 是一个单独的函数 利于融合代码
    # 输出为二维速度 输入可以修改

    # >>>> Alignment
    def rule1(self, obs):
        R1_list = list()
        for uav in self.seen_uavs:
            uav_vel = uav[1:]
            R1_list.append(uav_vel)
        if R1_list:
            R1 = sum(np.array(R1_list)) / len(R1_list)
        else:
            R1 = [0, 0]
        return R1

    # >>>> Target Orbit
    def rule2(self, obs):
        R2_list = list()
        self_vel = obs[self.index][0:2]
        vel_bearing = math.atan2(self_vel[1], self_vel[0])
        for tar in self.seen_targets:
            bearing = tar[0]
            d1_vec = bearing + math.pi/2
            d2_vec = bearing - math.pi/2
            if abs(d1_vec-vel_bearing) <= abs(d1_vec-vel_bearing):
                R2_list.append(d1_vec)
            else:
                R2_list.append(d2_vec)
        if R2_list:
            R2_ = sum(np.array(R2_list)) / len(R2_list)
            R2 = [math.cos(R2_), math.sin(R2_)]
        else:
            R2 = [0, 0]
        return R2

    # >>>> Cohesion
    def rule3(self, obs):
        R3_list = list()
        for uav in self.seen_uavs:
            bearing = uav[0]
            R3_list.append(bearing)
        if R3_list:
            R3_ = sum(np.array(R3_list)) / len(R3_list)
            R3 = [math.cos(R3_), math.sin(R3_)]
        else:
            R3 = [0, 0]
        return R3

    # >>>> Separation
    def rule4(self, obs):
        R4_list = list()
        for uav in self.seen_uavs:
            bearing = uav[0]
            R4_list.append(bearing)
        if R4_list:
            R4_ = sum(np.array(R4_list)) / len(R4_list)
            R4 = [-1*math.cos(R4_), -1*math.sin(R4_)]
        else:
            R4 = [0, 0]
        return R4

    # >>>> Take turns when losing sight of target
    def rule5(self, obs):
        if (not self.seen_targets) and self.target_sense:
            self_vel = obs[self.index][0:2]
            vel_bearing = math.atan2(self_vel[1], self_vel[0])
            new_dir = vel_bearing - math.pi/3
            R5 = [np.linalg.norm(self_vel)*math.cos(new_dir), np.linalg.norm(self_vel)*math.sin(new_dir)]
        else:
            R5 = [0, 0]
        return R5

    # >>>> Flat Target Repulsion
    def rule6(self, obs):
        R6_list = list()
        for tar in self.seen_targets:
            bearing = tar[0]
            if tar[3] < len(self.seen_uavs):
                R6_list.append(bearing)
        if R6_list:
            R6_ = sum(np.array(R6_list)) / len(R6_list)
            R6 = [-1*math.cos(R6_), -1*math.sin(R6_)]
            self.rule_act[1] = 1
            self.target_sense = 0
        else:
            R6 = [0, 0]
            self.rule_act[1] = 0
        return R6

    # >>>> Flat Attraction
    def rule8(self, obs):
        R8_list = list()
        for tar in self.seen_targets:
            bearing = tar[0]
            if tar[3] > len(self.seen_uavs):
                R8_list.append(bearing)
        if R8_list:
            R8_ = sum(np.array(R8_list)) / len(R8_list)
            R8 = [math.cos(R8_), math.sin(R8_)]
            self.rule_act[0] = 1
            self.target_sense = 1
        else:
            R8 = [0, 0]
            self.rule_act[0] = 0
        return R8

    # >>>> Evasion
    def rule9(self, obs):
        R9_list = list()
        for uav in self.seen_uavs:
            bearing = uav[0]
            uav_vel = uav[1:]
            dot = np.dot([-1*math.cos(bearing), -1*math.sin(bearing)], uav_vel)
            if dot >= 0.0001:
                R9_list.append([-1*dot*math.cos(bearing), -1*dot*math.sin(bearing)])
            else:
                pass
        if R9_list:
            R9 = sum(np.array(R9_list)) / len(R9_list)
        else:
            R9 = [0, 0]
        return R9
