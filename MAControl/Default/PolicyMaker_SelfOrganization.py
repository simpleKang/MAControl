from MAControl.Base.PolicyMaker import PolicyMaker
import numpy as np
import math
import MAControl.Default.Behavior_Archetype as BA


class PolicyMaker_SelfOrganization(PolicyMaker):

    def __init__(self, name, env, world, agent_index, arglist):
        super(PolicyMaker_SelfOrganization, self).__init__(name, env, world, agent_index, arglist)
        self.UD = [0, 0]                      # 存储决策(rule->BA)得出的速度期望

        self.uav_num = arglist.uav_num        # 小瓜子数量
        self.t_num = self.world.t_num         # 小花生数量
        self.seen_uavs = list()               # 个体视野中uav
        self.seen_targets = list()            # 个体视野中target

        self.pheromone = -1                   # (?)
        self.density = -1                     # 观测密度
        self.cluster = -1                     # 目标密度
        self.sense = 0                        # 记忆

        self.decision_frequency = 50          # 决策频率
        self.rule_act = 0                     # (?)

    def get_objects_in_sight(self, obs):

        _seen_uavs = list()
        _seen_targets = list()

        length = obs[self.index].__len__()
        num = int((length-6)/2)

        # 从环境里拿到的 observation 是 [bearing + index] 的形式
        # bearing 是真正的观测所得，而距离是未知的
        # index 作为接口来读取 agent 性质(target?UAV?)及其它认为可观测的量(heading)

        for i in range(num):
            bearing = obs[self.index][6+i*2]
            index = int(obs[self.index][7+i*2])
            if index < self.uav_num:
                _seen_uavs.append([bearing, math.atan2(obs[index][1], obs[index][0])])
            else:
                _seen_targets.append([bearing, math.atan2(obs[index][1], obs[index][0]),
                                      self.world.agents[index].alive])

        if self.index < self.uav_num:
            if _seen_targets.__len__() != 0:
                self.pheromone = 1
            else:
                self.pheromone = 0
        else:
            pass

        self.seen_uavs = _seen_uavs
        self.seen_targets = _seen_targets
        self.density = len(_seen_uavs)
        self.cluster = len(_seen_targets)

    def rule_summation(self, archetype, obs_n):

        W = archetype[4:]

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

        self.UD = UD

    def make_policy(self, obs_n, step):

        _opt_index = 0

        if self.index < self.uav_num:  # uav policy
            if step % self.decision_frequency == self.decision_frequency-1:
                self.get_objects_in_sight(obs_n)
            elif (step % self.decision_frequency == 1) and (step > self.decision_frequency):
                pheromone = self.pheromone
                density = self.density
                cluster = self.cluster
                sense = self.sense

                BEHAVIOR = [0, 0]
                for k, ba_k in enumerate(BA.SYS):
                    BA_k = pheromone * ba_k[0] + density * ba_k[1] + cluster * ba_k[2] + sense * ba_k[3]
                    BEHAVIOR = [BA_k, k] if BEHAVIOR[0] < BA_k else BEHAVIOR
                self.rule_summation(BA.SYS[BEHAVIOR[1]], obs_n)
                self.rule_act = BEHAVIOR[1]

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
            R1_list.append(uav[1])
        if R1_list:
            R1_k = sum(np.array(R1_list)) / len(R1_list)
            self_speed = np.linalg.norm(obs[self.index][0:2])
            R1 = [self_speed*math.cos(R1_k), self_speed*math.sin(R1_k)]
        else:
            R1 = [0, 0]
        return R1

    # >>>> Cohesion
    def rule2(self, obs):
        R2_list = list()
        for uav in self.seen_uavs:
            R2_list.append(uav[0])
        if R2_list:
            R2_k = sum(np.array(R2_list)) / len(R2_list)
            self_speed = np.linalg.norm(obs[self.index][0:2])
            R2 = [self_speed*math.cos(R2_k), self_speed*math.sin(R2_k)]
        else:
            R2 = [0, 0]
        return R2

    # >>>> Separation
    def rule3(self, obs):
        R3_list = list()
        for uav in self.seen_uavs:
            R3_list.append(uav[0])
        if R3_list:
            R3_k = sum(np.array(R3_list)) / len(R3_list)
            self_speed = np.linalg.norm(obs[self.index][0:2])
            R3 = [self_speed*math.cos(R3_k+math.pi), math.sin(R3_k+math.pi)]
        else:
            R3 = [0, 0]
        return R3

    # >>>> Mixed Evasion
    def rule4(self, obs):
        R4_list = list()
        for uav in self.seen_uavs:
            R4_list.append([math.cos(uav[0]) * math.cos(math.pi+uav[1]-uav[0]),
                            math.sin(uav[0]) * math.cos(math.pi+uav[1]-uav[0])])
        if R4_list:
            R4_k = sum(np.array(R4_list))
            R4_kk = math.atan2(R4_k[1], R4_k[0])
            self_speed = np.linalg.norm(obs[self.index][0:2])
            R4 = [self_speed*math.cos(R4_kk), math.sin(R4_kk)]
        else:
            R4 = [0, 0]
        return R4

    # >>>> One Target Attraction
    def rule5(self, obs):
        R5_list = list()
        for tar in self.seen_targets:
            R5_list.append(tar[0])
        if R5_list:
            R5_list_abs = np.array([abs(item) for item in R5_list])
            R5_k = R5_list[R5_list_abs.argmin()]
            self_speed = np.linalg.norm(obs[self.index][0:2])
            R5 = [self_speed*math.cos(R5_k), math.sin(R5_k)]
        else:
            R5 = [0, 0]
        return R5

    # >>>> One Target Repulsion
    def rule6(self, obs):
        R6_list = list()
        for tar in self.seen_targets:
            R6_list.append(tar[0])
        if R6_list:
            R6_list_abs = np.array([abs(item) for item in R6_list])
            R6_k = R6_list[R6_list_abs.argmin()]
            self_speed = np.linalg.norm(obs[self.index][0:2])
            R6 = [self_speed*math.cos(R6_k+math.pi), math.sin(R6_k+math.pi)]
        else:
            R6 = [0, 0]
        return R6

    # >>>> Multi Target Attraction
    def rule7(self, obs):
        R7_list = list()
        for tar in self.seen_targets:
            R7_list.append(tar[0])
        if R7_list:
            R7_k = sum(np.array(R7_list)) / len(R7_list)
            self_speed = np.linalg.norm(obs[self.index][0:2])
            R7 = [self_speed*math.cos(R7_k), math.sin(R7_k)]
        else:
            R7 = [0, 0]
        return R7

    # >>>> Multi Target Repulsion
    def rule8(self, obs):
        R8_list = list()
        for tar in self.seen_targets:
            R8_list.append(tar[0])
        if R8_list:
            R8_k = sum(np.array(R8_list)) / len(R8_list)
            self_speed = np.linalg.norm(obs[self.index][0:2])
            R8 = [self_speed*math.cos(R8_k+math.pi), math.sin(R8_k+math.pi)]
        else:
            R8 = [0, 0]
        return R8

    # >>>> Random
    def rule9(self, obs):
        R9_k = -1 * math.pi + np.random.randn() * 2 * math.pi
        self_speed = np.linalg.norm(obs[self.index][0:2])
        R9 = [self_speed * math.cos(R9_k), math.sin(R9_k)]
        return R9

    # >>>> Balance
    def rule10(self, obs):
        if len(self.seen_targets) * self.uav_num > len(self.seen_uavs) * self.t_num:
            print('t', self.t_num)
            R10 = self.rule2(obs)
        else:
            R10 = self.rule7(obs)
        return R10
