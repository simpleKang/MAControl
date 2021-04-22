from MAControl.Base.PolicyMaker import PolicyMaker
import numpy as np
import math


class PolicyMaker_SelfOrganization(PolicyMaker):

    def __init__(self, name, env, world, agent_index, arglist):
        super(PolicyMaker_SelfOrganization, self).__init__(name, env, world, agent_index, arglist)
        self.UD = [0, 0]                      # 存储决策出的速度期望
        self.n_view_a = []                    # 个体视野中 neighborhood (mate)
        self.n_view_t = []                    # 个体视野中 neighborhood (target)
        self.p_view = []                      # 个体视野中 projection
        self.perception_quan = []             # （量化）perception
        self.perception_dir = []              # （指向性）perception
        self.uav_num = arglist.uav_num        # 小瓜子数量
        self.decision_frequency = 50          # 小瓜子决策周期

    def raw_input_extraction(self, obs):

        ob = obs[self.index]

        n_view = ob[6:41]
        p_view = ob[41:]

        n_view_a = []
        n_view_t = []
        n_view_list = [n_view[5*i:(5*i+5)] for i in range(7)]
        for i in range(7):
            if float('inf') in n_view_list[i]:
                pass
            elif math.isnan(n_view_list[i][0]):
                pass
            elif n_view_list[i][3] == 1:
                n_view_a.append(n_view_list[i][0:5])
            else:
                n_view_t.append(n_view_list[i][0:4])

        self.p_view = p_view
        self.n_view_a = n_view_a
        self.n_view_t = n_view_t

        # 从环境里拿到的 每个 n_view 的倒数第二位表明了性质 mate / target
        # 从环境里拿到的 每个 p_view 都只表现区域的主要性质

    def perception(self, obs):

        p_view = self.p_view
        n_view_a = self.n_view_a
        n_view_t = self.n_view_t

        # Neighbouring Mean Distance
        p1_a_list = [n_view_a[2] for i in range(len(n_view_a))]
        p1_t_list = [n_view_t[2] for i in range(len(n_view_t))]
        p1 = (sum(p1_a_list) + sum(p1_t_list)) / (len(n_view_a) + len(n_view_t))

        # Neighbouring Agent Ratio
        p2 = len(n_view_a) / (len(n_view_a) + len(n_view_t))

        # Projected Agent Ratio
        p3 = p_view.count(1) / len(p_view)

        # Projected Target Ratio
        p4 = p_view.count(2) / 2 / len(p_view)

        # Neighbouring Agent Orientation
        p5_list = [n_view_a[4] for i in range(len(n_view_a))]
        p5 = sum(p5_list) / len(n_view_a)

        # Neighbouring Agent Bearing
        p6_list = [n_view_a[0] + n_view_a[1] for i in range(len(n_view_a))]
        p6 = math.fmod(sum(p6_list) / 2 / len(n_view_a), math.pi)

        # Neighbouring Target Bearing
        p7_list = [n_view_t[0] + n_view_t[1] for i in range(len(n_view_t))]
        p7 = math.fmod(sum(p7_list) / 2 / len(n_view_t), math.pi)

        # Projected Agent Bearing
        p8 = []

        # Projected Target Bearing
        p9 = []

        self.perception_quan = [p1, p2, p3, p4]
        self.perception_dir = [p5, p6, p7, p8, p9]
        # ↑↑↑ represented with inertial angle, not unit vector

    def rule_summation(self, archetype, obs_n):

        W = archetype[2:-2]
        W.append(2)
        W.append(archetype[-1])

        UR = list()
        UR.append(np.array(self.rule1(obs_n)))
        UR.append(np.array(self.rule2(obs_n)))
        UR.append(np.array(self.rule3(obs_n)))
        UR.append(np.array(self.rule4(obs_n)))
        UR.append(np.array(self.rule5(obs_n)))
        UR.append(np.array(self.rule6(obs_n)))
        # UR.append(np.array([0, 0]))
        UR.append(np.array(self.rule8(obs_n)))
        UR.append(np.array(self.rule9(obs_n)))
        UR.append(np.array(self.rule10(obs_n)))
        # UR.append(np.array([0, 0]))

        URLength = [np.linalg.norm(UR[i]) for i in range(len(UR))]
        threshold = sum(URLength) * 0.01

        UD = np.array([0, 0])

        for i in range(len(UR)):
            if URLength[i] > threshold:
                UD = UD + W[i] * UR[i] / URLength[i]
            else:
                pass

        self.UD = UD

    def update_sense(self):
        if self.seen_targets:
            self.target_sense = 0
            self.sense_count = 10
        else:
            self.sense_count -= 1
            self.target_sense = 0
            pass
        if 0 < self.sense_count < 10:
            self.target_sense = 1

    def make_policy(self, world, obstacles, obs_n, behavior_archetypes, step):

        _opt_index = 0

        if self.index < self.uav_num:  # uav policy
            if step % self.decision_frequency == self.decision_frequency-1:
                self.get_objects_in_sight(obs_n)
            elif (step % self.decision_frequency == 1) and (step > self.decision_frequency):

                self.update_sense()
                pheromone = self.pheromone
                density = self.get_UAV_density(obs_n)
                neednum = sum([self.seen_targets[i][3] for i in range(len(self.seen_targets))])
                sense = self.target_sense

                BEHAVIOR = [-10, 0, 0]
                for k, ba_k in enumerate(behavior_archetypes):
                    BA_k = pheromone * ba_k[0] + density * ba_k[1] + neednum * ba_k[2] + sense * ba_k[3]
                    # BA_k = pheromone * ba_k[0] + density * ba_k[1]
                    BEHAVIOR = [BA_k, ba_k, k] if BEHAVIOR[0] < BA_k else BEHAVIOR
                self.rule_summation(BEHAVIOR[1], obs_n)

                # if self.seen_uavs:
                #     PolicyMaker_SelfOrganization.uav_in_sight[self.index].append(1)
                # else:
                #     PolicyMaker_SelfOrganization.uav_in_sight[self.index].append(0)
                if self.seen_targets:
                    PolicyMaker_SelfOrganization.target_in_sight[self.index].append(1)
                else:
                    PolicyMaker_SelfOrganization.target_in_sight[self.index].append(0)

                _opt_index = 1

            else:
                pass

        else:  # target policy
            pass

        opt = [_opt_index, self.UD]
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
        if self.target_sense:
            self_vel = obs[self.index][0:2]
            vel_bearing = math.atan2(self_vel[1], self_vel[0])
            new_dir = vel_bearing - math.pi/2
            R5 = [np.linalg.norm(self_vel)*math.cos(new_dir), np.linalg.norm(self_vel)*math.sin(new_dir)]
        else:
            R5 = [0, 0]
        return R5

    # >>>> Flat Target Repulsion
    def rule6(self, obs):
        R6_list = list()
        for tar in self.seen_targets:
            bearing = tar[0]
            # if tar[3] <= len(self.seen_uavs):
            R6_list.append(bearing)
        if R6_list:
            R6_ = sum(np.array(R6_list)) / len(R6_list)
            R6 = [-1*math.cos(R6_), -1*math.sin(R6_)]
        else:
            R6 = [0, 0]
        return R6

    # >>>> Flat Attraction
    def rule8(self, obs):
        R8_list = list()
        for tar in self.seen_targets:
            bearing = tar[0]
            # if tar[3] > len(self.seen_uavs):
            R8_list.append(bearing)
        if R8_list:
            R8_ = sum(np.array(R8_list)) / len(R8_list)
            R8 = [math.cos(R8_), math.sin(R8_)]
        else:
            R8 = [0, 0]
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

    # >>>> 斜列编队
    def rule10(self, obs):
        R10_list = list()
        signal = 0
        angle_self_v = math.atan2(obs[self.index][1], obs[self.index][0])
        for uav in self.seen_uavs:
            angle_u_v = math.atan2(uav[2], uav[1])
            if abs(angle_self_v - angle_u_v) <= math.pi/2:
                signal = 1
                bearing = uav[0]
                R10_list.append(bearing)
            else:
                signal = 0
                break

        if R10_list and signal == 1:
            R10_ = sum(np.array(R10_list)) / len(R10_list)
            angle_v = math.atan2(obs[self.index][1], obs[self.index][0])
            angle = angle_v - R10_
            if angle > 0 and angle <= math.pi/4:
                R10 = [-obs[self.index][1], obs[self.index][0]]
            elif angle <= 0 and angle >= -math.pi/4:
                R10 = [obs[self.index][1], -obs[self.index][0]]
            else:
                R10 = [0, 0]
        else:
            R10 = [0, 0]
        return R10