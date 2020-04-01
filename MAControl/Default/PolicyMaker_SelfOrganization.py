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
        self.pheromonal = -1                  # uav 会将它更新为非负数. # 一直是 -1 表示自己是个target.
        self.uav_sensor_range = 0.3
        self.target_sensor_range = 0.8
        self.uav_engagement_range = 0.5
        self.target_engagement_range = 0.5
        self.r1 = 0.3
        self.r2 = 0
        self.r3 = 1.1
        self.size = 0.03
        self.uav_num = arglist.uav_num        # 小瓜子数量
        self.decision_frequency = 50

    def get_objects_in_sight(self, obs):

        _seen_uavs = list()
        _seen_targets = list()

        length = obs[self.index].__len__()
        num = (length-4)/2

        # 从环境里拿到的 observation 是 [bearing + index] 的形式
        # bearing 是真正的观测所得，而距离是未知的
        # index 作为接口来读取 agent 性质(target?UAV?)及其它认为可观测的量(vel)

        for i in range(num):
            bearing = obs[self.index][4+i*2]
            index = obs[self.index][5+i*2]
            if index < self.uav_num:
                _seen_uavs.append([bearing, obs[index][0], obs[index][1]])
            else:
                _seen_targets.append([bearing, obs[index][0], obs[index][1]])

        if self.index < self.uav_num:
            if _seen_targets.__len__() != 0:
                self.pheromonal = 1
            else:
                self.pheromonal = 0
        else:
            pass

        self.seen_uavs = _seen_uavs
        self.seen_targets = _seen_targets

    def get_UAV_density(self, obs):  # further bearing -> lesser density
        item = list()

        for uav in self.seen_uavs:
            item.append(1/uav[0])

        _density = sum(item)
        return _density

    def rule_summation(self, ba_index, obs_n, obstacles):

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
        UR.append(np.array(self.rule10(obs_n, obstacles)))

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
                pheromonal = self.pheromonal
                density = self.get_UAV_density(obs_n)
                BEHAVIOR = [-10, -10]
                for k, ba_k in enumerate(BA.BA):
                    BA_k = pheromonal * ba_k[0] + density * ba_k[1]
                    BEHAVIOR = [BA_k, k] if BEHAVIOR[0] < BA_k else BEHAVIOR
                self.rule_summation(BEHAVIOR[1], obs_n, obstacles)
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
            R1 = sum(R1_list) / len(R1_list)
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
            R2_ = sum(R2_list) / len(R2_list)
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
            R3_ = sum(R3_list) / len(R3_list)
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
            R4_ = sum(R4_list) / len(R4_list)
            R4 = [-1*math.cos(R4_), -1*math.sin(R4_)]
        else:
            R4 = [0, 0]
        return R4

    # >>>> Flat Target Repulsion
    def rule6(self, obs):
        R6_list = list()
        for tar in self.seen_targets:
            bearing = tar[0]
            R6_list.append(bearing)
        if R6_list:
            R6_ = sum(R6_list) / len(R6_list)
            R6 = [math.cos(R6_), math.sin(R6_)]
        else:
            R6 = [0, 0]
        return R6

    # >>>> Flat Attraction
    def rule8(self, obs):
        R8_list = list()
        for tar in self.seen_targets:
            bearing = tar[0]
            R8_list.append(bearing)
        if R8_list:
            R8_ = sum(R8_list) / len(R8_list)
            R8 = [-1*math.cos(R8_), -1*math.sin(R8_)]
        else:
            R8 = [0, 0]
        return R8

    # >>>> Evasion
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

    # >>>> Obstacle Avoidance
    def rule10(self, obs, obstacles):

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

        # R10 = obs[self.index][0:2]
        return R10
