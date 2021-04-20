from MAControl.Base.PolicyMaker import PolicyMaker
import numpy as np
import math

.gitignore
class PolicyMaker_SelfOrganization(PolicyMaker):

    uav_in_sight = list()
    target_in_sight = list()

    def __init__(self, name, env, world, agent_index, arglist):
        super(PolicyMaker_SelfOrganization, self).__init__(name, env, world, agent_index, arglist)
        self.UD = [0, 0]                      # 存储决策(rule->BA)得出的速度期望
        self.seen_uavs = list()               # 个体视野中uav
        self.seen_targets = list()            # 个体视野中target
        self.pheromone = -1                   # uav 会将它更新为非负数. # 一直是 -1 表示自己是个target.
        self.uav_num = arglist.uav_num        # 小瓜子数量
        self.decision_frequency = 50
        self.rule_act = 0                     # 记录选中的行为原型序号 0-默认 1-主回转 2-主排斥 3-主吸引
        self.target_sense = 0
        self.sense_count = -1
        self.current_behavior = None
        PolicyMaker_SelfOrganization.uav_in_sight.append([])
        PolicyMaker_SelfOrganization.target_in_sight.append([])

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

                self.current_behavior = BEHAVIOR[2]

                _opt_index = 1

            else:
                pass

        else:  # target policy
            pass

        opt = [_opt_index, self.UD]
        return opt, self.current_behavior

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