from MAControl.Base.PolicyMaker import PolicyMaker
import numpy as np
import random
import math
import os
path = '/track/' if os.name == 'posix' else '\\track\\'


class PolicyMaker_SO(PolicyMaker):

    def __init__(self, name, env, world, agent_index, arglist):
        super(PolicyMaker_SO, self).__init__(name, env, world, agent_index, arglist)
        self.UD = [0, 0]                              # 存储决策出的速度期望
        self.n_view_a = []                            # 个体视野中 neighborhood (mate)
        self.n_view_t = []                            # 个体视野中 neighborhood (target)
        self.p_views = []                             # 个体视野中 projection + gamma
        self.perception_quan = []                     # （量化）perception
        self.perception_dir = []                      # （指向性）perception
        self.uav_num = arglist.uav_num                # 小瓜子数量
        self.frequency = arglist.step_per_decision    # 小瓜子决策周期
        self.assigned = None                          # 小瓜子的最近可见目标 [ + memory ]

    def raw_input_extraction(self, obs):

        ob = obs[self.index]

        n_view = ob[6:41]
        p_views = ob[41:]

        n_view_a = []
        n_view_t = []
        n_view_list = [n_view[5*i:(5*i+5)] for i in range(7)]
        for i in range(7):
            if float('inf') in n_view_list[i]:
                pass
            elif math.isnan(n_view_list[i][0]):
                pass
            elif n_view_list[i][3] == 1:
                n_view_a.append(n_view_list[i])
            else:
                n_view_t.append(n_view_list[i])

        self.p_views = p_views
        self.n_view_a = n_view_a
        self.n_view_t = n_view_t

        if n_view_t:
            self.assigned = int(n_view_t[0][4])
        # 从环境里拿到的 每个 n_view 的倒数第二位表明了性质 mate / target
        # 从环境里拿到的 每个 p_view 都只表现区域的主要性质

    def perception(self, obs):
        self_ornt = math.atan2(obs[self.index][1], obs[self.index][0])

        p_view = self.p_views[0:-1]
        gamma = self.p_views[-1]
        n_view_a = self.n_view_a
        n_view_t = self.n_view_t

        # Neighbouring Mean Distance
        p1_a_list = [n_view_a[i][2] for i in range(len(n_view_a))]
        p1_t_list = [n_view_t[i][2] for i in range(len(n_view_t))]
        if p1_a_list or p1_t_list:
            p1 = (sum(p1_a_list) + sum(p1_t_list)) / (len(n_view_a) + len(n_view_t))
        else:
            p1 = 0

        # Neighbouring Agent Ratio
        if p1:
            p2 = len(n_view_a) / (len(n_view_a) + len(n_view_t))
        else:
            p2 = 0

        # Projected Agent Ratio
        p3 = list(p_view).count(1) / len(p_view)

        # Projected Target Ratio
        p4 = list(p_view).count(2) / 2 / len(p_view)

        # Neighbouring Agent Orientation
        p5_list = [n_view_a[i][4] for i in range(len(n_view_a))]
        if p5_list:
            p5 = sum(p5_list) / len(n_view_a)
        else:
            p5 = self_ornt

        # Neighbouring Agent Bearing
        p6_list = [n_view_a[i][0] + n_view_a[i][1] for i in range(len(n_view_a))]
        if p6_list:
            p6 = math.fmod(sum(p6_list) / 2 / len(n_view_a), math.pi)
        else:
            p6 = self_ornt

        # Neighbouring Target Bearing
        p7_list = [n_view_t[i][0] + n_view_t[i][1] for i in range(len(n_view_t))]
        if p7_list:
            p7 = math.fmod(sum(p7_list) / 2 / len(n_view_t), math.pi)
        else:
            p7 = self_ornt

        G2 = self_ornt + math.pi + gamma/2
        width = (2 * math.pi - gamma) / len(p_view)
        # Projected Agent Bearing
        p8_list = []
        for i in range(len(p_view)):
            if p_view[i] == 1:
                p8_list.append(G2 + (i+0.5) * width)
        if p8_list:
            p8 = sum(p8_list)/len(p8_list)
        else:
            p8 = self_ornt

        # Projected Target Bearing
        p9_list = []
        for i in range(len(p_view)):
            if p_view[i] == 2:
                p9_list.append(G2 + (i+0.5) * width)
        if p9_list:
            p9 = sum(p9_list)/len(p9_list)
        else:
            p9 = self_ornt

        self.perception_quan = [p1, p2, p3, p4]
        self.perception_dir = [p5, p6, p7, p8, p9]
        # ↑↑↑ represented with inertial angle, not unit vector

        # 保存小瓜子的 perception 信息
        pardir = os.path.dirname(os.path.dirname(os.path.dirname(__file__)))
        with open(pardir + path + 'uav_%d_perception.txt' % self.index, 'a') as f:
            f.write(str(p1) + ' ' + str(p2) + ' ' + str(p3) + ' ' +
                    str(p4) + ' ' + str(p5) + ' ' + str(p6) + ' ' +
                    str(p7) + ' ' + str(p8) + ' ' + str(p9) + ' ' +
                    str(gamma) + '\n')

    def rule_summation(self, archetype, obs_n):

        W = archetype[4:]
        p_dir = self.perception_dir

        UR = list()
        UR.append(np.array([math.cos(p_dir[0]), math.sin(p_dir[0])]))  # rule 1
        UR.append(np.array([math.cos(p_dir[1]), math.sin(p_dir[1])]))  # rule 2
        UR.append(np.array([math.cos(p_dir[2]), math.sin(p_dir[2])]))  # rule 3
        UR.append(np.array([math.cos(p_dir[3]), math.sin(p_dir[3])]))  # rule 4
        UR.append(np.array([math.cos(p_dir[4]), math.sin(p_dir[4])]))  # rule 5
        rand = -math.pi + 2*math.pi*random.random()
        UR.append(np.array([math.cos(rand), math.sin(rand)]))          # rule 6

        UD = np.array([0, 0])
        for i in range(len(UR)):
            UD = UD + W[i] * UR[i]

        self.UD = UD

    def make_policy(self, world, obstacles, obs_n, archetypes, step):

        opt_index = 0

        if self.index >= self.uav_num:  # target policy
            pass
        else:  # uav policy
            if not (step+1) % self.frequency == 0:
                pass
            else:  # ↓↓ policy cycle ↓↓
                opt_index = 1

                self.raw_input_extraction(obs_n)
                self.perception(obs_n)

                p_quan = self.perception_quan
                fun = [0 for i in range(len(archetypes))]
                for k, ba_k in enumerate(archetypes):
                    fun[k] = p_quan[0] * ba_k[0] + p_quan[1] * ba_k[1] + p_quan[2] * ba_k[2] + p_quan[3] * ba_k[3]
                s = fun.index(max(fun))
                self.rule_summation(archetypes[s], obs_n)

        opt = [opt_index, self.UD, self.assigned]
        return opt
