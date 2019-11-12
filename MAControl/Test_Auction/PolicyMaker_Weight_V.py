from MAControl.Base.PolicyMaker import PolicyMaker
from MAControl.Util.PointInRec import point_in_rec
from MAControl.Util.Constrain import constrain
import random
import numpy as np
import math
import operator
import os


class PolicyMaker_Weight_V(PolicyMaker):

    def __init__(self, name, env, world, agent_index, arglist):
        super(PolicyMaker_Weight_V, self).__init__(name, env, world, agent_index, arglist)
        self.opt_index = 0                    # 操作数, 目前有0 / 1 / 10
        self.decision = 0                     # 决策内容
        self.friends_in_sight = list()        # 决策前存储视野中友军
        self.targets_in_sight = list()        # 决策前存储视野中目标
        self.current_friends = list()         # 当前视野中友军
        self.current_targets = list()         # 当前视野中目标
        self.wait_step = -1                   # 决定开始决策前的等待步长
        self.init_step = 20                  # 初始前XX步内不进行任何决策
        self.after_decision_step = 0          # 决策后XX步内不进行任何决策
        self.num = env.n - len(world.movable_targets)   # 小瓜子数量
        # self.w = [0, 0, 0, 1]     # 权重 w1 / w2 / w3 / w4
        #
        # curdir = os.path.dirname(__file__)
        # pardir = os.path.dirname(os.path.dirname(curdir))
        # with open(pardir + '/track/para_w.txt', 'w') as f:
        #     f.write(str(self.w[0]) + '\n')
        #     f.write(str(self.w[1]) + '\n')
        #     f.write(str(self.w[2]) + '\n')

    def find_objects_in_sight(self, obs, worldtarget):

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

        self.current_friends.clear()
        self.current_targets.clear()

        # 寻找视场内友军
        for i in range(self.num):
            friends_pos = np.array(obs[i][2:4])
            if point_in_rec(selfview1, selfview2, selfview3, selfview4, friends_pos):
                self.current_friends.append(i)

        # 寻找视场内目标
        for target in worldtarget:
            targetpos = np.array(target[0:2])
            if point_in_rec(selfview1, selfview2, selfview3, selfview4, targetpos):
                self.current_targets.append(target)

    def v_1(self, obs):

        vel = obs[0:2]
        length = np.sqrt(vel.dot(vel))
        v1 = vel / length

        return v1

    def v_2(self, obs, target):

        pos = obs[2:4]
        sum_vec = np.array([0., 0.])
        for tar in target:
            tar_pos = tar[0:2]
            vec = tar_pos - pos
            sum_vec += vec
        length = np.sqrt(sum_vec.dot(sum_vec))
        v2 = sum_vec / length

        return v2

    def v_3(self, obs_n):

        selfangle = self.angle(obs_n[self.index][0:2])

        vel_angle = list()
        vel_angle.append(selfangle)

        for friend in self.friends_in_sight:
            friend_angle = self.angle(obs_n[friend][0:2])
            vel_angle.append(friend_angle)

        for i in range(len(vel_angle)):
            reverse_vec = vel_angle[i] + 180
            if reverse_vec > 360:
                reverse_vec -= 360
            vel_angle.append(reverse_vec)

        angle_normal = []
        for ang in vel_angle:
            nor = ang - selfangle
            if nor < 0:
                nor += 360
            angle_normal.append(nor)

        angle_normal.sort()
        angle_normal.append(360)

        plate = []
        for ele in angle_normal:
            plate.append(ele/360)

        bullet = random.random()
        k = 0
        while True:
            if plate[k] > bullet:
                k -= 1
                break
            k += 1

        desired_vel = random.randint(int(angle_normal[k]), int(angle_normal[k+1]))
        desired_vel += selfangle

        if desired_vel > 360:
            desired_vel -= 360

        vel_x = math.cos(desired_vel)
        vel_y = math.sin(desired_vel)

        v3 = np.array([vel_x, vel_y])

        return v3

    def v_4(self, obs_n):

        v4 = np.array([0., 0.])
        for index in self.friends_in_sight:
            v4 += obs_n[index][0:2]

        length = np.sqrt(v4.dot(v4))
        v4 /= length

        return v4

    def angle(self, vec):

        angle = math.atan2(vec[0], vec[1])
        angle_dgree = angle * 180 / math.pi
        if angle_dgree < 0:
            angle_dgree = 360 + angle_dgree

        return angle_dgree

    def make_policy(self, WorldTarget, obs_n, step):

        self.find_objects_in_sight(obs_n, WorldTarget)

        self.after_decision_step -= 1
        self.init_step -= 1

        self.opt_index = 0
        self.decision = 0

        if self.current_friends or self.current_targets:
            is_same_friends = operator.eq(self.current_friends, self.friends_in_sight)
            is_same_targets = operator.eq(self.current_targets, self.targets_in_sight)
            if (not is_same_friends) or (not is_same_targets):
                self.friends_in_sight = self.current_friends.copy()
                self.targets_in_sight = self.current_targets.copy()
                self.wait_step = step + 3

        if step == self.wait_step and self.init_step < 0 and self.after_decision_step < 0:

            v1 = self.v_1(obs_n[self.index])

            if self.targets_in_sight:
                v2 = self.v_2(obs_n[self.index], self.targets_in_sight)
            else:
                v2 = 0

            if self.friends_in_sight:
                v3 = self.v_3(obs_n)
                v4 = self.v_4(obs_n)
            else:
                v3 = 0
                v4 = 0

            # v = self.w[0]*v1 + self.w[1]*v2 + self.w[2]*v3 + self.w[3]*v4
            v = [v1, v2, v3, v4]

            self.opt_index = 1

            self.decision = v

            self.after_decision_step = 10

        opt = [self.opt_index, self.decision]

        return opt
