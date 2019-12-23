from MAControl.Base.PolicyMaker import PolicyMaker
from MAControl.Util.PointInRec import point_in_rec
import numpy as np
import math
import random
import operator

# [UP or DOWN, RIGHT or LEFT]
action_dict = {"0": [0., 1.],    # u
               "1": [1., 1.],    # u r
               "2": [1., 0.],    #   r
               "3": [1., -1.],   # d r
               "4": [0., -1.],   # d
               "5": [-1., -1.],  # d l
               "6": [-1., 0.],   #   l
               "7": [-1., 1.]}   # u l


class PolicyMaker_LocalDecision(PolicyMaker):

    def __init__(self, name, env, world, agent_index, arglist):
        super(PolicyMaker_LocalDecision, self).__init__(name, env, world, agent_index, arglist)
        self.opt_index = 0                    # 操作数, 目前有0 / 1 / 10
        self.decision = 0                     # 决策内容
        self.friends_in_sight = list()        # 决策前存储视野中友军
        self.current_friends = list()         # 当前视野中友军
        self.wait_step = -1                   # 决定开始决策前的等待步长
        self.init_step = 300                  # 初始前XX步内不进行任何决策
        self.after_decision_step = 100         # 决策后XX步内不进行任何决策
        self.num = arglist.agent_num          # 小瓜子数量
        self.is_decision = False
        self.state = np.zeros(self.num*2)
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

        self.current_friends.clear()

        # 寻找视场内友军
        for i in range(self.num):
            friends_pos = np.array(obs[i][2:4])
            if point_in_rec(selfview1, selfview2, selfview3, selfview4, friends_pos):
                self.current_friends.append(i)

    def unitization(self, obs):

        vel = obs[0:2]
        length = np.sqrt(vel.dot(vel))
        v1 = vel / length

        return v1

    def make_policy(self, WorldTarget, obs_n, step, trainer):

        self.find_objects_in_sight(obs_n)

        self.after_decision_step -= 1
        self.init_step -= 1

        self.state = np.zeros(self.num*2)
        self.is_decision = False
        self.opt_index = 0
        self.decision = 0

        if self.current_friends:
            is_same_friends = operator.eq(self.current_friends, self.friends_in_sight)
            if not is_same_friends:
                self.friends_in_sight = self.current_friends.copy()
                self.wait_step = step + 3

        if step == self.wait_step and self.init_step < 0 and self.after_decision_step < 0:

            if self.friends_in_sight:

                self.state[0:2] = self.unitization(obs_n[self.index])

                for num in range(len(self.friends_in_sight)):
                    self.state[(num+1)*2:(num+2)*2] = self.unitization(obs_n[self.friends_in_sight[num]])

                self.action_index = trainer.choose_action(self.state.reshape(1, len(self.state)))
                # self.action_index = random.randint(0, 7)

                self.decision = action_dict[str(int(self.action_index))]

                self.is_decision = True
                self.opt_index = 1
                self.after_decision_step = 100

        opt = [self.opt_index, self.decision]
        # 此轮是否进行决策 / 用于决策的状态 / 计算下一状态的辅助信息 / 决策输出的动作
        c_dis = [self.is_decision, self.state, self.friends_in_sight, self.action_index]

        return opt, c_dis
