import random
import numpy as np


class RandomState(object):

    def __init__(self, collect, uav_num):
        self.collect_num = collect
        self.uav_num = uav_num
        self.pos_dim = 2
        self.vel_dim = 2

        self.seed = [i for i in range(0, 100000)]
        self.state_set = list()
        for k in range(self.collect_num):
            self.state_set.append([])
            for num in range(self.uav_num):
                self.state_set[k].append([])
                self.state_set[k][-1] = random.sample(self.seed, self.pos_dim+self.vel_dim)

    def get_state(self, collect, index):

        random.seed(self.state_set[collect][index][0])
        p_x = random.uniform(-1.8, -1.5)
        random.seed(self.state_set[collect][index][1])
        p_y = random.uniform(1.5, 1.8)

        random.seed(self.state_set[collect][index][2])
        v_x = random.uniform(0.0, 0.01)
        random.seed(self.state_set[collect][index][3])
        v_y = random.uniform(-0.01, 0.0)

        return np.array([p_x, p_y]), np.array([v_x, v_y])
