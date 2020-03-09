import numpy as np


class GA():
    def __init__(self, arglist):
        print('ga init')
        self.pop_size = arglist.pop_size
        self.generation_num = arglist.generation_num
        self.collect_num = arglist.collect_num
        self.max_archetypes = arglist.max_behavior_archetypes
        self.crossover_rate = arglist.crossover_rate
        self.mutation_rate = arglist.mutation_rate
        self.mutation_neighborhood = arglist.mutation_neighborhood

        self.ba_c = 2
        self.ba_w = 9
        self.ba_r = 3

        # 初始化随机种群
        self.population = [[] for i in range(self.pop_size)]
        for individual in self.population:
            for ba_arch in range(self.max_archetypes):
                individual.append([])
                for weight in range(self.ba_c+self.ba_w+self.ba_r):
                    individual[-1].append(np.random.random())

        self.score = np.zeros((self.pop_size, self.collect_num))
        self.new_population = list()
        self.binary_population = list()

    def evolve(self):

        self.select()

        self.encode()

        self.crossover()

        self.mutate()

        self.decode()

        self.population = self.new_population.copy()

    # 选出部分个体进入下一代:对score进行加和操作并排序，取一半个体进入下一代
    def select(self):
        self.new_population.clear()
        self.binary_population.clear()
        pass

    # 交叉操作,交叉操作完成后，使得新种群数量与原种群保持一致
    def crossover(self):
        pass

    # 变异操作,不改变种群数量
    def mutate(self):
        pass

    # 将新种群编码为二进制形式
    def encode(self):
        self.binary_population = [[] for i in range(round(self.pop_size/2))]
        for individual in self.binary_population:
            for bit in range(5*self.max_archetypes*(self.ba_c+self.ba_w+self.ba_r)):
                individual.append(0 if np.random.random() < 0.5 else 1)

    # 解码操作,将二进制编码解码为权重形式
    def decode(self):
        pass

