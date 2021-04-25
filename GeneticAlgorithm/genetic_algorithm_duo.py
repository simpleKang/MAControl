import numpy as np
import math
import random
import os

pardir = os.path.dirname(os.path.dirname(__file__))
path = '/GeneticAlgorithm/model/' if os.name == 'posix' else '\\GeneticAlgorithm\\model\\'


class GA(object):
    def __init__(self, arglist):

        self.collect_num = arglist.collect_num

        self.pop_size = arglist.pop_size
        self.max_archetypes = arglist.max_behavior_archetypes
        self.cr1 = arglist.crossover_rate_inner
        self.cr2 = arglist.crossover_rate_outer
        self.mr1 = arglist.mutation_rate_inner
        self.mr2 = arglist.mutation_rate_outer
        self.ba_c = 4  # amount of weight for quantitative perception
        self.ba_w = 5  # amount of weight for directional perception
        self.evolved_pop_size = self.pop_size + math.ceil(self.pop_size*self.cr2) + math.ceil(self.pop_size*self.mr2)

        self.bit = 5  # how many bits per weight representation - each weight \in (-1,1]
        self.mutation_p = arglist.mutation_neighborhood  # how many bits per weight is mutated

        if arglist.restore:
            self.load_model()
            print('Loading existing model.')

        else:
            self.population = [[] for i in range(self.evolved_pop_size)]
            for individual in self.population:
                # 随机初始化种群 # shape: evolved_pop_size ✖ max_archetypes ✖ (self.ba_c+self.ba_w)
                for arch in range(self.max_archetypes):
                    individual.append([])
                    for weight in range(self.ba_c+self.ba_w):
                        individual[-1].append(np.random.random())

        self.score = np.zeros((self.evolved_pop_size, self.collect_num))
        self.new_population = list()
        self.binary_population = list()
        print('GA initiation complete')

    def evolve(self, gen):

        self.select()

        self.encode()

        self.crossover()

        self.mutate()

        self.decode()

        self.population = self.new_population.copy()

        self.save_model(gen)

        self.save_pop(gen)

    # 选出部分个体进入下一代 from self.population
    def select(self):
        # now we have population + score [both of evolved pop size]

        self.new_population.clear()
        self.binary_population.clear()

        score_sum_individual = np.sum(self.score, axis=1)  # sum over collect_num
        score_sum = [[i, score_sum_individual[i]] for i in range(len(self.score))]
        score_sum = sorted(score_sum, key=lambda x: x[1], reverse=True)

        for i in range(self.pop_size):
            self.new_population.append(self.population[score_sum[i][0]])

    # 将新种群编码为二进制形式
    def encode(self):
        # now we have new population [of pop size] + [empty] binary population

        def dec2bin(dec):
            binary = list()
            bin_dec = bin(dec)
            for i in range(2, len(bin_dec)):
                binary.append(int(bin_dec[i]))
            for i in range(2 + self.bit - len(bin_dec)):
                binary.insert(0, 0)
            return binary

        self.binary_population = [[] for i in range(self.pop_size)]
        for ind, individual in enumerate(self.binary_population):
            for arch in range(self.max_archetypes):
                for c in range(self.ba_c + self.ba_w):
                    current_weight = int(math.ceil(self.new_population[ind][arch][c] * 2**(self.bit-1)
                                                   + 2**(self.bit-1) - 1))
                    individual += dec2bin(current_weight)

    # 交叉操作
    def crossover(self):
        # now we have binary population [of pop size]

        cross_num = math.ceil(self.pop_size * self.cr2)
        cross_w = math.ceil((self.ba_c + self.ba_w) * self.max_archetypes * self.cr1)

        for i in range(cross_num):
            parent = random.sample(range(0, len(self.binary_population)), 2)
            parent0 = self.binary_population[parent[0]]
            parent1 = self.binary_population[parent[1]]

            points = random.sample(range(0, (self.max_archetypes * (self.ba_c + self.ba_w))), cross_w)
            points.sort()

            child = parent0.copy()
            for ii in range(len(points)):
                k = points[ii]
                child[k*self.bit:(k+1)*self.bit] = parent1[k*self.bit:(k+1)*self.bit]
            self.binary_population += child

    # 变异操作
    def mutate(self):
        # now we have binary population [of pop size + crossover results]

        mutate_num = math.ceil(self.pop_size * self.mr2)
        mutate_w = math.ceil((self.ba_c + self.ba_w) * self.max_archetypes * self.mr1)

        # 采用双层随机位变异
        for i in range(mutate_num):
            parent = random.sample(range(0, len(self.binary_population)), 1)
            points = random.sample(range(0, (self.max_archetypes * (self.ba_c + self.ba_w))), mutate_w)

            child = self.binary_population[parent].copy()
            for ii in range(len(points)):
                k = points[ii]
                weight_k = child[k*self.bit:(k+1)*self.bit]
                bits = random.sample(range(0, self.bit), self.mutation_p)
                weight_k_new = [weight_k[u] if u in bits else 1-weight_k[u] for u in range(len(weight_k))]
                child[k * self.bit:(k + 1) * self.bit] = weight_k_new
            self.binary_population += child

    # 解码操作,将二进制编码解码为权重形式
    def decode(self):
        # now we have binary population [of evolved pop size]

        def bin2dec(binary_array):
            binary_array = [str(i) for i in binary_array]
            result = '0b' + ''.join(binary_array)
            result = int(result, 2)
            return result

        self.population.clear()
        for ind in range(len(self.binary_population)):
            individual = list()
            for b in range(0, self.max_archetypes * (self.ba_c + self.ba_w)):
                binary = self.binary_population[ind][(b*self.bit):((b+1)*self.bit)]
                weight_value = (bin2dec(binary) - 2**(self.bit-1) + 1) / 2**(self.bit-1)
                individual.append(weight_value)
            for arch in range(self.max_archetypes):
                individual_arch = individual[(self.ba_w+self.ba_c)*arch:(self.ba_w+self.ba_c)*(arch+1)]
                self.population.append(individual_arch)

    # 存储模型(可读性高)
    def save_pop(self, gen):

        score_sum = list()
        score_sum_individual = np.sum(self.score, axis=1)

        for i in range(self.pop_size):
            individual_score = [i, score_sum_individual[i]]
            score_sum.append(individual_score)
        score_sum = sorted(score_sum, key=lambda x: x[1], reverse=True)

        open(pardir + path + '/weight_model_gen_%d.txt' % gen, 'w')
        with open(pardir + path + '/weight_model_gen_%d.txt' % gen, 'a') as f:
            for i in range(self.pop_size):
                rank = score_sum[i][0]
                f.write('Average Score: ' + str(round(score_sum[i][1]/self.collect_num, 5)) + '\n')
                for arch in self.population[rank]:
                    f.write('Archetype: ')
                    for weight in arch:
                        f.write(str(round(weight, 3)) + ', ')
                    f.write('\n')
                f.write('\n' + '\n')
        pass

    # 载入模型
    def load_model(self):

        model = np.loadtxt(pardir + path + 'model.txt')

        self.population = list()
        x, y = model.shape
        if (x == self.pop_size*self.max_archetypes) and (y == self.ba_c + self.ba_w):
            for i in range(self.pop_size):
                self.population.append(list())
                for j in range(self.max_archetypes):
                    self.population[i].append(list())
                    for k in range(self.ba_c + self.ba_w):
                        self.population[i][j].append(model[i * self.max_archetypes + j][k])
        else:
            raise Exception('Check parameters.')

    # 存储模型(方便载入)
    def save_model(self, gen):

        open(pardir + path + 'model_%d.txt' % gen, 'w')
        with open(pardir + path + 'model_%d.txt' % gen, 'a') as f:
            for ind in self.population:
                for arch in ind:
                    for weight in arch:
                        f.write(str(weight) + ' ')
                    f.write('\n')
