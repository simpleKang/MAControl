import numpy as np
import operator
import argparse
import random
import math
import os
import matplotlib.pyplot as plt
import MAControl.Util.SignIsSame as sis


# plt.rcParams['figure.dpi'] = 800
# curdir = os.path.dirname(__file__)
# pardir = os.path.dirname(os.path.dirname(curdir))
# data = np.loadtxt(pardir + '/cost.txt')
# plt.figure()
# line = plt.gca()
# line.plot(data[:, 0], data[:, 1], 'c--')
# # plt.xlim(0, 4000)
# plt.show()


# uav_pos = np.array([1, 1])
# tar_pos = np.array([1, 2])

def parse_args():
    parser = argparse.ArgumentParser("Control Experiments for Multi-Agent Environments")
    parser.add_argument("--evolve", action="store_false", default=True)
    parser.add_argument("--test", action="store_true", default=False)
    return parser.parse_args()


arglist = parse_args()
if arglist.evolve:
    print('Evolve')
if arglist.test:
    print('Test')


pass
