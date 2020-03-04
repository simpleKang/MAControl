import numpy as np
import operator
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


uav_pos = np.array([1, 1])
tar_pos = np.array([1, 2])

uav_vel = np.array([0.01, 0.02])
tar_vel = np.array([0.03, 0.05])

dist = np.linalg.norm(tar_pos - uav_pos)

a = 0.1

print(uav_pos/(2*a))




pass
