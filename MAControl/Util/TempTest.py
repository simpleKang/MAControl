import numpy as np
import tensorflow as tf
import operator
import random
import math
import os
import matplotlib.pyplot as plt
import pandas as pd
import MAControl.Util.Draw_box_plot as bp

# plt.rcParams['figure.dpi'] = 800
# curdir = os.path.dirname(__file__)
# pardir = os.path.dirname(os.path.dirname(curdir))
# data = np.loadtxt(pardir + '/reward.txt')
# plt.figure()
# line = plt.gca()
# line.plot(data[:, 0], data[:, 1], 'c--')
# plt.xlim(0, 4000)
# plt.show()

# tar = np.array([[0, 1, 2],
#                 [6, 5, 8],
#                 [8, 7, 4]])
#
# v1 = np.array([1, 2])
# v2 = np.array([3, 4])
# v3 = np.array([5, 6])
# v4 = np.array([7, 8])

# v2 = np.array([0., 0.])
# v3 = np.array([0., 0.])
# v4 = np.array([0., 0.])

# v = [v1, v2, v3, v4]

# bp.calculate_median(30)

new_obs_temp = np.array([i for i in range(16)])

new_obs_temp = np.delete(new_obs_temp, [0, 1, 4, 5, 6, 7], axis=0)









pass
