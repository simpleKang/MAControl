import numpy as np
import tensorflow as tf
import operator
import random
import math
import os
import matplotlib.pyplot as plt
from matplotlib import colors

tar = np.array([[0, 1, 2],
                [6, 5, 8],
                [8, 7, 4]])

re = tar[0]

v1 = np.array([0.1, 0.2])
v2 = np.array([0.3, 0.4])
v3 = np.array([0.5, 0.6])
v4 = np.array([0.7, 0.8])

v = [v1, v2, v3, v4]

w = [1, 2, 3, 4]

list_i = [1, v]


def update_w(list_i, w):
    if list_i[0] == 1 and len(list_i[1]) == len(w):
        v = 0
        for i in range(len(w)):
            v += list_i[1][i]*w[i]
        list_i[1] = v

    return list_i


result = update_w(list_i, w)








pass
