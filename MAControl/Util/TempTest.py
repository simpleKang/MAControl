import numpy as np
import tensorflow as tf
import operator
import random
import math
import os
import matplotlib.pyplot as plt
from matplotlib import colors

tar = [[0, 1, 2],
       [6, 5, 8],
       [8, 7, 4]]

w = [1, 2, 3, 4]

v1 = np.array([1, 1])
v2 = 0
v3 = np.array([3, 3])
v4 = np.array([4, 4])

v = [v1, v2, v3, v4]

he = np.array([0, 0])

for i in range(len(w)):
    v[i] = v[i]*w[i]
    he += v[i]


pass
