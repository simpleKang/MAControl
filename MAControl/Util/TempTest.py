import numpy as np
import math
import os
import matplotlib.pyplot as plt

t = np.zeros((10, 20))

# x = np.linspace(0, 1, 101)
# y = -x * np.log2(x) - (1-x) * np.log2(1-x)
#
# y[np.isnan(y)] = 0
# plt.plot(x, y)
# plt.show()

d = os.path.dirname(__file__)

parent_path = os.path.dirname(d)

abspath = os.path.abspath(d)

z = 0
