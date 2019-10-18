import numpy as np
import os
import matplotlib.pyplot as plt

curdir = os.path.dirname(__file__)
pardir = os.path.dirname(os.path.dirname(curdir))

data = np.loadtxt(pardir + '/cover_rate.txt')

plt.figure()
# plt.scatter(data[:, 0], data[:, 1], c='b', marker='o')
line = plt.gca()
line.plot(data[:, 0], data[:, 1], 'c--', label='Cover Rate')
plt.xlabel('step')
plt.ylabel('Cover Rate')
plt.legend()
plt.show()

line = plt.gca()
line.plot(data[:, 0], data[:, 2], 'r--', label='Overlap Rate')
plt.xlabel('step')
plt.ylabel('Overlap Rate')
plt.legend()
plt.show()

