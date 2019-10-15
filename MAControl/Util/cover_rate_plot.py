import numpy as np
import matplotlib.pyplot as plt

data = np.loadtxt('/home/wzq/pycode/MAC/MAControl/cover_rate.txt')

plt.figure()
plt.scatter(data[:, 0], data[:, 1], c='b', marker='o')
line = plt.gca()
line.plot(data[:, 0], data[:, 1], 'r--')
plt.show()

