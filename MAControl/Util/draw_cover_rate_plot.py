import numpy as np
import matplotlib.pyplot as plt

data = np.loadtxt('/home/wzq/pycode/MAC/MAControl/cover_rate.txt')

plt.figure()
# plt.scatter(data[:, 0], data[:, 1], c='b', marker='o')
line = plt.gca()
line.plot(data[:, 0], data[:, 1], 'c--', label='Cover Rate')
# line.plot(data[:, 0], data[:, 2], 'r--', label='Overlap Rate')
plt.xlabel('step')
plt.ylabel('Rate')
plt.legend()
plt.show()

line = plt.gca()
line.plot(data[:, 0], data[:, 2], 'r--', label='Overlap Rate')
plt.xlabel('step')
plt.ylabel('Rate')
plt.legend()
plt.show()

