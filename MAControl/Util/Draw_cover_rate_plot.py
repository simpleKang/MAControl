import numpy as np
import os
import matplotlib.pyplot as plt

curdir = os.path.dirname(__file__)
pardir = os.path.dirname(os.path.dirname(curdir))

data = np.loadtxt(pardir + '/cover_rate.txt')

# for i in range(-10, 300, 10):

plt.figure()
line = plt.gca()
line.plot(data[:, 0], data[:, 1], 'c--')
plt.yticks([0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0])
# plt.xlim(0, 4000)
plt.ylim(0, 1)
plt.xlabel('step')
plt.ylabel('Cover Rate / %')
plt.title('Cover Rate')
# plt.legend()
# plt.savefig('cover %d.png' % i)
plt.show()

line = plt.gca()
line.plot(data[:, 0], data[:, 2], 'r--')
# plt.yticks([0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.0])
# plt.xlim(0, 4000)
# plt.ylim(0, 5.0)
plt.xlabel('step')
plt.ylabel('Overlap Rate / %')
plt.title('Overlap Rate')
# plt.legend()
# plt.savefig('overlap %d.png' % i)
plt.show()

