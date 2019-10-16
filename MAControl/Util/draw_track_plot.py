import numpy as np
import matplotlib.pyplot as plt

para = np.loadtxt('/home/wzq/pycode/MAC/MAControl/track/para.txt')
num = int(para[1])

track = []
for i in range(num):
    track.append(np.loadtxt('/home/wzq/pycode/MAC/MAControl/track/agent_%d_track.txt' % i))

plt.figure()
line = plt.gca()
plt.xlim(-1.2, 1.2)
plt.ylim(-1.2, 1.2)
edge = np.array(([1, 1], [1, -1], [-1, -1], [-1, 1], [1, 1]))
line.plot(edge[:, 0], edge[:, 1], 'r--')

color = ['b-', 'g-', 'r-', 'c-', 'm-', 'y-', 'k-',
         'b-.', 'g-.', 'r-.', 'c-.', 'm-.', 'y-.', 'k-.',
         'b:', 'g:', 'r:', 'c:', 'm:', 'y:', 'k:']

for i in range(num):
    line.plot(track[i][:, 2], track[i][:, 3], color[i])

plt.xlabel('X / km')
plt.ylabel('Y / km')
plt.show()

