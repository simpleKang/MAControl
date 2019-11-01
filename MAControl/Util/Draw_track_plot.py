import numpy as np
import os
import matplotlib.pyplot as plt
import MAEnv.scenarios.TargetProfile as T

curdir = os.path.dirname(__file__)
pardir = os.path.dirname(os.path.dirname(curdir))

para = np.loadtxt(pardir + '/track/para.txt')
num = int(para[1])

track = []
for i in range(num):
    track.append(np.loadtxt(pardir + '/track/agent_%d_track.txt' % i))

# for k in range(-10, 1501, 25):

plt.figure()
line = plt.gca()
plt.xlim(-(T.edge+0.2), T.edge+0.2)
plt.ylim(-(T.edge+0.2), T.edge+0.2)
edge = np.array(([T.edge, T.edge], [T.edge, -T.edge], [-T.edge, -T.edge], [-T.edge, T.edge], [T.edge, T.edge]))
line.plot(edge[:, 0], edge[:, 1], 'r--')

color = ['b-', 'g-', 'r-', 'c-', 'm-', 'y-', 'k-',
         'b-.', 'g-.', 'r-.', 'c-.', 'm-.', 'y-.', 'k-.',
         'b:', 'g:', 'r:', 'c:', 'm:', 'y:', 'k:']

# TODO
pos = ['b', 'g', 'r', 'c', 'm', 'y', 'k',
       'b', 'g', 'r', 'c', 'm', 'y', 'k',
       'b', 'g', 'r', 'c', 'm', 'y', 'k']

for i in range(num):
    k = i
    if i >= 21:
        k = i - 21
    plt.scatter(track[i][0, 2], track[i][0, 3], c=pos[k], marker='o')
    line.plot(track[i][:, 2], track[i][:, 3], color[k])

# i = 9
# plt.scatter(track[i][0, 2], track[i][0, 3], c=pos[i], marker='o')
# line.plot(track[i][:, 2], track[i][:, 3], color[i])

plt.xlabel('X / km')
plt.ylabel('Y / km')
# plt.savefig('track %d.png' % k)
plt.show()

