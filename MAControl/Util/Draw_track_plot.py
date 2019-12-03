import numpy as np
import os
import matplotlib.pyplot as plt
import MAEnv.scenarios.TargetProfile as T

plt.rcParams['figure.dpi'] = 800

curdir = os.path.dirname(__file__)
pardir = os.path.dirname(os.path.dirname(curdir))

para = np.loadtxt(pardir + '/track/para.txt')
num = int(para[1])

track = []
for i in range(num):
    track.append(np.loadtxt(pardir + '/track/agent_%d_track.txt' % i))

# for k in range(-10, 1501, 25):

plt.figure(facecolor='w')
line = plt.gca()
line.patch.set_facecolor('black')
plt.xlim(-(T.edge+0.2), T.edge+0.2)
plt.ylim(-(T.edge+0.2), T.edge+0.2)
edge = np.array(([T.edge, T.edge], [T.edge, -T.edge], [-T.edge, -T.edge], [-T.edge, T.edge], [T.edge, T.edge]))
line.plot(edge[:, 0], edge[:, 1], 'r--')

color = ['black', 'darkorange', 'forestgreen', 'slategrey', 'lightcoral', 'gold', 'mediumturquoise', 'darkviolet',
         'gray', 'burlywood', 'limegreen', 'cornflowerblue', 'firebrick', 'khaki', 'teal', 'plum',
         'silver', 'darkgoldenrod', 'lime', 'slateblue', 'red', 'yellow', 'cyan', 'purple',
         'lightgrey', 'gold', 'turquoise', 'blueviolet', 'darksalmon', 'darkseagreen', 'deepskyblue', 'hotpink']

for i in range(num):
    line.plot(track[i][:, 2], track[i][:, 3], color='silver', linewidth=20)

for i in range(num):
    k = i % len(color)
    plt.scatter(track[i][0, 2], track[i][0, 3], c=color[k], marker='o')
    line.plot(track[i][:, 2], track[i][:, 3], color[k])

# i = 9
# plt.scatter(track[i][0, 2], track[i][0, 3], c=color[i], marker='o')
# line.plot(track[i][:, 2], track[i][:, 3], color[i])

plt.xlabel('X / km')
plt.ylabel('Y / km')
# plt.savefig('track %d.png' % k)
plt.show()
