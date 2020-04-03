import numpy as np
import os
import matplotlib.pyplot as plt
import MAEnv.scenarios.TargetProfile as T


plt.rcParams['figure.dpi'] = 200

curdir = os.path.dirname(__file__)
pardir = os.path.dirname(os.path.dirname(curdir))

para = np.loadtxt(pardir + '/track/para.txt')
num = int(para[0])

track = []
for i in range(num):
    track.append(np.loadtxt(pardir + '/track/uav_%d_track.txt' % i))

# for k in range(-10, 1501, 25):

plt.figure(facecolor='w')
line = plt.gca()
line.set_aspect(1)
line.patch.set_facecolor('white')
plt.xlim(-(T.edge+0.2), T.edge+0.2)
plt.ylim(-(T.edge+0.2), T.edge+0.2)
edge = np.array(([T.edge, T.edge], [T.edge, -T.edge], [-T.edge, -T.edge], [-T.edge, T.edge], [T.edge, T.edge]))
line.plot(edge[:, 0], edge[:, 1], 'r--')

color = ['black', 'darkorange', 'forestgreen', 'slategrey', 'lightcoral', 'gold', 'mediumturquoise', 'darkviolet',
         'gray', 'burlywood', 'limegreen', 'cornflowerblue', 'firebrick', 'khaki', 'teal', 'plum',
         'silver', 'darkgoldenrod', 'lime', 'slateblue', 'red', 'yellow', 'cyan', 'purple',
         'lightgrey', 'gold', 'turquoise', 'blueviolet', 'darksalmon', 'darkseagreen', 'deepskyblue', 'hotpink']

for i in range(num):
    k = i % len(color)
    plt.scatter(track[i][0, 2], track[i][0, 3], c=color[k], marker='o')
    line.plot(track[i][:, 2], track[i][:, 3], color[k])

if T.num_square:
    for s in range(T.num_square):
        plt.scatter(T.square_pos[s][0], T.square_pos[s][1], c='k', marker='*', linewidths=20)

if T.num_targets:
    for t in range(T.num_targets):
        plt.scatter(T.target_pos[t][0], T.target_pos[t][1], c='b', marker='.', linewidths=10)

plt.xlabel('X / km')
plt.ylabel('Y / km')
plt.savefig('track.png')
plt.show()

