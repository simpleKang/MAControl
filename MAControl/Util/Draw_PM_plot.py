import numpy as np
import math
import os
import matplotlib.pyplot as plt
import MAEnv.scenarios.TargetProfile as T


plt.rcParams['figure.dpi'] = 200

curdir = os.path.dirname(__file__)
pardir = os.path.dirname(os.path.dirname(curdir))

uav_num = 8

track = []
for k in range(uav_num):
    PATH = pardir + '/track/A8-pi-E-Par/gen=7 ind=0 num=0'
    track.append(np.loadtxt(PATH + '/uav_%d_track.txt' % k))

EDK = list()
RDK = list()
EVK = list()
RVK = list()
POK = list()

d_ref = 2*1.0/math.sqrt(uav_num)
v_ref = 0.04

for lt in range(len(track[0])):

    edk = list()
    rdk = list()
    evk = list()
    rvk = list()
    pok = list()

    for i in range(uav_num):
        lenV_i = np.linalg.norm([track[lt][i][0], track[lt][i][1]])
        evk.append(abs(lenV_i-v_ref))
        rvk.append(lenV_i)
        for j in range(uav_num):
            dist_ij = np.linalg.norm([track[lt][i][2]-track[lt][j][2], track[lt][i][3]-track[lt][j][3]])
            dotV_ij = np.dot([track[lt][i][0], track[lt][i][1]], [track[lt][j][0], track[lt][j][1]])
            lenV_j = np.linalg.norm([track[lt][j][0], track[lt][j][1]])
            edk.append(abs(dist_ij-d_ref))
            rdk.append(dist_ij)
            pok.append(dotV_ij/lenV_i/lenV_j)

    EDK.append(sum(edk)/uav_num/uav_num/d_ref)
    RDK.append(max(rdk)/d_ref-min(rdk)/d_ref)
    EVK.append(sum(evk)/uav_num/v_ref)
    RVK.append(max(rvk)/v_ref-min(rvk)/v_ref)
    POK.append(sum(pok)/uav_num/uav_num)

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

for i in range(len(track)):
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
plt.savefig(pardir+'/track/-plot-/track.png')
plt.show()

