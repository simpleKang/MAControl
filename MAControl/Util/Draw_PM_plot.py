import numpy as np
import math
import os
import matplotlib.pyplot as plt

plt.rcParams['figure.dpi'] = 200
curdir = os.path.dirname(__file__)
pardir = os.path.dirname(os.path.dirname(curdir))

uav_num = 8
loop_num = 4
track = [[] for c in range(loop_num)]
for loop in range(loop_num):
    for k in range(uav_num):
        PATH = pardir + '/track/A8-pi-E-Par/gen=7 ind=0' + '/num=%d' % loop
        track[loop].append(np.loadtxt(PATH + '/uav_%d_track.txt' % k))

EDK = [[] for c in range(loop_num)]
RDK = [[] for c in range(loop_num)]
EVK = [[] for c in range(loop_num)]
RVK = [[] for c in range(loop_num)]
POK = [[] for c in range(loop_num)]

d_ref = 2*1.0/math.sqrt(uav_num)
v_ref = 0.04

for c in range(loop_num):
    for lt in range(len(track[0][0])):

        edk = list()
        rdk = list()
        evk = list()
        rvk = list()
        pok = list()

        for i in range(uav_num):
            lenV_i = np.linalg.norm([track[c][i][lt][0], track[c][i][lt][1]])
            evk.append(abs(lenV_i-v_ref))
            rvk.append(lenV_i)
            for j in range(uav_num):
                dist_ij = np.linalg.norm([track[c][i][lt][2]-track[c][j][lt][2], track[c][i][lt][3]-track[c][j][lt][3]])
                dotV_ij = np.dot([track[c][i][lt][0], track[c][i][lt][1]], [track[c][j][lt][0], track[c][j][lt][1]])
                lenV_j = np.linalg.norm([track[c][j][lt][0], track[c][j][lt][1]])
                edk.append(abs(dist_ij-d_ref))
                rdk.append(dist_ij)
                pok.append(dotV_ij/lenV_i/lenV_j)

        EDK[c].append(sum(edk)/uav_num/uav_num/d_ref)
        RDK[c].append(max(rdk)/d_ref-min(rdk)/d_ref)
        EVK[c].append(sum(evk)/uav_num/v_ref)
        RVK[c].append(max(rvk)/v_ref-min(rvk)/v_ref)
        POK[c].append(sum(pok)/uav_num/uav_num)

fig = plt.figure(facecolor='w')
X = [i for i in range(len(track[0][0]))]
color = [plt.get_cmap('tab10')(k) for k in range(5)]

EDK_max = np.array(EDK).max(axis=0)
EDK_min = np.array(EDK).min(axis=0)
EDK_00 = np.array(EDK).mean(axis=0)

RDK_max = np.array(RDK).max(axis=0)
RDK_min = np.array(RDK).min(axis=0)
RDK_00 = np.array(RDK).mean(axis=0)

EVK_max = np.array(EVK).max(axis=0)
EVK_min = np.array(EVK).min(axis=0)
EVK_00 = np.array(EVK).mean(axis=0)

RVK_max = np.array(RVK).max(axis=0)
RVK_min = np.array(RVK).min(axis=0)
RVK_00 = np.array(RVK).mean(axis=0)

POK_max = np.array(POK).max(axis=0)
POK_min = np.array(POK).min(axis=0)
POK_00 = np.array(POK).mean(axis=0)

ax = fig.add_subplot(111)
ax.plot(X, EDK_00, label='EDK: Inter Distance Error', color=color[0])
ax.fill_between(X, EDK_min, EDK_max, alpha=0.2, facecolor=color[0])
ax.plot(X, RDK_00, label='RDK: Inter Distance Range', color=color[1])
ax.fill_between(X, RDK_min, RDK_max, alpha=0.2, facecolor=color[1])
ax.plot(X, POK_00, label='POK: Order', color=color[2])
ax.fill_between(X, POK_min, POK_max, alpha=0.2, facecolor=color[2])
ax.legend()
ax.set_ylim(-0.5, 4.0)

ax2 = ax.twinx()
ax2.plot(X, EVK_00, label='EVK: Speed Error', color=color[3])
ax2.fill_between(X, EVK_min, EVK_max, alpha=0.2, facecolor=color[3])
ax2.plot(X, RVK_00, label='RVK: Speed Range', color=color[4])
ax2.fill_between(X, RVK_min, RVK_max, alpha=0.2, facecolor=color[4])
ax2.legend()
ax2.set_ylim(0, 0.2)

plt.xlim(30, 1000)
plt.savefig(pardir+'/track/-plot-/k.png')
print('ok')
plt.show()
