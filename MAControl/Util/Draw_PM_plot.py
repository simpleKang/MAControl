import numpy as np
import math
import os
import matplotlib.pyplot as plt

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
        lenV_i = np.linalg.norm([track[i][lt][0], track[i][lt][1]])
        evk.append(abs(lenV_i-v_ref))
        rvk.append(lenV_i)
        for j in range(uav_num):
            dist_ij = np.linalg.norm([track[i][lt][2]-track[j][lt][2], track[i][lt][3]-track[j][lt][3]])
            dotV_ij = np.dot([track[i][lt][0], track[i][lt][1]], [track[j][lt][0], track[j][lt][1]])
            lenV_j = np.linalg.norm([track[j][lt][0], track[j][lt][1]])
            edk.append(abs(dist_ij-d_ref))
            rdk.append(dist_ij)
            pok.append(dotV_ij/lenV_i/lenV_j)

    EDK.append(sum(edk)/uav_num/uav_num/d_ref)
    RDK.append(max(rdk)/d_ref-min(rdk)/d_ref)
    EVK.append(sum(evk)/uav_num/v_ref)
    RVK.append(max(rvk)/v_ref-min(rvk)/v_ref)
    POK.append(sum(pok)/uav_num/uav_num)

fig = plt.figure(facecolor='w')
X = [i for i in range(len(track[0]))]

ax = fig.add_subplot(111)
ax.plot(X, EDK, label='EDK: Inter Distance Error', color=plt.get_cmap('tab10')(0))
ax.plot(X, RDK, label='RDK: Inter Distance Range', color=plt.get_cmap('tab10')(1))
ax.plot(X, POK, label='POK: Order', color=plt.get_cmap('tab10')(2))
ax.legend()
# ax.set_ylim(-1, 4)

ax2 = ax.twinx()
ax2.plot(X, EVK, label='EVK: Speed Error', color=plt.get_cmap('tab10')(3))
ax2.plot(X, RVK, label='RVK: Speed Range', color=plt.get_cmap('tab10')(4))
ax2.legend()
# ax2.set_ylim(0, 0.6)

plt.savefig(pardir+'/track/-plot-/k.png')
plt.show()

