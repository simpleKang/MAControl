import numpy as np
import math
import os
import matplotlib.pyplot as plt
from collections import Counter

plt.rcParams['figure.dpi'] = 200
curdir = os.path.dirname(__file__)
pardir = os.path.dirname(os.path.dirname(curdir))
path = '/track/Yard-C/'

raw = np.loadtxt(pardir + path + 'target-info-15-1000-4-C.txt', comments='#')
raw2 = np.loadtxt(pardir + path + 'score.txt', comments='#')
raw3 = np.loadtxt(pardir + path + 'assign.txt')

key_p = [0, 50, 100, 150, 200, 400, 600, 800]
assign_p = [raw3[k][1:] for k in key_p]
# 0   # 1 0 nan 3 0 3 nan 1 nan 0 3 3 nan 3 0
# 50  # 1 3  0  3 1 3  0  1  3  0 3 3 nan 3 0
# 100 # 1 3  0  3 0 3  0  3  3  0 0 3 nan 3 0
# 150 # 3 3  0  3 1 3  0  1  3  3 0 3  3  2 0
# 200 # 1 0  1  0 1 2  0  1  0  0 3 3  3  2 3
# 400 # 0 0  0  1 1 2  3  1  0  0 3 2  3  3 3
# 600 # 1 0  1  0 2 2  0  0  3  0 1 1  3  3 3
# 800 # 1 3  1  1 2 2  0  0  3  3 0 1  0  3 3
count_p = list()
for m in range(len(key_p)):
    cc = Counter(assign_p[m])
    count_p.append([cc[0], cc[1], cc[2], cc[3], cc[4]])
    count_p[-1].append(15-cc[0]-cc[1]-cc[2]-cc[3]-cc[4])

w_p = list()
s_p = list()
sc_p = list()
sct_p = list()
# N_A = 15 # N_T = 5 # R_T = 3
for m in range(len(key_p)):
    w = [math.exp(-1*abs(count_p[m][j]-3))/5 for j in range(5)]
    s = sum([-1*wj*math.log(wj) for wj in w])
    sc = (s*s)/(s*s+0.04)
    w_p.append(w)
    s_p.append(s)
    sc_p.append(sc)
    sct_p.append(sum(sc_p)/len(sc_p))

gen_list = list()
for ind in range(8):
    for loop in range(4):
        k_index = (ind * 4 + loop) * 200 + 0
        array = raw[k_index:k_index + 200].T[1]
        gen_list.append(list(array))

fig = plt.figure(facecolor='w')
X = [i*5 for i in range(200)]
ax = fig.add_subplot(111)
# for count in range(32):
#    ax.plot(X, gen_list[count], label='count '+str(count))
# ax.plot(X, gen_list[23], label='count '+str(23))
ax.plot(X, raw2.T[1], label='score')
ax.scatter(key_p, sc_p, label='sc', color='r', marker='*', s=100)
ax.scatter(key_p, sct_p, label='sct', color='m', marker='+', s=50)

ax.legend()
plt.savefig(pardir+'/track/-plot-/CC.png')
plt.show()
