import numpy as np
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
    count_p.append([cc[0], cc[1], cc[2], cc[3]])
    count_p[-1].append(15-cc[0]-cc[1]-cc[2]-cc[3])

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
ax.plot(X, gen_list[23], label='count '+str(23))
ax.plot(X, raw2.T[1], label='score')

ax.legend()
plt.savefig(pardir+'/track/-plot-/CC.png')
plt.show()
