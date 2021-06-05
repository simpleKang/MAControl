import numpy as np
import os
import matplotlib.pyplot as plt

plt.rcParams['figure.dpi'] = 200
curdir = os.path.dirname(__file__)
pardir = os.path.dirname(os.path.dirname(curdir))
path = '/track/Yard-C/'

raw = np.loadtxt(pardir + path + 'target-info-15-1000-4-C.txt', comments='#')

gen_list = list()
for ind in range(8):
    for loop in range(4):
        k_index = (ind * 4 + loop) * 200 + 0
        array = raw[k_index:k_index + 200].T[1]
        gen_list.append(list(array))

fig = plt.figure(facecolor='w')
X = [i for i in range(200)]
ax = fig.add_subplot(111)
# for count in range(32):
#    ax.plot(X, gen_list[count], label='count '+str(count))
ax.plot(X, gen_list[19], label='count '+str(19))

ax.legend()
plt.savefig(pardir+'/track/-plot-/CC.png')
plt.show()
