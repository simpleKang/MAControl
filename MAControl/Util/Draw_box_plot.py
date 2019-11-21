import numpy as np
import os
import matplotlib.pyplot as plt
import pandas as pd


curdir = os.path.dirname(__file__)
pardir = os.path.dirname(os.path.dirname(curdir))

coverage_set = list()

for i in range(30):
    coverage_set.append(np.loadtxt(pardir + '/coverage-20-4000/cover_rate-20-4000-%d.txt' % i))

step, _ = coverage_set[0].shape

# plt.figure()

box = list()

for k in range(int(step)):

    box.append([])

    for j in range(30):

        box[-1].append(coverage_set[j][k][1])

# for x in range(step):
#     plt.boxplot(box[x], positions=[x*5])

array = np.array(box).T

box_ = pd.DataFrame(array)

f = box_.plot.box(showfliers=False, patch_artist=True, showcaps=False, return_type='dict')

for whisker in f['whiskers']:
    whisker.set(color='deepskyblue')
for box in f['boxes']:
    box.set(color='deepskyblue')
    box.set(facecolor='deepskyblue')
for median in f['medians']:
    median.set(color='lime')

plt.xticks([0, 200, 400, 600, 800], [0, 1000, 2000, 3000, 4000])
plt.show()














pass
