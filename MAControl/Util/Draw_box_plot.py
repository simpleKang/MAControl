import numpy as np
import math
import os
from collections import Counter
import matplotlib.pyplot as plt
import pandas as pd


def get_box(data_num, tc):

    r1 = raw_data(data_num, 'TestN2-OK-A-Partial', 10)
    r2 = raw_data(data_num, 'TestN1-OK-A-Partial', 15)
    r3 = raw_data(data_num, 'Test1-OK-A-Partial',  20)
    r4 = raw_data(data_num, 'TestN3-OK-A-Partial', 25)

    g = 5

    rr1 = [math.floor(r1[i][j][k] * 200) / 200 for k in range(200) for j in range(32) for i in range(g)]
    rr2 = [math.floor(r2[i][j][k] * 200) / 200 for k in range(200) for j in range(32) for i in range(g)]
    rr3 = [math.floor(r3[i][j][k] * 200) / 200 for k in range(200) for j in range(32) for i in range(g)]
    rr4 = [math.floor(r4[i][j][k] * 200) / 200 for k in range(200) for j in range(32) for i in range(g)]

    ct1 = Counter(rr1)
    ct2 = Counter(rr2)
    ct3 = Counter(rr3)
    ct4 = Counter(rr4)

    kr1 = [ct1[0.6+i*0.01] for i in range(41)]
    kr2 = [ct2[0.6+i*0.01] for i in range(41)]
    kr3 = [ct3[0.6+i*0.01] for i in range(41)]
    kr4 = [ct4[0.6+i*0.01] for i in range(41)]

    r = np.concatenate((rr1, rr2, rr3, rr4), axis=1)
    box = pd.DataFrame(r)

    return box


def raw_data(data_num, name, uav_num):

    curdir = os.path.dirname(__file__)
    pardir = os.path.dirname(os.path.dirname(curdir))
    path = '/track/' + name

    coverage_set = list()

    for i in range(data_num):
        raw = np.loadtxt(pardir + path + '/cover_rate-' + str(uav_num) + '-1000-%d.txt' % i, comments='#')
        gen_list = list()
        for ind in range(8):
            for loop in range(4):
                k_index = (ind * 4 + loop) * 200 + 0
                array = raw[k_index:k_index + 200].T[1]
                gen_list.append(list(array))
        coverage_set.append(gen_list)

    return coverage_set


def set_group_color(f, k):

    for whisker in f['whiskers']:
        whisker.set(color=plt.get_cmap('tab10')(k), alpha=0.5, linewidth=0.3)
    for box in f['boxes']:
        box.set(color=plt.get_cmap('tab10')(k), alpha=0.5, linewidth=0.3)
        box.set(facecolor=plt.get_cmap('tab10')(k), alpha=0.5, linewidth=0.3)
    for median in f['medians']:
        median.set(color=plt.get_cmap('tab10')(k), alpha=0.9, linewidth=2)


if __name__ == '__main__':

    plt.rcParams['figure.dpi'] = 1600
    data_num = 8

    co = [[] for k in range(4)]
    for k in range(4):
        control_box = get_box(data_num, k+1)
        co[k] = control_box.boxplot(showfliers=False, patch_artist=True, showcaps=False, return_type='dict')
        set_group_color(co[k], k)

    k1_list = [i*20 for i in range(9)]   # actual
    # k2_list = ['', r'$\gamma = \frac{1}{3}\pi$', '', r'$\gamma = \frac{2}{3}\pi$',
    #           '', r'$\gamma = \pi$', '', r'$\gamma = \frac{4}{3}\pi$', '']  # show
    k2_list = ['', r'$N_A = 10$', '', r'$N_A = 15$', '', r'$N_A = 20$', '', r'$N_A = 25$', '']  # show
    plt.xticks(k1_list, k2_list)

    font = {'family': 'Times New Roman', 'weight': 'normal', 'size': 13}
    plt.ylabel('Cover-Rate', font)

    plt.xlim((0, 160))
    plt.ylim((0.6, 1))
    curdir = os.path.dirname(__file__)
    pardir = os.path.dirname(os.path.dirname(curdir))
    plt.savefig(pardir+'/track/-plot-/draw-test.png')
    plt.show()
