import numpy as np
import os
import matplotlib.pyplot as plt
import pandas as pd


def get_box(data_num):

    r1 = raw_data(data_num, 'Test1-OK-A-Partial')
    r2 = raw_data(data_num, 'Test2-OK-A-Partial')
    r3 = raw_data(data_num, 'Test3-OK-A-Partial')
    r4 = raw_data(data_num, 'Test4-OK-A-Partial')

    rr1 = np.array([[r1[i][j][199] for j in range(32)] for i in range(8)]).T.reshape((4, 64), order='F')
    rr2 = np.array([[r2[i][j][199] for j in range(32)] for i in range(8)]).T.reshape((4, 64), order='F')
    rr3 = np.array([[r3[i][j][199] for j in range(32)] for i in range(8)]).T.reshape((4, 64), order='F')
    rr4 = np.array([[r4[i][j][199] for j in range(32)] for i in range(8)]).T.reshape((4, 64), order='F')

    r = np.concatenate((rr3, rr2, rr1, rr4), axis=1)
    box = pd.DataFrame(r)

    return box


def raw_data(data_num, name):

    curdir = os.path.dirname(__file__)
    pardir = os.path.dirname(os.path.dirname(curdir))
    path = '/track/' + name

    coverage_set = list()

    for i in range(data_num):
        raw = np.loadtxt(pardir + path + '/cover_rate-20-1000-%d.txt' % i, comments='#')
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
        whisker.set(color=plt.get_cmap('summer')(k), alpha=0.5, linewidth=0.3)
    for box in f['boxes']:
        box.set(color=plt.get_cmap('summer')(k), alpha=0.5, linewidth=0.3)
        box.set(facecolor=plt.get_cmap('summer')(k), alpha=0.5, linewidth=0.3)
    for median in f['medians']:
        median.set(color=plt.get_cmap('summer')(k), alpha=0.9, linewidth=2)


if __name__ == '__main__':

    plt.rcParams['figure.dpi'] = 1600
    data_num = 8

    co = [[] for k in range(4)]
    for k in range(4):
        control_box = get_box(data_num)
        co[k] = control_box.boxplot(showfliers=False, patch_artist=True, showcaps=False, return_type='dict')
        set_group_color(co[k], k)

    k1_list = [i*64 for i in range(5)]   # actual
    k2_list = ['', '>>> case 1', '>>> case 2', '>>> case 3', '>>> case 4']  # show
    plt.xticks(k1_list, k2_list)

    font = {'family': 'Times New Roman', 'weight': 'normal', 'size': 13}
    plt.xlabel('Step', font)
    plt.ylabel('Cover-Rate', font)

    plt.xlim((0, 256))
    plt.ylim((0.6, 1))
    curdir = os.path.dirname(__file__)
    pardir = os.path.dirname(os.path.dirname(curdir))
    plt.savefig(pardir+'/track/-plot-/draw-test.png')
    plt.show()
