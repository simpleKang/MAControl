import numpy as np
import os
import matplotlib.pyplot as plt
import pandas as pd


def get_box(data_num, name, gen):
    r = raw_data(data_num, name)
    box = pd.DataFrame(r[gen])

    return box


def raw_data(data_num, name):

    curdir = os.path.dirname(__file__)
    pardir = os.path.dirname(os.path.dirname(curdir))
    path = '/track/0527-output/' + name

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


def set_group_color(f, gen):

    for whisker in f['whiskers']:
        whisker.set(color=plt.get_cmap('Set3')(k), alpha=0.7, linewidth=0.5)
    for box in f['boxes']:
        box.set(color=plt.get_cmap('Set3')(k), alpha=0.7, linewidth=0.3)
        box.set(facecolor=plt.get_cmap('Set3')(k), alpha=0.7, linewidth=0.3)
    for median in f['medians']:
        median.set(color=plt.get_cmap('Set3')(k), alpha=0.95, linewidth=2)


if __name__ == '__main__':

    plt.rcParams['figure.dpi'] = 800
    data_num = 8
    folder_co = 'Test1-OK-A'
    # draw = [0, 1, 2, 3, 4, 5, 6, 7]
    draw = [0, 1, 2, 3, 4]
    # draw = [0]

    co = [[] for k in range(data_num)]
    for kk in range(len(draw)):
        k = draw[kk]
        control_box = get_box(data_num, folder_co, k)
        co[k] = control_box.boxplot(showfliers=False, patch_artist=True, showcaps=False, return_type='dict')
        set_group_color(co[k], k)

    k1_list = [i*20 for i in range(11)]  # actual
    k2_list = [i*100 for i in range(11)]  # show
    plt.xticks(k1_list, k2_list)

    font = {'family': 'Times New Roman', 'weight': 'normal', 'size': 13}
    plt.xlabel('Step', font)
    plt.ylabel('Cover-Rate', font)

    plt.xlim((0, 200))
    curdir = os.path.dirname(__file__)
    pardir = os.path.dirname(os.path.dirname(curdir))
    plt.savefig(pardir+'/track/-plot-/draw-o.png')
    plt.show()
