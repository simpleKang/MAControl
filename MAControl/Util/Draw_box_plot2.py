import numpy as np
import os
import matplotlib.pyplot as plt
import pandas as pd


def get_box(data_num, name):
    r = raw_data(data_num, name)
    box = pd.DataFrame(r[0][0])

    return box


def raw_data(data_num, name):

    curdir = os.path.dirname(__file__)
    pardir = os.path.dirname(os.path.dirname(curdir))
    path = '/track/0527-output/' + name

    coverage_set = list()

    for i in range(data_num):
        raw = np.loadtxt(pardir + path + '/cover_rate-20-1000-%d.txt' % i, comments='#')
        for ind in range(8):
            k_index = [(ind * 4 + loop) * 200 + 0 for loop in range(4)]
            array = list()
            array.append(raw[k_index[0]:k_index[0] + 200].T[0])
            array.append(raw[k_index[0]:k_index[0] + 200].T[1])
            array.append(raw[k_index[1]:k_index[1] + 200].T[1])
            array.append(raw[k_index[2]:k_index[2] + 200].T[1])
            array.append(raw[k_index[3]:k_index[3] + 200].T[1])
            coverage_set.append(np.array(array).T)

    return coverage_set


def set_controlled_group_color(f, b_color='pink', me_color='crimson'):

    for whisker in f['whiskers']:
        whisker.set(color=b_color, alpha=0.5)
    for box in f['boxes']:
        box.set(color=b_color, alpha=0.5)
        box.set(facecolor=b_color, alpha=0.5)
    for median in f['medians']:
        median.set(color=me_color)


if __name__ == '__main__':

    plt.rcParams['figure.dpi'] = 800
    data_num_ = 8

    folder_co = 'Test1-OK-A'
    control_box = get_box(data_num_, folder_co)
    co = control_box.boxplot(showfliers=False, patch_artist=True, showcaps=False, return_type='dict')
    k1_list = [i*8 for i in range(17)]  # actual
    k2_list = [i for i in range(17)]  # show
    plt.xticks(k1_list, k2_list)

    set_controlled_group_color(co)

    plt.xlim((0, 150))
    curdir = os.path.dirname(__file__)
    pardir = os.path.dirname(os.path.dirname(curdir))
    plt.savefig(pardir+'/track/-plot-/draw1.png')
    plt.show()
