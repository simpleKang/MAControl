import numpy as np
import os
import matplotlib.pyplot as plt
import pandas as pd


def get_box(data_num, name, gen, uav_num, stype):
    r = raw_data(data_num, name, uav_num, stype)
    box = pd.DataFrame(r[gen])

    return box


def raw_data(data_num, name, uav_num, stype):

    curdir = os.path.dirname(__file__)
    pardir = os.path.dirname(os.path.dirname(curdir))
    path = '/track/' + name

    coverage_set = list()
    str1 = '/cover-rate-' if stype == 'A' else '/target-info-'
    str2 = '' if stype == 'A' else '-' + str(stype)

    for i in range(data_num):
        raw = np.loadtxt(pardir + path + str1 + str(uav_num) + '-1000-%d' % i + str2 + '.txt', comments='#')
        gen_list = list()
        for ind in range(8):
            for loop in range(4):
                k_index = (ind * 4 + loop) * 200 + 0
                array = raw[k_index:k_index + 200].T[1]
                gen_list.append(list(array))
        coverage_set.append(gen_list)

    return coverage_set


def set_group_color(f, color_s):

    for whisker in f['whiskers']:
        whisker.set(color=color_s, alpha=0.5, linewidth=0.5)
    for box in f['boxes']:
        box.set(color=color_s, alpha=0.5, linewidth=0.3)
        box.set(facecolor=color_s, alpha=0.5, linewidth=0.3)
    for median in f['medians']:
        median.set(color=color_s, alpha=0.9, linewidth=2)


if __name__ == '__main__':

    plt.rcParams['figure.dpi'] = 160
    data_num = 8

    folder_ok = 'Pecp1-OK-B-Partial'
    folder_co = 'B-RPCBF-B'
    sr = 'B'  # 'A'  #  'C'
    un = 20
    draw = [0, 1, 2, 3, 4]

    co = [[] for k in range(data_num)]
    for kk in range(len(draw)):
        k = draw[kk]
        control_box = get_box(data_num, folder_ok, k, un, sr)
        co[k] = control_box.boxplot(showfliers=False, patch_artist=True, showcaps=False, return_type='dict')
        color_str = plt.get_cmap('ocean')(kk * 40 + 20)
        set_group_color(co[k], color_str)
        plt.text(161, kk*0.06 + 0.1, 'generation ' + str(kk), fontsize=10, weight='book', color=color_str)
        # A kk*0.06 + 0.1 # B kk*0.004 + 0.027 # C kk*0.04 + 0.22

    control_box = get_box(1, folder_co, 0, un, sr)
    co.append(control_box.boxplot(showfliers=False, patch_artist=True, showcaps=False, return_type='dict'))
    set_group_color(co[-1], 'red')
    plt.text(128, 0.5, 'RoleProjControlBirdFlock', fontsize=10, weight='book', color='r')  # A 0.5 # B 0.047 # C 0.92

    k1_list = [i*20 for i in range(11)]  # actual
    k2_list = [i*100 for i in range(11)]  # show
    plt.xticks(k1_list, k2_list)

    font = {'family': 'Times New Roman', 'weight': 'normal', 'size': 13}
    plt.xlabel('Step', font)
    plt.ylabel('Cover-Rate', font)  # A 'Cover-Rate' # B 'Perception-Ratio' # C 'Assignment-Score'

    plt.xlim((0, 200))
    plt.ylim((0.07, 1))  # A (0.07, 1) # B (-0.002, 0.055) # C (0.2, 1)
    curdir = os.path.dirname(__file__)
    pardir = os.path.dirname(os.path.dirname(curdir))
    plt.savefig(pardir+'/track/-plot-/draw-test.png')
    plt.show()
