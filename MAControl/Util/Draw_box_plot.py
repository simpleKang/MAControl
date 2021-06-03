import numpy as np
import math
import os
import matplotlib.pyplot as plt


def get_box(data_num, tc):

    r1 = raw_data(data_num, 'TestN2-OK-A-Partial', 10, 'A')
    r2 = raw_data(data_num, 'TestN1-OK-A-Partial', 15, 'A')
    r3 = raw_data(data_num, 'Test1-OK-A-Partial',  20, 'A')
    r4 = raw_data(data_num, 'TestN3-OK-A-Partial', 25, 'A')

    g = 5

    rr1 = [math.floor(r1[i][j][m] * 200) / 200 for m in range(200) for j in range(32) for i in range(g)]
    rr2 = [math.floor(r2[i][j][m] * 200) / 200 for m in range(200) for j in range(32) for i in range(g)]
    rr3 = [math.floor(r3[i][j][m] * 200) / 200 for m in range(200) for j in range(32) for i in range(g)]
    rr4 = [math.floor(r4[i][j][m] * 200) / 200 for m in range(200) for j in range(32) for i in range(g)]

    rr = [rr1, rr2, rr3, rr4]

    box = np.array(rr[tc])

    return box


def raw_data(data_num, name, uav_num, stype):

    curdir = os.path.dirname(__file__)
    pardir = os.path.dirname(os.path.dirname(curdir))
    path = '/track/' + name

    coverage_set = list()

    str1 = '/cover_rate-' if stype == 'A' else '/target-info-'
    str2 = '' if stype == 'A' else '-' + str(stype)

    for i in range(data_num):
        raw = np.loadtxt(pardir + path + str1 + str(uav_num) + '-1000-%d.txt' % i + str2, comments='#')
        gen_list = list()
        for ind in range(8):
            for loop in range(4):
                k_index = (ind * 4 + loop) * 200 + 0
                array = raw[k_index:k_index + 200].T[1]
                gen_list.append(list(array))
        coverage_set.append(gen_list)

    return coverage_set


if __name__ == '__main__':

    plt.rcParams['figure.dpi'] = 160
    data_num = 8
    tr = 'NA'  # 'G'
    sr = 'A'  # 'B'  #  'C'

    if tr == 'NA':
        str_list = [r'$N_A = 10$', r'$N_A = 15$', r'$N_A = 20$', r'$N_A = 25$']
        b_num = 100
    else:
        k_list = [r'$\frac{1}{3}\pi$', r'$\frac{2}{3}\pi$', r'$\pi$', r'$\frac{4}{3}\pi$']
        str_list = [r'$\gamma = $' + item for item in k_list]
        b_num = 50

    for k in range(4):
        data_box = get_box(data_num, k)
        color_str = plt.get_cmap('Dark2')(k)
        plt.hist(data_box, bins=b_num, facecolor=color_str, edgecolor='black', alpha=0.3, label=str_list[k])

    k1_list = [i/10 for i in range(11)]
    plt.xticks(k1_list, k1_list)

    font = {'family': 'Times New Roman', 'weight': 'normal', 'size': 13}
    if sr == 'A':
        plt.xlabel('Cover-Rate', font)
    elif sr == 'B':
        plt.xlabel('Perception-Ratio', font)
    else:
        plt.xlabel('Assignment-Score', font)
    plt.xlim((0.0, 1.0))
    plt.legend()

    curdir = os.path.dirname(__file__)
    pardir = os.path.dirname(os.path.dirname(curdir))
    plt.savefig(pardir+'/track/-plot-/draw-test.png')
    print('ok')
    plt.show()
