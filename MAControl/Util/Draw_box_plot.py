import numpy as np
import math
import os
import matplotlib.pyplot as plt


def get_box(tc):

    r1 = raw_data('A8-pi-E-Par',      8, 'A', [2, 3, 4, 5, 6])
    r2 = raw_data('A12-pi-E-Par',    12, 'A', [2, 3, 4, 5, 6])
    r3 = raw_data('A16-pi-E(r)-Par', 16, 'A', [0, 1, 2, 3, 4])
    r4 = raw_data('A20-pi-E-Par',    20, 'A', [0, 1, 2, 3, 4])

    g = 5

    rr1 = [math.floor(r1[i][j][m] * 200) / 200 for m in range(200) for j in range(32) for i in range(g)]
    rr2 = [math.floor(r2[i][j][m] * 200) / 200 for m in range(200) for j in range(32) for i in range(g)]
    rr3 = [math.floor(r3[i][j][m] * 200) / 200 for m in range(200) for j in range(32) for i in range(g)]
    rr4 = [math.floor(r4[i][j][m] * 200) / 200 for m in range(200) for j in range(32) for i in range(g)]

    rr = [rr1, rr2, rr3, rr4]

    box = np.array(rr[tc])

    return box


def raw_data(name, uav_num, stype, gens):

    curdir = os.path.dirname(__file__)
    pardir = os.path.dirname(os.path.dirname(curdir))
    path = '/track/' + name

    coverage_set = list()

    str1 = '/cover-rate-' if stype == 'A' else '/target-info-'
    str2 = '' if stype == 'A' else '-' + str(stype)

    for u in range(len(gens)):
        i = gens[u]
        raw = np.loadtxt(pardir + path + str1 + str(uav_num) + '-1000-%d' % i + str2 + '.txt', comments='#')
        gen_list = list()
        for ind in range(8):
            for loop in range(4):
                k_index = (ind * 4 + loop) * 200 + 0
                array = raw[k_index:k_index + 200].T[1]
                array = array / 100 if stype == 'B' else array
                gen_list.append(list(array))
        coverage_set.append(gen_list)

    return coverage_set


if __name__ == '__main__':

    plt.rcParams['figure.dpi'] = 200
    tr = 'NA'  # 'NA' # 'G'
    sr = 'A'  # 'A' # 'B' # 'C'
    lr = 'Cover-Rate'  # A 'Cover-Rate' # B 'Perception-Ratio' # C 'Assignment-Score'

    if tr == 'NA':
        str_list = [r'$N_A = 8$', r'$N_A = 12$', r'$N_A = 16$', r'$N_A = 20$']
        b_num = 100
    else:
        k_list = [r'$\frac{1}{3}\pi$', r'$\frac{2}{3}\pi$', r'$\pi$', r'$\frac{4}{3}\pi$']
        str_list = [r'$\gamma = $' + item for item in k_list]
        b_num = 80

    for k in range(4):
        data_box = get_box(k)
        color_str = plt.get_cmap('Dark2')(k)
        plt.hist(data_box, bins=b_num, facecolor=color_str, edgecolor='black', alpha=0.3, label=str_list[k])

    k1_list = [i/10 for i in range(11)]
    plt.xticks(k1_list, k1_list)

    font = {'family': 'Times New Roman', 'weight': 'normal', 'size': 13}
    plt.xlabel(lr, font)
    plt.xlim((0.0, 1.0))
    plt.legend()

    curdir = os.path.dirname(__file__)
    pardir = os.path.dirname(os.path.dirname(curdir))
    plt.savefig(pardir+'/track/-plot-/draw-test.png')
    print('ok')
    plt.show()
