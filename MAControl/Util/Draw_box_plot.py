import numpy as np
import math
import os
import matplotlib.pyplot as plt


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

    rr = [rr1, rr2, rr3, rr4]

    box = np.array(rr[tc])

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


if __name__ == '__main__':

    plt.rcParams['figure.dpi'] = 800

    data_num = 8
    str_list = [r'$N_A = 10$', r'$N_A = 15$', r'$N_A = 20$', r'$N_A = 25$']
    k_list = [r'$\frac{1}{3}\pi$', r'$\frac{2}{3}\pi$', r'$\pi$', r'$\frac{4}{3}\pi$']
    kr_list = [r'$\gamma = $' + item for item in k_list]

    for k in range(4):
        data_box = get_box(data_num, k)
        color_str = plt.get_cmap('BuGn')(k * 40 + 20)
        plt.hist(data_box, bins=100, facecolor=color_str, edgecolor='black', alpha=0.3, label=str_list[k])

    k1_list = [i/10 for i in range(11)]
    plt.xticks(k1_list, k1_list)

    font = {'family': 'Times New Roman', 'weight': 'normal', 'size': 13}
    plt.xlabel('Cover-Rate', font)
    plt.xlim((0.0, 1.0))
    plt.legend()

    curdir = os.path.dirname(__file__)
    pardir = os.path.dirname(os.path.dirname(curdir))
    plt.savefig(pardir+'/track/-plot-/draw-test.png')
    print('ok')
    plt.show()
