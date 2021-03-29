import numpy as np
import os
import matplotlib.pyplot as plt
import pandas as pd


def draw_box_plot(data_num, name):

    curdir = os.path.dirname(__file__)
    pardir = os.path.dirname(os.path.dirname(curdir))

    coverage_set = list()

    for i in range(data_num):
        coverage_set.append(np.loadtxt(pardir + '/coverage-20-4000-%s/cover_rate-20-4000-%d.txt' % (str(name), i)))

    step, _ = coverage_set[0].shape

    box = list()

    for k in range(int(step)):

        box.append([])

        for j in range(data_num):

            box[-1].append(coverage_set[j][k][1])

    box = np.array(box).T

    box_ = pd.DataFrame(box)

    return box_


def set_controlled_group_color(f, b_color='pink', me_color='crimson'):

    for whisker in f['whiskers']:
        whisker.set(color=b_color, alpha=0.5)
    for box in f['boxes']:
        box.set(color=b_color, alpha=0.5)
        box.set(facecolor=b_color, alpha=0.5)
    for median in f['medians']:
        median.set(color=me_color)


def set_experimental_group_color(f, b_color='lightgreen', me_color='seagreen'):

    for whisker in f['whiskers']:
        whisker.set(color=b_color, alpha=0.3)
    for box in f['boxes']:
        box.set(color=b_color, alpha=0.3)
        box.set(facecolor=b_color, alpha=0.3)
    for median in f['medians']:
        median.set(color=me_color)


def set_random_group_color(f, b_color='cornflowerblue', me_color='blue'):

    for whisker in f['whiskers']:
        whisker.set(color=b_color, alpha=0.3)
    for box in f['boxes']:
        box.set(color=b_color, alpha=0.3)
        box.set(facecolor=b_color, alpha=0.3)
    for median in f['medians']:
        median.set(color=me_color)


def calculate_median(dataset):

    # 需要在根目录下放置Data文件夹,并在其内放好数据

    curdir = os.path.dirname(__file__)
    pardir = os.path.dirname(os.path.dirname(curdir))
    coverage_set = list()

    for i in range(dataset):
        coverage_set.append(np.loadtxt(pardir + '/Data/cover_rate-20-4000-%d.txt' % i))

    open(pardir + '/median.txt', 'w')

    shape, _ = coverage_set[0].shape

    for j in range(shape):

        median_cur = list()

        for k in range(dataset):
            median_cur.append(coverage_set[k][j][1])

        median_cur = np.array([median_cur])

        median = np.median(median_cur)

        with open(pardir + '/median.txt', 'a') as c:
            c.write(str(median) + '\n')

    print('Finished')


if __name__ == '__main__':

    plt.rcParams['figure.dpi'] = 800
    data_num_ = 30

    folder_co = 'controlled'
    control_box = draw_box_plot(data_num_, folder_co)
    co = control_box.boxplot(showfliers=False, patch_artist=True, showcaps=False, return_type='dict')
    plt.xticks([0, 200, 400, 600, 800], [0, 1000, 2000, 3000, 4000])

    folder_tr = 'experimental'
    trained_box = draw_box_plot(data_num_, folder_tr)
    tr = trained_box.boxplot(showfliers=False, patch_artist=True, showcaps=False, return_type='dict')
    plt.xticks([0, 200, 400, 600, 800], [0, 1000, 2000, 3000, 4000])

    folder_ra = 'random'
    random_box = draw_box_plot(data_num_, folder_ra)
    ra = random_box.boxplot(showfliers=False, patch_artist=True, showcaps=False, return_type='dict')
    plt.xticks([0, 200, 400, 600, 800], [0, 1000, 2000, 3000, 4000])

    set_controlled_group_color(co)
    set_experimental_group_color(tr)
    set_random_group_color(ra)

    plt.show()
