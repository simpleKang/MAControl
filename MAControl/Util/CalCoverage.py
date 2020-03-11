#-*-coding:utf-8-*-

import numpy as np
import os
import time
import MAEnv.scenarios.TargetProfile as T
import MAControl.Util.OnlineCoverRate as CR


def calculate_coverage(num, step, loop=0):

    curdir = os.path.dirname(__file__)
    pardir = os.path.dirname(os.path.dirname(curdir))

    track = list()
    for i in range(num):
        track.append(np.loadtxt(pardir + '/track/uav_%d_track.txt' % i))

    cell = 200                     # 区域划分精度
    area = np.zeros((cell, cell))
    area_width = T.edge * 2        # 正方形区域实际边长
    scale = area_width / cell      # 离散度(比例尺)
    sensor_range = 0.3

    for s in range(0, step, 10):

        for n in range(num):

            # 计算迭代区域的上下界
            x = track[n][s][2]
            y = track[n][s][3]
            uav_pos = np.array([x, y])

            # 将迭代区域上下界限制在区域内
            width_down = int(CR.constrain(round((T.edge - y - sensor_range) / scale), 'down', 0))
            width_up = int(CR.constrain(round((T.edge - y + sensor_range) / scale), 'up', cell))
            length_down = int(CR.constrain(round((T.edge + x - sensor_range) / scale), 'down', 0))
            length_up = int(CR.constrain(round((T.edge + x + sensor_range) / scale), 'up', cell))

            for i in range(width_down, width_up):
                for j in range(length_down, length_up):
                    grid = np.array([scale * j - T.edge, T.edge - scale * i])
                    dist = np.linalg.norm(uav_pos - grid)
                    if dist <= sensor_range:
                        area[i][j] = 1

    coverage_, _ = CR.cal_cover_rate(area)

    return coverage_


if __name__ == '__main__':

    start = time.time()
    curdir_ = os.path.dirname(__file__)
    pardir_ = os.path.dirname(os.path.dirname(curdir_))
    para = np.loadtxt(pardir_ + '/track/para.txt')
    num_ = int(para[0])
    step_ = int(para[1])

    coverage = calculate_coverage(num_, step_)
    end = time.time()
    consume = end - start
    print(coverage, consume)
