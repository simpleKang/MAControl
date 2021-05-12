import numpy as np
import os
import MAEnv.scenarios.TargetProfile as T
import math


def calculate_coverage(num, step, loop=0):

    curdir = os.path.dirname(__file__)
    pardir = os.path.dirname(os.path.dirname(curdir))

    txt_name = '/cover_rate-%s-%s-%s.txt' % (str(num), str(step), str(loop))
    open(pardir + '/cover_rate_Folder' + txt_name, 'w')

    cell = 200                     # 区域划分精度
    track = list()
    last_cover = list()
    for i in range(num):
        track.append(np.loadtxt(pardir + '/MAControl-dqn/track/agent_%d_track.txt' % i))
        last_cover.append([])

    area = np.zeros((cell, cell))
    area_width = T.edge*2          # 正方形区域实际边长
    scale = area_width/cell        # 离散度(比例尺)
    iter_range = 0.25              # 迭代区域大小

    for l in range(0, np.size(track[-1], 0), 5):

        new_last = list()
        angle_list = list()

        for k in range(num):

            new_last.append([])

            # 计算迭代区域的上下界
            x = round((track[k][l][2]+T.edge)/scale)
            y = round((track[k][l][3]+T.edge)/scale)
            v = [track[k][l][0], track[k][l][1]]
            angle1 = math.atan2(track[k][l][1], track[k][l][0])
            angle_max = angle1 + math.pi/3
            angle_min = angle1 - math.pi / 3
            for j in range(40):
                if (angle_min+j/25) <angle_max:
                    angle_list.append(angle_min+j/25)
                else:
                    angle_list.append(angle_max)
                    break
            for length in range(0,26):
                for angle in angle_list:
                    x_test = int(round(x + length*math.cos(angle)))
                    y_test = int(round(y + length*math.sin(angle)))
                    if x_test >= 200 or y_test >= 200:
                        break
                    if area[x_test][y_test] == 0:
                        area[x_test][y_test] = 1
        print('>>> Round', loop, 'Total ', np.size(track[-1], 0), ' >>> step ', l)

        cover_rate = np.sum(area)/cell/cell
        with open(pardir + '/MAControl-视觉/cover_rate_Folder' + txt_name, 'a') as c:
            c.write(str(l) + ' ' + str(cover_rate) + '\n')
    print('Finished!')

if __name__ == '__main__':

    curdir_ = os.path.dirname(__file__)
    pardir_ = os.path.dirname(os.path.dirname(curdir_))
    para = np.loadtxt(pardir_ + '/track/para.txt')
    num_ = int(para[0])
    step_ = int(para[1])

    calculate_coverage(10, step_)