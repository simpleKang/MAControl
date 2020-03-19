import numpy as np
import os
import MAControl.Util.OnlineCoverRate as CR
import MAEnv.scenarios.TargetProfile as T
_path_ = '/cover_rate_Folder/' if os.name == 'posix' else '\\cover_rate_Folder\\'


def calculate_coverage(num, step, loop=0):

    curdir = os.path.dirname(__file__)
    pardir = os.path.dirname(os.path.dirname(curdir))

    txt_name = 'cover_rate-%s-%s-%s.txt' % (str(num), str(step), str(loop))
    open(pardir + _path_ + txt_name, 'w')

    cell = 200                     # 区域划分精度
    track = list()
    last_cover = list()
    for i in range(num):
        track.append(np.loadtxt(pardir + '/track/uav_%d_track.txt' % i))
        last_cover.append([])

    area = np.zeros((cell, cell))
    area_width = T.edge*2          # 正方形区域实际边长
    scale = area_width/cell        # 离散度(比例尺)
    iter_range = 0.25              # 迭代区域大小

    for l in range(0, step, 5):

        new_last = list()

        for k in range(num):

            new_last.append([])

            # 计算迭代区域的上下界
            x = track[k][l][2]
            y = track[k][l][3]

            # 将迭代区域上下界限制在区域内
            width_down = int(CR.constrain(round((T.edge - y - iter_range) / scale), 'down', 0))
            width_up = int(CR.constrain(round((T.edge - y + iter_range) / scale), 'up', cell))
            length_down = int(CR.constrain(round((T.edge + x - iter_range) / scale), 'down', 0))
            length_up = int(CR.constrain(round((T.edge + x + iter_range) / scale), 'up', cell))

            # 计算视场边界
            point1, point2, point3, point4 = CR.agent_cover_range(track[k][l])

            # 判断迭代区域内的点是否在探测范围内，更新区域离散矩阵
            for i in range(width_down, width_up):
                for j in range(length_down, length_up):
                    target = np.array([scale*j-T.edge, T.edge-scale*i])
                    if CR.point_in_rec(point1, point2, point3, point4, target):
                        if area[i][j] == 0:
                            area[i][j] = 1
                            new_last[k].append([i, j])
                        elif area[i][j] > 0 and [i, j] in last_cover[k]:
                            new_last[k].append([i, j])
                        elif area[i][j] > 0 and [i, j] not in last_cover[k]:
                            area[i][j] += 1
                            new_last[k].append([i, j])
                        else:
                            raise Exception('Unexpected situation!!!')
        last_cover[:] = new_last[:]
        print('>>> Collect', loop, ' >>> step ', l)

        cover_rate, overlap_rate = CR.cal_cover_rate(area)
        with open(pardir + _path_ + txt_name, 'a') as c:
            c.write(str(l) + ' ' + str(cover_rate) + ' ' + str(overlap_rate) + '\n')

    # np.savetxt(pardir + _path_ + 'area.text', area, fmt='%d')

    return cover_rate


if __name__ == '__main__':

    curdir_ = os.path.dirname(__file__)
    pardir_ = os.path.dirname(os.path.dirname(curdir_))
    para = np.loadtxt(pardir_ + '/track/para.txt')
    num_ = int(para[0])
    step_ = int(para[1])

    calculate_coverage(num_, step_)
