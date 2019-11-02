import numpy as np
import os
import MAControl.Util.OnlineCoverRate as CR

curdir = os.path.dirname(__file__)
pardir = os.path.dirname(os.path.dirname(curdir))

para = np.loadtxt(pardir + '/track/para.txt')
cell = int(para[0])
num = int(para[1])

para_w = np.loadtxt(pardir + '/track/para_w.txt')
txt_name = '/cover_rate-%s-%s-%s.txt' % (str(para_w[0]), str(para_w[1]), str(para_w[2]))
open(pardir + txt_name, 'w')

track = []
last_cover = []
for i in range(num):
    track.append(np.loadtxt(pardir + '/track/agent_%d_track.txt' % i))
    last_cover.append([])

area = np.zeros((cell, cell))
area_width = 2                 # 正方形区域实际边长
scale = area_width/cell        # 离散度(比例尺)
iter_range = 0.25              # 迭代区域大小


for l in range(0, np.size(track[-1], 0), 5):

    new_last = []

    for k in range(num):

        new_last.append([])

        # 计算迭代区域的上下界
        x = track[k][l][2]
        y = track[k][l][3]

        # 将迭代区域上下界限制在区域内
        width_down = int(CR.constrain(round((1 - y - iter_range) / scale), 'down', 0))
        width_up = int(CR.constrain(round((1 - y + iter_range) / scale), 'up', cell))
        length_down = int(CR.constrain(round((1 + x - iter_range) / scale), 'down', 0))
        length_up = int(CR.constrain(round((1 + x + iter_range) / scale), 'up', cell))

        # 计算视场边界
        point1, point2, point3, point4 = CR.agent_cover_range(track[k][l])

        # 判断迭代区域内的点是否在探测范围内，更新区域离散矩阵
        for i in range(width_down, width_up):
            for j in range(length_down, length_up):
                target = np.array([scale*j-1, 1-scale*i])
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
    print('Total ', np.size(track[-1], 0), ' >>> step ', l)

    cover_rate, overlap_rate = CR.cal_cover_rate(area)
    with open(pardir + txt_name, 'a') as c:
        c.write(str(l) + ' ' + str(cover_rate) + ' ' + str(overlap_rate) + '\n')

# np.savetxt(pardir + '/area.text', area, fmt='%d')

print('Finished!')
