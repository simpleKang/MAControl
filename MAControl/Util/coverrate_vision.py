import numpy as np
import os
import MAEnv.scenarios.TargetProfile as T
import math
gamma = T.blind_angle[2]


def calculate_coverage(uav_num, step, loop=0):

    curdir = os.path.dirname(__file__)
    pardir = os.path.dirname(os.path.dirname(curdir))
    cover_rate = 0

    txt_name = '/cover_rate-%s-%s-%s.txt' % (str(uav_num), str(step), str(loop))
    open(pardir + '/scene_Folder' + txt_name, 'w')

    cell = 200                     # 区域划分精度
    track = list()
    for i in range(uav_num):
        track.append(np.loadtxt(pardir + '/track/uav_%d_track.txt' % i))

    area = np.zeros((cell, cell))
    area_width = T.edge*2          # 正方形区域实际边长
    scale = area_width/cell        # 离散度(比例尺)

    for lt in range(0, np.size(track[-1], 0), 5):  # np.size(A,0) 返回该二维矩阵的行数 # range(a,b,c) 类似 matlab [a:c:b)

        angle_list = list()

        for k in range(uav_num):

            # 计算迭代区域的上下界
            x = round((track[k][lt][2]+T.edge)/scale)
            y = round((track[k][lt][3]+T.edge)/scale)
            angle1 = math.atan2(track[k][lt][1], track[k][lt][0])

            angle_max = math.fmod(angle1 + math.pi - gamma/2, math.pi)
            angle_min = math.fmod(angle1 + gamma, math.pi)
            if angle_min < angle_max:
                angle_list = [angle_min + j/25*(angle_max - angle_min) for j in range(26)]
            else:
                angle_list = [angle_max + j/25*(angle_min + 2*math.pi - angle_max) for j in range(26)]

            for length in range(0, 21):  # 20/200 = 10%
                for angle in angle_list:
                    x_test = int(round(x + length*math.cos(angle)))
                    y_test = int(round(y + length*math.sin(angle)))
                    if x_test >= cell or y_test >= cell or x_test < 0 or y_test < 0:
                        break
                    if area[x_test][y_test] == 0:
                        area[x_test][y_test] = 1
        # print('>>> Round', loop, 'Total ', np.size(track[-1], 0), ' >>> step ', lt)

        cover_rate = np.sum(area)/cell/cell
        with open(pardir + '/scene_Folder' + txt_name, 'a') as c:
            c.write(str(lt) + ' ' + str(cover_rate) + '\n')
    # print('Finished!')

    return cover_rate
