import numpy as np
import os
import MAEnv.scenarios.TargetProfile as T
import math
import MAControl.Util.OnlineCoverRate as CR

def calculate_coverage(num, step, loop=0):

    curdir = os.path.dirname(__file__)
    pardir = os.path.dirname(os.path.dirname(curdir))

    txt_name = '/cover_rate-%s-%s-%s.txt' % (str(num), str(step), str(loop))
    open(pardir + '/cover_rate_Folder' + txt_name, 'w')

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
    level1 = 10
    level2 = 20
    level3 = 30
    level4 = 40

    for l in range(0, np.size(track[-1], 0), 5):

        new_last = list()
        angle_list = list()
        angle_list1 = list()
        angle_list2 = list()
        angle_list3 = list()
        angle_list4 = list()

        for k in range(num):

            new_last.append([])

            # 计算迭代区域的上下界
            x = round((track[k][l][2]+T.edge)/scale)
            y = round((track[k][l][3]+T.edge)/scale)
            v = [track[k][l][0], track[k][l][1]]
            angle1 = math.atan2(track[k][l][1], track[k][l][0])
            angle_max = angle1 + math.pi/3
            angle_min = angle1 - math.pi / 3
            for j1 in range(level1):
                if (angle_min+j1/5) <angle_max:
                    angle_list1.append(angle_min+j1/5)
                else:
                    angle_list1.append(angle_max)
                    break
            for j2 in range(level2):
                if (angle_min+j2/10) <angle_max:
                    angle_list2.append(angle_min+j2/10)
                else:
                    angle_list2.append(angle_max)
                    break
            for j3 in range(level3):
                if (angle_min+j3/15) <angle_max:
                    angle_list3.append(angle_min+j3/15)
                else:
                    angle_list3.append(angle_max)
                    break
            for j4 in range(level4):
                if (angle_min+j4/20) <angle_max:
                    angle_list4.append(angle_min+j4/20)
                else:
                    angle_list4.append(angle_max)
                    break
            for length in range(0,26):
                if length <= 5:
                    angle_list = angle_list1
                elif length <= 10:
                    angle_list = angle_list2
                elif length <= 17:
                    angle_list = angle_list3
                elif length <= 25:
                    angle_list = angle_list4
                for angle in angle_list:
                    x_test = int(round(x + length*math.cos(angle)))
                    y_test = int(round(y + length*math.sin(angle)))
                    if abs(x_test) >= 200 or abs(y_test) >= 200:
                        break
                    if area[x_test][y_test] == 0:
                        area[x_test][y_test] = 1
        # print('>>> Round', loop, 'Total ', np.size(track[-1], 0), ' >>> step ', l)

    cover_rate = np.sum(area)/cell/cell
        # with open(pardir + '/MAControl-视觉/cover_rate_Folder' + txt_name, 'a') as c:
        #     c.write(str(l) + ' ' + str(cover_rate) + '\n')
    print('Finished!')
    return cover_rate

def calculate_coverage_overlap(num, step, loop=0):

    curdir = os.path.dirname(__file__)
    pardir = os.path.dirname(os.path.dirname(curdir))

    txt_name = '/cover_rate-%s-%s-%s.txt' % (str(num), str(step), str(loop))
    open('/home/k/论文/MAControl-dqn/0%loss-0step/cover_rate_Folder' + txt_name, 'w')

    cell = 200                     # 区域划分精度
    track = list()
    last_cover = list()
    for i in range(num):
        track.append(np.loadtxt('/home/k/论文/MAControl-dqn/0%%loss-0step/track/agent_%d_track.txt' % i))
        last_cover.append([])

    area = np.zeros((cell, cell))
    area_width = T.edge*2          # 正方形区域实际边长
    scale = area_width/cell        # 离散度(比例尺)
    iter_range = 0.25              # 迭代区域大小
    level1 = 10
    level2 = 20
    level3 = 30
    level4 = 40

    for l in range(0, np.size(track[-1], 0), 5):

        new_last = list()
        angle_list = list()
        angle_list1 = list()
        angle_list2 = list()
        angle_list3 = list()
        angle_list4 = list()

        for k in range(num):

            new_last.append([])

            # 计算迭代区域的上下界
            x = round((track[k][l][2]+T.edge)/scale)
            y = round((track[k][l][3]+T.edge)/scale)
            v = [track[k][l][0], track[k][l][1]]
            angle1 = math.atan2(track[k][l][1], track[k][l][0])
            angle_max = angle1 + math.pi/3
            angle_min = angle1 - math.pi / 3
            for j1 in range(level1):
                if (angle_min+j1/5) <angle_max:
                    angle_list1.append(angle_min+j1/5)
                else:
                    angle_list1.append(angle_max)
                    break
            for j2 in range(level2):
                if (angle_min+j2/10) <angle_max:
                    angle_list2.append(angle_min+j2/10)
                else:
                    angle_list2.append(angle_max)
                    break
            for j3 in range(level3):
                if (angle_min+j3/15) <angle_max:
                    angle_list3.append(angle_min+j3/15)
                else:
                    angle_list3.append(angle_max)
                    break
            for j4 in range(level4):
                if (angle_min+j4/20) <angle_max:
                    angle_list4.append(angle_min+j4/20)
                else:
                    angle_list4.append(angle_max)
                    break
            for length in range(0,26):
                if length <= 5:
                    angle_list = angle_list1
                elif length <= 10:
                    angle_list = angle_list2
                elif length <= 17:
                    angle_list = angle_list3
                elif length <= 25:
                    angle_list = angle_list4
                for angle in angle_list:
                    x_test = int(round(x + length*math.cos(angle)))
                    y_test = int(round(y + length*math.sin(angle)))
                    if abs(x_test) >= 200 or abs(y_test) >= 200:
                        break
                    if area[x_test][y_test] == 0:
                        area[x_test][y_test] = 1
                        new_last[k].append([x_test, y_test])
                    elif area[x_test][y_test] > 0 and [x_test, y_test] in last_cover[k]:
                        new_last[k].append([x_test, y_test])
                    elif area[x_test][y_test] > 0 and [x_test, y_test] not in last_cover[k]:
                        area[x_test][y_test] += 1
                        new_last[k].append([x_test, y_test])
                    else:
                        raise Exception('Unexpected situation!!!')
        last_cover[:] = new_last[:]
        print('>>> Round', loop, 'Total ', np.size(track[-1], 0), ' >>> step ', l)

        cover_rate, overlap_rate = CR.cal_cover_rate(area)
        with open('/home/k/论文/MAControl-dqn/0%loss-0step/cover_rate_Folder' + txt_name, 'a') as c:
            c.write(str(l) + ' ' + str(cover_rate) + ' ' + str(overlap_rate) + '\n')

    print('Finished!')


if __name__ == '__main__':

    curdir_ = os.path.dirname(__file__)
    pardir_ = os.path.dirname(os.path.dirname(curdir_))
    para = np.loadtxt('/home/k/论文/MAControl-dqn/0%loss-0step/track/para.txt')
    num_ = int(para[1])
    step_ = int(para[2])

    calculate_coverage_overlap(num_, step_)