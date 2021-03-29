#
import numpy as np
import numpy.fft as nf
import os
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import math
import scipy.signal

plt.rcParams['figure.dpi'] = 200

curdir = os.path.dirname(__file__)
pardir = os.path.dirname(os.path.dirname(os.path.dirname(curdir)))


# 傅里叶变化-v-method2
def entropy_fft(case, number):
    case = case
    number = number

    para = np.loadtxt(pardir + '/case/case%d_%d/para.txt' % (case, number))
    num = int(para[0])

    track = []
    for i in range(num):
        track.append(np.loadtxt(pardir + '/case/case%d_%d/uav_%d_track.txt' % (case, number, i)))

    # for k in range(-10, 1501, 25):

    color = ['black', 'darkorange', 'forestgreen', 'slategrey', 'lightcoral', 'gold', 'mediumturquoise',
             'darkviolet',
             'gray', 'burlywood', 'limegreen', 'cornflowerblue', 'firebrick', 'khaki', 'teal', 'plum',
             'silver', 'darkgoldenrod', 'lime', 'slateblue', 'red', 'yellow', 'cyan', 'purple',
             'lightgrey', 'gold', 'turquoise', 'blueviolet', 'darksalmon', 'darkseagreen', 'deepskyblue', 'hotpink']

    indicator_1 = []
    indicator_2 = []
    indicator_3 = []
    indicator_4 = []

    for i in range(len(track[0]) - 1):
        index = [0] * 5
        index1 = [0] * num
        index1_methon2 = [0] * 8
        index1_methon2_score = [0] * 1000
        index2 = list
        index3 = [0] * num
        index3_method2_score = [[0] * 20 for row in range(20)]
        index4 = [0] * num
        angle_list = []

        # 速度归一化并求出均值
        # for index_uav in range(num):
        #     variable_0 = track[index_uav][i][0]
        #     variable_1 = track[index_uav][i][1]
        #     variable_2 = np.sqrt(variable_0**2 + variable_1**2)
        #     track[index_uav][i][0] = variable_0/variable_2
        #     track[index_uav][i][1] = variable_1/variable_2
        variable_list0 = []
        variable_list1 = []
        variable_list2 = []
        variable_list3 = []
        for index_uav in range(num):
            variable_list2.append(track[index_uav][i][2])
            variable_list3.append(track[index_uav][i][2])

        # average_v_0 = np.mean(track[:][i][0])
        # average_v_1 = np.mean(track[:][i][1])
        average_p_0 = np.mean(variable_list2)
        average_p_1 = np.mean(variable_list3)

        # 求角度均值
        for index_uav in range(num):
            v_0 = track[index_uav][i][0]
            v_1 = track[index_uav][i][1]
            if v_0 == 0:
                v_0 = 0.00000000000000000000001
            angle = math.atan(v_1 / v_0)
            if track[index_uav][i][0] < 0:
                angle = angle + math.pi
            angle_list.append(angle)
        average_angle = np.mean(angle_list)

        variable_0 = 0
        variable_1 = 0
        variable_2 = 0
        variable_3 = 0
        variable_angle = 0
        for index_uav in range(num):
            variable_angle = variable_angle + (angle_list[index_uav] - average_angle) ** 2
            # variable_0 = variable_0 + track[index_uav][i][0] - average_v_0
            # variable_1 = variable_1 + track[index_uav][i][1] - average_v_1
            variable_2 = variable_2 + np.sqrt(
                (track[index_uav][i][2] - average_p_0) ** 2 + (track[index_uav][i][3] - average_p_1) ** 2)
            variable_3 = variable_3

        for j in range(num):
            v_0 = track[j][i][0]
            v_1 = track[j][i][1]
            pos_0 = track[j][i][2]
            pos_1 = track[j][i][3]

            # 指标一
            if v_0 == 0:
                v_0 = 0.00000000000000000000001
            angle = math.atan(v_1 / v_0)
            if track[j][i][0] < 0:
                angle = angle + math.pi
            p_angle = (angle - average_angle) ** 2 / variable_angle
            index1[j] = p_angle * (-math.log(p_angle))

            # 指标一(分速度方向）
            angle = math.atan(v_1 / v_0)
            if -math.pi / 8 <= angle < math.pi / 8:
                if v_0 > 0:
                    index1_methon2[0] = index1_methon2[0] + 1
                else:
                    index1_methon2[4] = index1_methon2[4] + 1
            if math.pi / 8 <= angle < 3 * math.pi / 8:
                if v_0 > 0:
                    index1_methon2[1] = index1_methon2[1] + 1
                else:
                    index1_methon2[5] = index1_methon2[5] + 1
            if -3 * math.pi / 8 <= angle < -math.pi / 8:
                if v_0 > 0:
                    index1_methon2[2] = index1_methon2[2] + 1
                else:
                    index1_methon2[6] = index1_methon2[6] + 1
            if (-3 * math.pi / 8 > angle) or (3 * math.pi / 8 <= angle):
                if v_0 > 0:
                    index1_methon2[3] = index1_methon2[3] + 1
                else:
                    index1_methon2[7] = index1_methon2[7] + 1

            # 指标三
            # p_2 = (np.sqrt(pos_0**2+pos_1**2) - np.sqrt(average_p_0**2 + average_p_1**2))**2/variable_2
            p_2 = np.sqrt((pos_0 - average_p_0) ** 2 + (pos_1 - average_p_1) ** 2) / variable_2
            # p_3 = /variable_3
            index3[j] = p_2 * (-math.log(p_2))
            # index4[j] = p_3*(-math.log(p_3))

            # 指标三 （画格子）
            if abs(pos_0) <= 2 and abs(pos_1) <= 2:
                index3_method2_score[int(pos_0 / 0.2) + 10][int(pos_1 / 0.2) + 10] = \
                    index3_method2_score[int(pos_0 / 0.2) + 10][int(pos_1 / 0.2) + 10] + 1

        # 指标一(分速度方向）
        for k in range(8):
            if index1_methon2[k] == 0:
                index1_methon2[k] = 0.1
            # index1_methon2_score[j] = index1_methon2_score[j] + (
            #             -(index1_methon2[k] / num) * math.log(index1_methon2[k] / num))
            index[1] = index[1] + (
                    -(index1_methon2[k] / num) * math.log(index1_methon2[k] / num))

        # with open(pardir + '/case/case%d_%d/case%d_v1.0_v_method2_log.txt' % (case, number, case), 'a') as f:
        #     f.write(str() + ' ' + str(index1_methon2_score) + ' ' + str(index1_methon2) + '\n')

        # 指标三（画格子）
        for l in range(20):
            for m in range(20):
                if index3_method2_score[l][m] == 0:
                    index3_method2_score[l][m] = 0.00001
                index[3] = index[3] + (
                            -(index3_method2_score[l][m] / num) * math.log(index3_method2_score[l][m] / num))

        for k in range(num):
            index[0] = index[0] + index1[k]
            # index[1] = index[1] + index1_methon2_score[k]
            index[2] = index[2] + index3[k]
            # index[3] = index[3] + index3_method2_score
            # index[3] = index[3] + index4[k]
        indicator_1.append(index[0])
        indicator_2.append(index[1])
        indicator_3.append(index[2])
        indicator_4.append(index[3])

    f, z, zxx = scipy.signal.stft(indicator_2, fs = 12.75, window = 'hann', nperseg = 256, nfft = None, return_onesided=False )
    ferqs = nf.fftfreq(len(indicator_2), 0.07843)
    complex_array = nf.fft(indicator_2)


    return indicator_1, indicator_2, indicator_3, indicator_4, f, z, zxx


def entropy(case, number):
    case = case
    number = number

    para = np.loadtxt(pardir + '/case/case%d_%d/para.txt' % (case, number))
    num = int(para[0])

    track = []
    for i in range(num):
        track.append(np.loadtxt(pardir + '/case/case%d_%d/uav_%d_track.txt' % (case, number, i)))

    # for k in range(-10, 1501, 25):

    color = ['black', 'darkorange', 'forestgreen', 'slategrey', 'lightcoral', 'gold', 'mediumturquoise',
             'darkviolet',
             'gray', 'burlywood', 'limegreen', 'cornflowerblue', 'firebrick', 'khaki', 'teal', 'plum',
             'silver', 'darkgoldenrod', 'lime', 'slateblue', 'red', 'yellow', 'cyan', 'purple',
             'lightgrey', 'gold', 'turquoise', 'blueviolet', 'darksalmon', 'darkseagreen', 'deepskyblue', 'hotpink']

    indicator_1 = []
    indicator_2 = []
    indicator_3 = []
    indicator_4 = []

    for i in range(len(track[0]) - 1):
        index = [0] * 5
        index1 = [0] * num
        index1_methon2 = [0] * 8
        index1_methon2_score = [0] * 1000
        index2 = list
        index3 = [0] * num
        index3_method2_score = [[0] * 20 for row in range(20)]
        index4 = [0] * num
        angle_list = []

        # 速度归一化并求出均值
        # for index_uav in range(num):
        #     variable_0 = track[index_uav][i][0]
        #     variable_1 = track[index_uav][i][1]
        #     variable_2 = np.sqrt(variable_0**2 + variable_1**2)
        #     track[index_uav][i][0] = variable_0/variable_2
        #     track[index_uav][i][1] = variable_1/variable_2
        variable_list0 = []
        variable_list1 = []
        variable_list2 = []
        variable_list3 = []
        for index_uav in range(num):
            variable_list2.append(track[index_uav][i][2])
            variable_list3.append(track[index_uav][i][2])

        # average_v_0 = np.mean(track[:][i][0])
        # average_v_1 = np.mean(track[:][i][1])
        average_p_0 = np.mean(variable_list2)
        average_p_1 = np.mean(variable_list3)

        # 求角度均值
        for index_uav in range(num):
            v_0 = track[index_uav][i][0]
            v_1 = track[index_uav][i][1]
            if v_0 == 0:
                v_0 = 0.00000000000000000000001
            angle = math.atan(v_1 / v_0)
            if track[index_uav][i][0] < 0:
                angle = angle + math.pi
            angle_list.append(angle)
        average_angle = np.mean(angle_list)

        variable_0 = 0
        variable_1 = 0
        variable_2 = 0
        variable_3 = 0
        variable_angle = 0
        for index_uav in range(num):
            variable_angle = variable_angle + (angle_list[index_uav] - average_angle) ** 2
            # variable_0 = variable_0 + track[index_uav][i][0] - average_v_0
            # variable_1 = variable_1 + track[index_uav][i][1] - average_v_1
            variable_2 = variable_2 + np.sqrt(
                (track[index_uav][i][2] - average_p_0) ** 2 + (track[index_uav][i][3] - average_p_1) ** 2)
            variable_3 = variable_3

        for j in range(num):
            v_0 = track[j][i][0]
            v_1 = track[j][i][1]
            pos_0 = track[j][i][2]
            pos_1 = track[j][i][3]

            # 指标一
            if v_0 == 0:
                v_0 = 0.00000000000000000000001
            angle = math.atan(v_1 / v_0)
            if track[j][i][0] < 0:
                angle = angle + math.pi
            p_angle = (angle - average_angle) ** 2 / variable_angle
            index1[j] = p_angle * (-math.log(p_angle))

            # 指标一(分速度方向）
            angle = math.atan(v_1 / v_0)
            if -math.pi / 8 <= angle < math.pi / 8:
                if v_0 > 0:
                    index1_methon2[0] = index1_methon2[0] + 1
                else:
                    index1_methon2[4] = index1_methon2[4] + 1
            if math.pi / 8 <= angle < 3 * math.pi / 8:
                if v_0 > 0:
                    index1_methon2[1] = index1_methon2[1] + 1
                else:
                    index1_methon2[5] = index1_methon2[5] + 1
            if -3 * math.pi / 8 <= angle < -math.pi / 8:
                if v_0 > 0:
                    index1_methon2[2] = index1_methon2[2] + 1
                else:
                    index1_methon2[6] = index1_methon2[6] + 1
            if (-3 * math.pi / 8 > angle) or (3 * math.pi / 8 <= angle):
                if v_0 > 0:
                    index1_methon2[3] = index1_methon2[3] + 1
                else:
                    index1_methon2[7] = index1_methon2[7] + 1

            # 指标三
            # p_2 = (np.sqrt(pos_0**2+pos_1**2) - np.sqrt(average_p_0**2 + average_p_1**2))**2/variable_2
            p_2 = np.sqrt((pos_0 - average_p_0) ** 2 + (pos_1 - average_p_1) ** 2) / variable_2
            # p_3 = /variable_3
            index3[j] = p_2 * (-math.log(p_2))
            # index4[j] = p_3*(-math.log(p_3))

            # 指标三 （画格子）
            if abs(pos_0) <= 2 and abs(pos_1) <= 2:
                index3_method2_score[int(pos_0 / 0.2) + 10][int(pos_1 / 0.2) + 10] = \
                    index3_method2_score[int(pos_0 / 0.2) + 10][int(pos_1 / 0.2) + 10] + 1

        # 指标一(分速度方向）
        for k in range(8):
            if index1_methon2[k] == 0:
                index1_methon2[k] = 0.1
            # index1_methon2_score[j] = index1_methon2_score[j] + (
            #             -(index1_methon2[k] / num) * math.log(index1_methon2[k] / num))
            index[1] = index[1] + (
                    -(index1_methon2[k] / num) * math.log(index1_methon2[k] / num))

        # with open(pardir + '/case/case%d_%d/case%d_v1.0_v_method2_log.txt' % (case, number, case), 'a') as f:
        #     f.write(str() + ' ' + str(index1_methon2_score) + ' ' + str(index1_methon2) + '\n')

        # 指标三（画格子）
        for l in range(20):
            for m in range(20):
                if index3_method2_score[l][m] == 0:
                    index3_method2_score[l][m] = 0.00001
                index[3] = index[3] + (
                            -(index3_method2_score[l][m] / num) * math.log(index3_method2_score[l][m] / num))

        for k in range(num):
            index[0] = index[0] + index1[k]
            # index[1] = index[1] + index1_methon2_score[k]
            index[2] = index[2] + index3[k]
            # index[3] = index[3] + index3_method2_score
            # index[3] = index[3] + index4[k]
        indicator_1.append(index[0])
        indicator_2.append(index[1])
        indicator_3.append(index[2])
        indicator_4.append(index[3])
    return indicator_1, indicator_2, indicator_3, indicator_4


def entropy_v03(case, number):
    case = case
    number = number

    para = np.loadtxt(pardir + '/case/case%d_%d/para.txt' % (case, number))
    num = int(para[0])

    track = []
    for i in range(num):
        track.append(np.loadtxt(pardir + '/case/case%d_%d/uav_%d_track.txt' % (case, number, i)))

    # for k in range(-10, 1501, 25):

    color = ['black', 'darkorange', 'forestgreen', 'slategrey', 'lightcoral', 'gold', 'mediumturquoise', 'darkviolet',
             'gray', 'burlywood', 'limegreen', 'cornflowerblue', 'firebrick', 'khaki', 'teal', 'plum',
             'silver', 'darkgoldenrod', 'lime', 'slateblue', 'red', 'yellow', 'cyan', 'purple',
             'lightgrey', 'gold', 'turquoise', 'blueviolet', 'darksalmon', 'darkseagreen', 'deepskyblue', 'hotpink']

    indicator_1 = []

    # 不区分目标类型
    for i in range(len(track[0])-1):
        index = [0]*5
        index1 = [0] * num
        angle_list = []
        variable_2 = 0

        A_i = 0
        S_uav = 0
        for index_uav in range(num):
            if track[index_uav][i][4] != 0:
                A_i = A_i + 1
        if A_i == 0 or A_i == num:
            variable_1 = 0

        else:
            S_uav = -A_i/num*math.log(A_i/num)-(num-A_i)/num*math.log((num-A_i)/num)
            variable_1 = (S_uav**2/(S_uav**2 + (A_i/num)**2) + S_uav**2/(S_uav**2 + ((num-A_i)/num)**2))/2
        indicator_1.append(variable_1)

    return indicator_1


# 速度与位置熵加权合并
def entropy_v04(case, number):
    case = case
    number = number
    para = np.loadtxt(pardir + '/case/case%d_%d/para.txt' % (case, number))
    num = int(para[0])
    indicator = []

    track = []
    for i in range(num):
        track.append(np.loadtxt(pardir + '/case/case%d_%d/uav_%d_track.txt' % (case, number, i)))

    indicator_v, indicator_v_method2, indicator_p, indicator_p_method2 = entropy(case, number)
    indicator_attack = [0]*len(indicator_v_method2)
    indicator_p_method2 = normal(indicator_p_method2)
    indicator_v_method2 = normal(indicator_v_method2)
    for i in range(len(track[0])-1):
        for index_uav in range(num):
            if track[index_uav][i][4] != 0:
                indicator_attack[i] = indicator_attack[i] + 1
    for i in range(len(indicator_attack)):
        indicator.append((num-indicator_attack[i])/num*indicator_v_method2[i]+\
                    (indicator_attack[i])/num*indicator_p_method2[i])
    return indicator_v, indicator_v_method2, indicator_p, indicator_p_method2, indicator


# 在v03的基础上区分攻击目标类型
# 分成3类
def entropy_v05(case, number):
    case = case
    number = number

    para = np.loadtxt(pardir + '/case/case%d_%d/para.txt' % (case, number))
    num = int(para[0])

    track = []
    for i in range(num):
        track.append(np.loadtxt(pardir + '/case/case%d_%d/uav_%d_track.txt' % (case, number, i)))

    # for k in range(-10, 1501, 25):

    color = ['black', 'darkorange', 'forestgreen', 'slategrey', 'lightcoral', 'gold', 'mediumturquoise', 'darkviolet',
             'gray', 'burlywood', 'limegreen', 'cornflowerblue', 'firebrick', 'khaki', 'teal', 'plum',
             'silver', 'darkgoldenrod', 'lime', 'slateblue', 'red', 'yellow', 'cyan', 'purple',
             'lightgrey', 'gold', 'turquoise', 'blueviolet', 'darksalmon', 'darkseagreen', 'deepskyblue', 'hotpink']

    indicator_1 = []

    # 区分目标类型
    # 目标编号1、3、5是一种类型；编号2、4是一种
    for i in range(len(track[0])-1):
        A_i = [0]*3
        S_uav = 0
        variable_1 = 0
        for index_uav in range(num):
            if track[index_uav][i][4] == 0:
                A_i[0] = A_i[0] + 1
            elif track[index_uav][i][4] == 2 or track[index_uav][i][4] == 4:
                A_i[1] = A_i[1] + 1
            else:
                A_i[2] = A_i[2] + 1

        for j in range(len(A_i)):
            if A_i[j] == 0 or A_i[j] == num:
                S_uav = S_uav-0
            else:
                S_uav = S_uav-A_i[j] / num * math.log(A_i[j] / num)

        for j in range(len(A_i)):
            if S_uav == 0:
                variable_1 = 0
            else:
                variable_1 = variable_1 + (S_uav ** 2 / (S_uav ** 2 + (A_i[j] / num) ** 2))
        indicator_1.append(variable_1/3)
        # print(i)
        # print(A_i)

    return indicator_1


# 在v03的基础上区分攻击目标类型
# 分成6类
def entropy_v06(case, number):
    case = case
    number = number

    para = np.loadtxt(pardir + '/case/case%d_%d/para.txt' % (case, number))
    num = int(para[0])

    track = []
    for i in range(num):
        track.append(np.loadtxt(pardir + '/case/case%d_%d/uav_%d_track.txt' % (case, number, i)))

    # for k in range(-10, 1501, 25):

    color = ['black', 'darkorange', 'forestgreen', 'slategrey', 'lightcoral', 'gold', 'mediumturquoise', 'darkviolet',
             'gray', 'burlywood', 'limegreen', 'cornflowerblue', 'firebrick', 'khaki', 'teal', 'plum',
             'silver', 'darkgoldenrod', 'lime', 'slateblue', 'red', 'yellow', 'cyan', 'purple',
             'lightgrey', 'gold', 'turquoise', 'blueviolet', 'darksalmon', 'darkseagreen', 'deepskyblue', 'hotpink']

    indicator_1 = []

    # 区分目标类型
    # 目标编号1、3、5是一种类型；编号2、4是一种
    for i in range(len(track[0])-1):
        A_i = [0]*6
        S_uav = 0
        variable_1 = 0
        for index_uav in range(num):
            if track[index_uav][i][4] == 0:
                A_i[0] = A_i[0] + 1
            elif track[index_uav][i][4] == 1:
                A_i[1] = A_i[1] + 1
            elif track[index_uav][i][4] == 2:
                A_i[2] = A_i[2] + 1
            elif track[index_uav][i][4] == 3:
                A_i[3] = A_i[3] + 1
            elif track[index_uav][i][4] == 4:
                A_i[4] = A_i[4] + 1
            else:
                A_i[5] = A_i[5] + 1

        for j in range(len(A_i)):
            if A_i[j] == 0 or A_i[j] == num:
                S_uav = S_uav-0
            else:
                S_uav = S_uav-A_i[j] / num * math.log(A_i[j] / num)

        for j in range(len(A_i)):
            if S_uav == 0:
                variable_1 = 0
            else:
                variable_1 = variable_1 + (S_uav ** 2 / (S_uav ** 2 + (A_i[j] / num) ** 2))
                print(variable_1)
        indicator_1.append(variable_1/6)
        # print(S_uav)
        # print(variable_1/6)
        # print(i)
        # print(A_i)

    return indicator_1


# 区分目标分配情况
# 需要用到目标分配情况
def entropy_v07(case, number):
    case = case
    number = number

    para = np.loadtxt(pardir + '/case/case%d_%d/para.txt' % (case, number))
    num = int(para[0])

    track = []
    for i in range(num):
        track.append(np.loadtxt(pardir + '/case/case%d_%d/uav_%d_track.txt' % (case, number, i)))

    # for k in range(-10, 1501, 25):

    color = ['black', 'darkorange', 'forestgreen', 'slategrey', 'lightcoral', 'gold', 'mediumturquoise', 'darkviolet',
             'gray', 'burlywood', 'limegreen', 'cornflowerblue', 'firebrick', 'khaki', 'teal', 'plum',
             'silver', 'darkgoldenrod', 'lime', 'slateblue', 'red', 'yellow', 'cyan', 'purple',
             'lightgrey', 'gold', 'turquoise', 'blueviolet', 'darksalmon', 'darkseagreen', 'deepskyblue', 'hotpink']

    indicator_1 = []

    # 各目标打击所需数量
    expect_att = [4, 4, 4, 4, 4]

    open(pardir + '/case/case%d_%d/attack_list.txt' % (case, number), 'w')

    # 区分目标类型
    for i in range(len(track[0]) - 1):
        A_i = [0] * 6
        S_uav = 0
        variable_2 = 0
        for index_uav in range(num):
            if track[index_uav][i][4] == 0:
                A_i[0] = A_i[0] + 1
            elif track[index_uav][i][4] == 1:
                A_i[1] = A_i[1] + 1
            elif track[index_uav][i][4] == 2:
                A_i[2] = A_i[2] + 1
            elif track[index_uav][i][4] == 3:
                A_i[3] = A_i[3] + 1
            elif track[index_uav][i][4] == 4:
                A_i[4] = A_i[4] + 1
            else:
                A_i[5] = A_i[5] + 1

        # 计算S
        for j in range(len(expect_att)):
            if A_i[j+1] == expect_att[j]:
                variable_1 = 1/len(expect_att)
            else:
                variable_1 = math.exp(-abs(A_i[j+1]-expect_att[j]))/len(expect_att)
            print(variable_1)
            S_uav = S_uav - variable_1*math.log(variable_1)
        for j in range(len(expect_att)):
            variable_2 = variable_2 + S_uav**2/(S_uav**2 + (expect_att[j]/num)**2)

        with open(pardir + '/case/case%d_%d/attack_list.txt' % (case, number), 'a') as f:
            f.write(str(i) + ' ' + str(variable_2/5) + ' ' + str(A_i[0]) + ' ' + str(A_i[1]) + ' ' + str(A_i[2]) + ' ' + str(A_i[3]) + ' ' +
                    str(A_i[4]) + ' ' + str(A_i[5]) + '\n')

        indicator_1.append(variable_2/5)
        # print(i)
        # print(A_i)
        # print(variable_2)
    indicator_1 = normal(indicator_1)
    print(indicator_1)
    return indicator_1


# 速度与位置熵通过阈值函数合并
def entropy_v08(case, number):
    case = case
    number = number
    para = np.loadtxt(pardir + '/case/case%d_%d/para.txt' % (case, number))
    num = int(para[0])
    indicator = []

    track = []
    for i in range(num):
        track.append(np.loadtxt(pardir + '/case/case%d_%d/uav_%d_track.txt' % (case, number, i)))

    indicator_v, indicator_v_method2, indicator_p, indicator_p_method2 = entropy(case, number)
    threshold = entropy_v07(case, number)
    threshold = normal(threshold)
    indicator_p_method2 = normal(indicator_p_method2)
    indicator_v_method2 = normal(indicator_v_method2)
    for i in range(len(indicator_v_method2)):
        indicator.append((1 - threshold[i]) * indicator_v_method2[i] +
                         (threshold[i]) * indicator_p_method2[i])
    return indicator_v, indicator_v_method2, indicator_p, indicator_p_method2, indicator, threshold


# 归一化
def normal(list1):
    a = max(list1)
    b = min(list1)
    for i in range(len(list1)):
        list1[i] = (list1[i] - b)/(a-b)
        # list1[i] = list1[i] / int(a)
    return list1


fig = plt.figure(facecolor='w')
case = 6
number = 5
indicator_v, indicator_v_method2, indicator_p, indicator_p_method2, indicator_v08, threshold = entropy_v08(case, number)
x = range(len(indicator_v08))
# indicator_v3 = entropy_v03(case,number)
# indicator_v5 = entropy_v05(case, number)
# indicator_v6 = entropy_v06(case, number)
# indicator_v7 = entropy_v07(case, number)
# in_1, in_2, in_3, in_4, f, z, zxx = entropy_fft(case, number)

# indicator_1, indicator_2, indicator_3, indicator_4 = entropy(case, number)
# indicator_5 = [x*2 for x in entropy_v03(case, number)]
# indicator_v = indicator_1
# indicator_v_method2 = [x/2 for x in indicator_2]
# indicator_p = indicator_3
# indicator_p_method2 = [x/3 for x in indicator_4]
# indicator_all = []

# x_1 = max(len(indicator_v), len(indicator_v_method2), len(indicator_p), len(indicator_p_method2))
# x = range(0, x_1)

# 子图
# fig.add_subplot(2,2,1)
# plt.plot(x[:], indicator_v_method2[:], 'r-', label='no', color='yellow')
# # plt.plot(range(x[-200], x[-150]), [0.95] * 50, 'r-', label='no', color='black')
# # plt.annotate('velocity', xy=(x[-150], 0.95*1), xytext=(x[-150], 0.95*1))
# plt.title("velocity")
# plt.xlabel('Step')
# plt.ylabel('S')
#
# fig.add_subplot(2,2,2)
# plt.plot(x[:], indicator_p_method2[:], 'r-', label='no', color='darkturquoise')
# # plt.plot(range(x[-200], x[-150]), [1.2] * 50, 'r-', label='no', color='forestgreen')
# # plt.annotate('velocity_2', xy=(x[-150], 1.2*1), xytext=(x[-150], 1.2*1))
# plt.title("position")
# plt.xlabel('Step')
# plt.ylabel('S')
#
# fig.add_subplot(2,2,3)
# plt.plot(x[:], indicator_v08[:], 'r-', label='no', color='black')
# # plt.plot(range(x[-200], x[-150]), [1.45] * 50, 'r-', label='no', color='slategrey')
# # plt.annotate('position', xy=(x[-150], 1.45*1), xytext=(x[-150], 1.45*1))
# plt.title("mix")
# plt.xlabel('Step')
# plt.ylabel('S')
#
# fig.add_subplot(2,2,4)
# plt.plot(x[:], threshold[:], 'r-', label='no', color='red')
# # plt.plot(range(x[-200], x[-150]), [1.7] * 50, 'r-', label='no', color='darkorange')
# # plt.annotate('position_2', xy=(x[-150], 1.7*1), xytext=(x[-150], 1.7*1))
# plt.title("threshold")
# plt.xlabel('Step')
# plt.ylabel('Value')
# fig.tight_layout()
# plt.suptitle('case %d_%d' % (case, number))
#
# fig.add_subplot(2,3,5)
# plt.plot(x[:], indicator_5[:], 'r-', label='no', color='darkorange')
# # plt.plot(range(x[-200], x[-150]), [1.7] * 50, 'r-', label='no', color='darkorange')
# # plt.annotate('position_2', xy=(x[-150], 1.7*1), xytext=(x[-150], 1.7*1))
# plt.title("v0.3")
# plt.xlabel('Step')
# plt.ylabel('signal')
# plt.savefig(pardir + '/case/case%d_%d/case%d_v0.4_2.png' % (case, number, case))
# plt.savefig(pardir + '/case/picture/case%d_%d_v0.4_2.png' % (case, number))
# plt.show()

# together
# x = range(len(indicator_v08))

# 归一化
# indicator_v04 = normal(indicator_v04)

# indicator_v3 = normal(indicator_v3)
# cm=plt.cm.get_cmap('flag')
# plt.pcolormesh(z, f, np.abs(zxx), cmap=cm)

# pows = np.abs(complex_array)
# plt.plot(freqs[freqs > 0], pows[freqs > 0], 'r-', label='no', color='black')
# plt.plot(range(x[-200], x[-150]), [0.95] * 50, 'r-', label='no', color='black')
# plt.annotate('velocity', xy=(x[-150], 0.95*1), xytext=(x[-150], 0.95*1))

# plt.plot(x[:], indicator_v04[:], 'r-', label='no', color='forestgreen', linewidth=3)
# plt.plot(range(x[-500], x[-300]), [0.1] * 200, 'r-', label='no', color='forestgreen')
# plt.annotate('mix', xy=(x[-300], 0.1*1), xytext=(x[-300], 0.1*1))
#
# plt.plot(x[:], indicator_v3[:], 'r-', label='no', color='black', linewidth=2)
# plt.plot(range(x[-500], x[-300]), [0.4] * 200, 'r-', label='no', color='black')
# plt.annotate('search_tar', xy=(x[-300], 0.4*1), xytext=(x[-300], 0.4*1))
#
plt.plot(x[:], indicator_p_method2[:], '-', label='no', color='darkturquoise', linewidth=1)
plt.plot(range(8500, 9000), [0.2] * 500, '-', label='no', color='darkturquoise')
plt.annotate('pos', xy=(9000, 0.2*1), xytext=(9000, 0.2*1))
#
plt.plot(x[:], indicator_v_method2[:], '-', label='no', color='yellow', linewidth=1)
plt.plot(range(8500, 9000), [0.1] * 500, 'r-', label='no', color='yellow')
plt.annotate('vel', xy=(9000, 0.1*1), xytext=(9000, 0.1*1))

plt.plot(x[:], threshold[:], 'r-', label='no', color='red', linewidth=2)
plt.plot(range(8500, 9000), [0.3] * 500, 'r-', label='no', color='red')
plt.annotate('threshold', xy=(9000, 0.3*1), xytext=(9000, 0.3*1))

# plt.plot(x[:], indicator_v6[:], 'r-', label='no', color='lime', linewidth=3)
# plt.plot(range(x[-500], x[-300]), [0.4] * 200, 'r-', label='no', color='lime')
# plt.annotate('5_target', xy=(x[-300], 0.4*1), xytext=(x[-300], 0.4*1))

plt.plot(x[:], indicator_v08[:], 'r-', label='no', color='black', linewidth=2)
plt.plot(range(8500, 9000), [0.4] * 500, 'r-', label='no', color='black')
plt.annotate('mix', xy=(9000, 0.4*1), xytext=(9000, 0.4*1))

# plt.title('case%d_%d_all target need 1 uav' % (case, number))
# plt.title('all target need 1 uav' )
plt.xlabel('Step')
plt.ylabel('Value')
plt.xlim(0, 11000)
# plt.ylabel('Frequency [Hz]')
# plt.xlabel('Time [sec]')
# plt.xlabel('Frequency')
# plt.ylabel('Power')
plt.savefig(pardir + '/case/case%d_%d/case%d_v0.4_v8_1.png' % (case, number, case))
plt.savefig(pardir + '/case/picture/case%d_%d_v0.4_v8_1.png' % (case, number))
plt.show()