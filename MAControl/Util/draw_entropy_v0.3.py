#
import numpy as np
import os
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import math

plt.rcParams['figure.dpi'] = 200

curdir = os.path.dirname(__file__)
pardir = os.path.dirname(os.path.dirname(os.path.dirname(curdir)))


def entropy(case, number):
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
        print(i)
        print(A_i)
        print(variable_1)

    return indicator_1

    # # 区分目标类型
    # for i in range(len(track[0])-1):
    #     index = [0]*5
    #     index1 = [0] * num
    #     angle_list = []
    #     variable_2 = 00
    #     A_i = [0]*5
    #     S_uav = 0
    #     for index_uav in range(num):
    #         variable_0 = track[index_uav][i][4]
    #         A_i[variable_0-1] =  A_i[variable_0-1] + 1
    #     for j in range(len(A_i)):
    #         if A_i[j] == 0:
    #             S_uav = S_uav - 0
    #         else:
    #             S_uav = S_uav - variable_list0[j]/num *math.log(variable_list0[j]/num)
    #     for j in range(len(variable_list0)):
    #         if variable_list0[j] == 0:
    #             variable_1 = variable_1 + 0
    #         else:
    #             variable_1 = variable_1 + S_uav**2/(S_uav**2 + (variable_list0[j]/num)**2)
    #     variable_2 = variable_2 + variable_1
    #     indicator_1.append(variable_2/5)

    # return indicator_1

fig = plt.figure(facecolor='w')
case = 6
number = 5
indicator_1 = entropy(case, number)

x = range(0, len(indicator_1))

# fig.add_subplot(2,2,1)
plt.plot(x[:], indicator_1[:], 'r-', label='no', color='black')
# plt.plot(range(x[-200], x[-150]), [0.95] * 50, 'r-', label='no', color='black')
# plt.annotate('velocity', xy=(x[-150], 0.95*1), xytext=(x[-150], 0.95*1))
plt.title('case %d_%d' % (case, number))
plt.xlabel('Step')
plt.ylabel('S')
plt.savefig(pardir + '/case/case%d_%d/case%d_v0.3_1.png' % (case, number, case))
plt.savefig(pardir + '/case/picture/case%d_%d_v0.3_1.png' % (case, number))
plt.show()