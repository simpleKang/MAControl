import numpy as np
import os
import MAEnv.scenarios.TargetProfile as T
import math


def target_distribute(uav_num, step, loop=0):

    curdir = os.path.dirname(__file__)
    pardir = os.path.dirname(os.path.dirname(curdir))

    txt_name = '/perception-%s-%s-%s.txt' % (str(uav_num), str(step), str(loop))
    open(pardir + '/track' + txt_name, 'w')

    perception = list()
    for i in range(uav_num):
        perception.append(np.loadtxt(pardir + '/track/uav_%d_perception.txt' % i))

    for lt in range(0, np.size(perception[-1], 0), 5):  # np.size(A,0) 返回该二维矩阵的行数 # range(a,b,c) 类似 matlab [a:c:b)

        p2_array = list()
        p4_array = list()

        for k in range(uav_num):
            p2_array.append(1-perception[k][lt][1])
            p4_array.append(perception[k][lt][3])
            
        p2_sum = np.sum(p2_array)
        p4_sum = np.sum(p4_array)
        with open(pardir + '/track' + txt_name, 'a') as c:
            c.write(str(lt) + ' ' + str(p2_sum) + ' ' + str(p4_sum) + '\n')

