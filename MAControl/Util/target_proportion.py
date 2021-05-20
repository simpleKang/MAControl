import numpy as np
import os
import MAEnv.scenarios.TargetProfile as T
import math


def target_distribute(uav_num, step, loop, scene):

    curdir = os.path.dirname(__file__)
    pardir = os.path.dirname(os.path.dirname(curdir))

    txt_name = '/target-info-%s-%s-%s-%s.txt' % (str(uav_num), str(step), str(loop), scene)
    open(pardir + '/scene_Folder' + txt_name, 'w')

    perception = list()
    for i in range(uav_num):
        perception.append(np.loadtxt(pardir + '/track/uav_%d_perception.txt' % i))

    assignment = np.loadtxt(pardir + '/track/target_lock.txt')

    p2_array = [[] for i in range(uav_num)]
    p4_array = [[] for i in range(uav_num)]
    target_array = [[] for i in range(T.num_targets)]

    for lt in range(0, np.size(perception[-1], 0), 5):  # np.size(A,0) 返回该二维矩阵的行数 # range(a,b,c) 类似 matlab [a:c:b)

        for k in range(uav_num):
            p2_array[k].append(1-perception[k][lt][1])
            p4_array[k].append(perception[k][lt][3])
            
        p2_sum = sum([sum(item) for item in p2_array]) / len(p2_array)
        p4_sum = sum([sum(item) for item in p4_array]) / len(p4_array)
        with open(pardir + '/track' + txt_name, 'a') as c:
            c.write(str(lt) + ' ' + str(p2_sum) + ' ' + str(p4_sum) + '\n')

        return p2_sum + p4_sum
