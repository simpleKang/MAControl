import numpy as np
import os
import MAEnv.scenarios.TargetProfile as T
import math
from collections import Counter


def target_distribute(uav_num, step, loop, scene):

    curdir = os.path.dirname(__file__)
    pardir = os.path.dirname(os.path.dirname(curdir))
    score = 0

    txt_name = '/target-info-%s-%s-%s-%s.txt' % (str(uav_num), str(step), str(loop), scene)
    open(pardir + '/scene_Folder' + txt_name, 'w')

    perception = list()
    for i in range(uav_num):
        perception.append(np.loadtxt(pardir + '/track/uav_%d_perception.txt' % i))
    assignment = np.loadtxt(pardir + '/track/target_lock.txt')

    p2_array = [[] for i in range(uav_num)]
    p4_array = [[] for i in range(uav_num)]
    target_array = []

    for lt in range(0, np.size(perception[-1], 0), 5):  # np.size(A,0) 返回该二维矩阵的行数 # range(a,b,c) 类似 matlab [a:c:b)

        for k in range(uav_num):
            p2_array[k].append(1-perception[k][lt][1])
            p4_array[k].append(perception[k][lt][3])

        w_array = list()
        c = Counter(assignment[lt][1:])
        for m in range(T.num_targets):
            # ## # ↓↓ credit: KSB ↓↓ # ## #
            a_m = -1 * abs(c[m] - math.floor(uav_num/T.num_targets))
            w_m = math.exp(a_m) / T.num_targets
            w_array.append(w_m)
        s = sum([-1*item*math.log(item) for item in w_array])
        r = math.floor(uav_num/T.num_targets)/uav_num
        thr = (s*s)/(s*s+r*r)
        target_array.append(thr)

        if scene == 'B':
            p2_sum = sum([sum(item) for item in p2_array]) / uav_num / len(p2_array[0])
            p4_sum = sum([sum(item) for item in p4_array]) / uav_num / len(p4_array[0])
            with open(pardir + '/track' + txt_name, 'a') as c:
                c.write(str(lt) + ' ' + str(p2_sum) + ' ' + str(p4_sum) + '\n')
            score = p2_sum + p4_sum

        elif scene == 'C':
            score = sum(target_array) / len(target_array)
        else:
            pass

    return score
