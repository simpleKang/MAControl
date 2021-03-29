import numpy as np
import scipy.io
import os

# data = scipy.io.loadmat(r'Data_swarm.mat')
# print(data)
curdir = os.path.dirname(__file__)
pardir = os.path.dirname(os.path.dirname(curdir))

para = np.loadtxt(pardir + '/track/para.txt')
num = int(para[0])
case = 4
track = []
V_matrix = []
Pos_matrix = []

for i in range(num):
    track.append(np.loadtxt(pardir + '/case/case%d/uav_%d_track.txt' % (case, i)))
for index_uav in range(num):
    V_matrix.append([])
    Pos_matrix.append([])
    for i in range(len(track[0])-1):
        V_matrix[-1].append(track[index_uav][i][0:2])
        Pos_matrix[-1].append(track[index_uav][i][2:4])

scipy.io.savemat('Data_swarm_case%d.mat' % case, mdict={'V_matrix': V_matrix, 'Pos_matrix': Pos_matrix})