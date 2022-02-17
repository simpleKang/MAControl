import os
import casadi as ca
import numpy as np
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel
import sys
import matplotlib.pyplot as plt

open(os.path.dirname(__file__) + 'track.txt', 'w')

T = 4.0
dt = 0.2
numObstacles = 6

# 给出障碍物距离计算公式， 此处先假定障碍物在（10,10,3）
obstaclePosition = ca.SX.zeros(3*numObstacles, 1)
obstacleR = 0.5
obstaclePosition[0] = 3
obstaclePosition[1] = -2
obstaclePosition[2] = obstacleR

obstaclePosition[3] = 3
obstaclePosition[4] = 3
obstaclePosition[5] = obstacleR

obstaclePosition[6] = 5
obstaclePosition[7] = 0
obstaclePosition[8] = obstacleR

obstaclePosition[9] = 5
obstaclePosition[10] = 2
obstaclePosition[11] = obstacleR

obstaclePosition[12] = 7
obstaclePosition[13] = -2
obstaclePosition[14] = obstacleR

obstaclePosition[15] = 7
obstaclePosition[16] = 3
obstaclePosition[17] = obstacleR

# obstaclePosition[18] = 5
# obstaclePosition[19] = 5
# obstaclePosition[20] = obstacleR
#
# obstaclePosition[21] = 5
# obstaclePosition[22] = -3
# obstaclePosition[23] = obstacleR

# 给出障碍物距离计算公式， 此处先假定障碍物在（10,10,3）
obstaclePositionList = []
obstaclePositionList.append(3)
obstaclePositionList.append(-2)
obstaclePositionList.append(obstacleR)

obstaclePositionList.append(3)
obstaclePositionList.append(3)
obstaclePositionList.append(obstacleR)

obstaclePositionList.append(5)
obstaclePositionList.append(0)
obstaclePositionList.append(obstacleR)

obstaclePositionList.append(5)
obstaclePositionList.append(2)
obstaclePositionList.append(obstacleR)

obstaclePositionList.append(7)
obstaclePositionList.append(-2)
obstaclePositionList.append(obstacleR)

obstaclePositionList.append(7)
obstaclePositionList.append(3)
obstaclePositionList.append(obstacleR)

# obstaclePositionList.append(5)
# obstaclePositionList.append(5)
# obstaclePositionList.append(obstacleR)
#
# obstaclePositionList.append(5)
# obstaclePositionList.append(-3)
# obstaclePositionList.append(obstacleR)

class NMPC:
    def __init__(self, num_agent, T, dt):
        self.model = AcadosModel()
        self.ocp = AcadosOcp()
        self.T = T
        self.dt = dt
        self.N = num_agent
        self.numObstacles = numObstacles
        self.max_neig = 3
        self.maxA = np.sqrt(4/3)
        self.communciationRange = 200
        self.collisionRadiusAgent = 0.1
        self.collisionRadiusObstacles = 0.2

        self.position = ca.SX.sym("position", 3 * self.N)
        self.velocity = ca.SX.sym("velocity", 3 * self.N)
        self.acceleration = ca.SX.sym("acceleration", 3 * self.N)
        self.x = ca.vertcat(self.position, self.velocity)
        self.xdot = ca.SX.sym("xdot", 6*self.N, 1)

        # 对应matlab中的compute_closest_neighbors.m____________________________________________________________
        # 邻域个体
        # self.neighborIndex = [[random.randint(0, 4)] * self.max_neig] * self.N
        self.neighborDist = ca.SX.sym("D", self.N, self.N)
        self.neighborIndex = ca.SX.zeros(self.max_neig, self.N)
        for i in range(self.N):
            for j in range(self.N):
                if i == j:
                    self.neighborDist[i, j] = 1e10
                elif j > i:
                    dist2 = self.calculateArrayLength([self.position[3 * i]-self.position[3 * j],
                                                       self.position[3 * i + 1]-self.position[3 * j + 1],
                                                       self.position[3 * i + 2]-self.position[3 * j + 2]])
                    self.neighborDist[i, j] = dist2
                    self.neighborDist[j, i] = dist2
        for i in range(self.N):
            toSort = ca.SX.ones(5, 1)
            dist2ToSort = self.neighborDist[:, i]
            for j in range(self.max_neig):
                minDist2 = ca.SX(1e9)
                minIndex = 0
                for k in range(self.N):
                    minIndex = ca.if_else(ca.logic_and(dist2ToSort[k] < minDist2, toSort[k]),
                                          k, minIndex)
                    minDist2 = ca.if_else(ca.logic_and(dist2ToSort[k] < minDist2, toSort[k]),
                                          dist2ToSort[k], minDist2)
                for l in range(self.N):
                    toSort[l] = ca.if_else(minIndex == l, 0, toSort[l])
                self.neighborIndex[j, i] = minIndex

        # 对应matlab中swarming_model_max_neig.m的损失函数部分____________________________________________________
        self.distReference = 0.8
        self.vReference = 0.5
        self.uReference = [1, 0, 0]
        self.sym_sep = ca.SX.zeros(self.N * self.max_neig, 1)
        self.sym_dir = ca.SX.zeros(self.N, 1)
        self.sym_nav = ca.SX.zeros(self.N, 1)
        for i in range(self.N):
            selfPosition = [self.position[3 * i], self.position[3 * i + 1], self.position[3 * i + 2]]
            selfVelocity = [self.velocity[3*i], self.velocity[3*i+1], self.velocity[3*i+2]]
            neighbors = self.neighborIndex[:, i]
            for j in range(self.max_neig):
                neighbor = neighbors[j]
                neighborIndex = [1, 2, 3] + 3 * neighbor*ca.SX.ones(3, 1)
                agent_idx = [0, 1, 2] + 3*i*np.ones((1, 3))
                neighborX = neighborIndex[0] - 1
                neighborY = neighborIndex[1] - 1
                neighborZ = neighborIndex[2] - 1
                tmp = ca.repmat(self.position[agent_idx[0]], self.N, 1)
                pos_rel_cell = ca.vertsplit(self.position - ca.repmat(self.position[agent_idx[0]], self.N, 1))
                pos_rel_default = [self.distReference, 0, 0]
                pos_rel = ca.SX.zeros(3, 1)
                pos_rel[0] = ca.conditional(neighborX, pos_rel_cell, pos_rel_default[0], False)
                pos_rel[1] = ca.conditional(neighborY, pos_rel_cell, pos_rel_default[1], False)
                pos_rel[2] = ca.conditional(neighborZ, pos_rel_cell, pos_rel_default[2], False)
                self.sym_sep[i*self.max_neig+j] = (pos_rel[0]*pos_rel[0]+pos_rel[1]*pos_rel[1]+pos_rel[2]*pos_rel[2]-self.distReference**2)/self.max_neig
            self.sym_dir[i] = 1-(self.uReference[0]*selfVelocity[0]+self.uReference[1]*selfVelocity[1]
                                 +self.uReference[2]*selfVelocity[2])**2/self.calculateArrayLength(selfVelocity)
            a = self.calculateArrayLength(selfVelocity)
            self.sym_nav[i] = self.calculateArrayLength(selfVelocity) - self.vReference**2

    # 定义计算模长函数
    def calculateArrayLength(self, array):
        length = 0
        for i in range(len(array)):
            length += array[i]**2
        return length

    def getVehicleStates(self, data):
        self.initStates = data

    def initModel(self, x0):
        # 对应matlab中cl_init.m中的参数设置
        self.model.name = "swarm"
        self.ocp.solver_options.tf = self.T
        self.ocp.dims.nx = 6*self.N
        self.ocp.dims.nu = 3*self.N
        # self.ocp.dims.ny = 40
        self.ocp.dims.ny = self.N*self.max_neig + self.N + self.N + 3*self.N
        self.ocp.dims.ny_e = self.N*self.max_neig + self.N + self.N
        self.ocp.dims.nh = int(3*self.N + self.N*(self.N-1)/2 + self.N*6)
        self.ocp.dims.nh_e = 0
        self.ocp.dims.N = int(self.T/self.dt)

        self.model.x = self.x
        self.model.u = self.acceleration

        self.model.xdot = self.xdot
        self.ocp.cost.cost_type = "NONLINEAR_LS"
        self.ocp.cost.cost_type_e = "NONLINEAR_LS"
        # self.ocp.model.cost_y_expr = ca.vertcat(self.sym_sep, self.sym_dir, self.sym_nav, self.acceleration)
        self.model.cost_y_expr = ca.vertcat(self.sym_sep, self.sym_dir, self.sym_nav, 4e-1*self.acceleration)
        self.model.cost_y_expr_e = ca.vertcat(self.sym_sep, self.sym_dir, self.sym_nav)

        w_spretion = 1
        w_direction = 5
        w_navigation = 5
        w_u = 0.4
        self.ocp.cost.W = np.diag(np.concatenate((w_spretion/self.max_neig * np.ones(self.max_neig * self.N),
                                             w_direction * np.ones(self.N),
                                             w_navigation * np.ones(self.N),
                                             w_u * w_spretion * np.ones(3 * self.N))))
        self.ocp.cost.W_e = np.diag(np.concatenate((w_spretion * np.ones(self.max_neig * self.N),
                                               w_direction * np.ones(self.N),
                                               w_navigation * np.ones(self.N))))
        self.ocp.cost.yref = np.zeros(self.max_neig*self.N+self.N+self.N+3*self.N)
        self.ocp.cost.yref_e = np.zeros(self.max_neig*self.N+self.N+self.N)

        self.model.f_expl_expr = ca.vertcat(self.velocity, self.acceleration)
        self.ocp.solver_options.integrator_type = "IRK"
        self.model.f_impl_expr = ca.vertcat(self.velocity, self.acceleration) - self.xdot

        # todo 转python

        self.ocp.constraints.x0 = x0
        # x0 = np.array([1] * 30)
        # self.ocp.constraints.x0 = np.array([0.2,
        #        1,
        #        -2.5,
        #        -0.5,
        #        0.2,
        #        -2.5,
        #        -0.1,
        #        -0.5,
        #        -2.5,
        #        0.2,
        #        0.2,
        #        -2.5,
        #        -0.5,
        #        1,
        #        -2.5,
        #         0,
        #         0,
        #         0,
        #         0,
        #         0,
        #         0,
        #         0,
        #         0,
        #         0,
        #         0,
        #         0,
        #         0,
        #         0,
        #         0,
        #         0])
        # 对应matlab中swarming_model_max_neig.m的约束部分____________________________________________________
        self.agentDist = ca.SX.zeros(int(self.N * (self.N - 1) / 2), 1)
        # 给出个体间距计算方法
        index = 0
        for i in range(self.N):
            selfPosition = [self.position[3 * i], self.position[3 * i + 1], self.position[3 * i + 2]]
            for j in range(i + 1, self.N, 1):
                otherPosition = [self.position[3 * j], self.position[3 * j + 1], self.position[3 * j + 2]]
                self.agentDist[index] = self.calculateArrayLength([selfPosition[0] - otherPosition[0],selfPosition[1] -
                                                                   otherPosition[1],selfPosition[2] - otherPosition[2]])
                index = index + 1
        # 距离障碍物最近距离
        self.minDistObstacleAgent = ca.SX.zeros(self.N*self.numObstacles, 1)

        for i in range(self.N):
            for j in range(self.numObstacles):
                selfPosition = [self.position[3 * i], self.position[3 * i + 1], self.position[3 * i + 2]]
                dist = [selfPosition[0] - obstaclePosition[j*3+0], selfPosition[1] - obstaclePosition[j*3+1]]
                # selfPosition = np.array([self.position[3 * i], self.position[3 * i + 1], self.position[3 * i + 2]])
                selfVelocity = [self.velocity[3 * i], self.velocity[3 * i + 1], self.velocity[3 * i + 2]]
                self.minDistObstacleAgent[self.numObstacles*i+j] = (dist[0] ** 2 + dist[1] ** 2 - obstaclePosition[j*3+2]**2)
        self.model.con_h_expr = ca.vertcat(self.acceleration, self.agentDist, self.minDistObstacleAgent)
        #---------------------------------------------------------------------------------------------------------

        lhU = -self.maxA * np.ones(3*self.N)
        uhU = self.maxA * np.ones(3*self.N)
        lhAgentColl = 4 * self.collisionRadiusAgent * self.collisionRadiusAgent * np.ones(
            int(self.N * (self.N - 1) / 2))
        uhAgentColl = 300 * 300 * np.ones(int(self.N * (self.N - 1) / 2))
        lhObstacleColl = np.array([self.collisionRadiusObstacles * self.collisionRadiusObstacles] * self.N*self.numObstacles)
        uhObstacleColl = np.array([1e16] * self.N*self.numObstacles)
        a = np.concatenate((lhU, lhAgentColl, lhObstacleColl))
        self.ocp.constraints.lh = np.concatenate((lhU, lhAgentColl, lhObstacleColl))
        self.ocp.constraints.uh = np.concatenate((uhU, uhAgentColl, uhObstacleColl))
        # todo 约束问题，与matlab不同
        self.ocp.solver_options.nlp_solver_type = 'SQP'
        # self.ocp.solver_options.hessian_approx = 0
        self.ocp.solver_options.regularize_method = 'NO_REGULARIZE'
        self.ocp.solver_options.nlp_solver_max_iter = 4
        self.ocp.solver_options.nlp_solver_tol_stat = 1.0
        self.ocp.solver_options.nlp_solver_tol_eq = 1e-1
        self.ocp.solver_options.nlp_solver_tol_ineq = 1e-2
        self.ocp.solver_options.nlp_solver_tol_comp = 1e-1
        self.ocp.solver_options.nlp_solver_step_length = 0.05
        self.ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
        # self.ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
        self.ocp.solver_options.qp_solver_iter_max = 10
        self.ocp.solver_options.qp_solver_cond_N = int(self.ocp.dims.N/2)


        self.ocp.solver_options.qp_solver_warm_start = 0

        self.ocp.solver_options.sim_method_num_stages = 4
        self.ocp.solver_options.sim_method_num_steps = 3
        self.ocp.model = self.model


        acados_source_path = os.environ['ACADOS_SOURCE_DIR']
        sys.path.insert(0, acados_source_path)
        self.ocp.acados_include_path = acados_source_path + '/include'
        self.ocp.acados_lib_path = acados_source_path + '/lib'

        # acados_ocp_solver = AcadosOcpSolver(ocp, json_file='acados_ocp_' + model.name + '.json')

        self.solver = AcadosOcpSolver(self.ocp, json_file='acados_ocp_' + self.model.name + '.json')


def plotTrajectory(numAgent, data, obstacle):
    plt.rcParams['figure.dpi'] = 200
    plt.figure(facecolor='w')
    line = plt.gca()
    line.set_aspect(1)
    line.patch.set_facecolor('white')
    plt.xlim(-8, 8)
    plt.ylim(-8, 8)

    color = ['black', 'darkorange', 'forestgreen', 'slategrey', 'lightcoral', 'gold', 'mediumturquoise', 'darkviolet',
             'gray', 'burlywood', 'limegreen', 'cornflowerblue', 'firebrick', 'khaki', 'teal', 'plum',
             'silver', 'darkgoldenrod', 'lime', 'slateblue', 'red', 'yellow', 'cyan', 'purple',
             'lightgrey', 'gold', 'turquoise', 'blueviolet', 'darksalmon', 'darkseagreen', 'deepskyblue', 'hotpink']

    for i in range(numAgent):
        x = []
        y = []
        for j in range(len(data)):
            x.append(data[j][3 * i + 1])
            y.append(data[j][3 * i + 0])
        k = i % len(color)
        line.plot(x, y, color[i])
    for i in range(numObstacles):
        k = i % len(color) + numAgent
        r = obstaclePositionList[3 * i + 2]
        center = [None] * 2
        center[0] = obstaclePositionList[3 * i + 1]
        center[1] = obstaclePositionList[3 * i + 0]
        x = np.linspace(center[0] - r, center[0] + r, 5000)
        y1 = np.sqrt(r ** 2 - (x - center[0]) ** 2) + center[1]
        y2 = -np.sqrt(r ** 2 - (x - center[0]) ** 2) + center[1]
        plt.plot(x, y1, c='k')
        plt.plot(x, y2, c='k')

    plt.xlabel('Y / m')
    plt.ylabel('X / m')
    plt.savefig('track.png')
    plt.show()


if __name__ == '__main__':

    # initx = np.array([1]*30)
    initx = np.array([1.264159998732038, -0.8916011767763817, -2.5, 1.2539152395810407, -0.3085129002509677, -2.5,
                      1.2688904234780618, -0.00316691427353433, -2.5, 1.2555256553382745, 0.31782030957973084, -2.5,
                      1.2643282271126681, 0.8942179961509248, -2.5, 0.039925713048763994, 0.0022601337274828523, 0,
                      0.039699735413837166, 0.0048800290520242, 0, 0.04000247645216936, -0.00024145288295886365, 0,
                      0.03980800556772525, -0.003728883269117943, 0, 0.03995350776085171, -0.002067290982718784, 0])
    # initx = np.array([0,
    #                -1,
    #                -2.5,
    #                0,
    #                -0.5,
    #                -2.5,
    #                0,
    #                0,
    #                -2.5,
    #                0,
    #                0.5,
    #                -2.5,
    #                0,
    #                1,
    #                -2.5,
    #                 0.05,
    #                 0,
    #                 0,
    #                 0.05,
    #                 0,
    #                 0,
    #                 0.05,
    #                 0,
    #                 0,
    #                 0.05,
    #                 0,
    #                 0,
    #                 0.05,
    #                 0,
    #                 0])
    u_init = np.array([0] * 15)
    x_init = initx
    x_init = np.stack(x_init)

    example = NMPC(5, T, dt)
    example.initModel(initx)

    timeStep = [i for i in range(int(T/dt) + 1)]
    tmp = []
    for i in range(15):
        tmp.append(timeStep)
    tmp1 = np.array(tmp)
    tmp2 = []
    tmpArray = [1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0]
    for i in range(int(T/dt) + 1):
        tmp2.append(tmpArray)
    tmp3 = 0.5*dt*np.array(tmp2).transpose()
    # b = 0.5*0.1*ca.repmat([1, 0, 0], 5, 41)
    c = []
    for i in range(int(T/dt) + 1):
        c.append(initx[0:15])
    c = np.array(c).transpose()
    posTraj = c + tmp3*tmp1
    x_traj_init = np.append(posTraj, tmp3*10, axis=0)

    u_traj_init = np.zeros((15, int(T/dt)))
    x_history = []

    # x_traj_init = np.array([[ 2.20519551e+00,  2.39492302e+00,  2.57214974e+00,  2.73276355e+00,   2.87290699e+00,  2.98793143e+00,  3.07213898e+00,  3.11853523e+00,   3.11853523e+00],
    #                         [-8.24367144e-01 -8.12495195e-01 -8.04784988e-01 -8.00603471e-01,  -7.98703662e-01 -7.97951005e-01 -7.97479514e-01 -7.97260327e-01,  -7.97260327e-01],
    #                         [-2.50000000e+00 -2.50000000e+00 -2.50000000e+00 -2.50000000e+00,  -2.50000000e+00 -2.50000000e+00 -2.50000000e+00 -2.50000000e+00,  -2.50000000e+00],
    #                         [ 2.09455470e+00  2.26600537e+00  2.42847096e+00  2.57718083e+00,   2.70815444e+00  2.81664308e+00  2.89686358e+00  2.94171361e+00,   2.94171361e+00],
    #                         [-1.85298190e-01 -1.60248069e-01 -1.37487364e-01 -1.18535732e-01,  -1.03839867e-01 -9.33966575e-02 -8.68633488e-02 -8.38916511e-02,  -8.38916511e-02],
    #                         [-2.50000000e+00 -2.50000000e+00 -2.50000000e+00 -2.50000000e+00,  -2.50000000e+00 -2.50000000e+00 -2.50000000e+00 -2.50000000e+00,  -2.50000000e+00],
    #                         [ 2.29810388e+00  2.50366159e+00  2.69376345e+00  2.86495477e+00,   3.01347916e+00  3.1346595...)
    # u_traj_init = np.array([[ 7.87203129e-03  1.49952687e-04 -4.25219112e-04 -5.23859229e-04,  -5.36898171e-04 -5.57103755e-04 -6.06791183e-04 -6.06791183e-04],
    #                         [-1.55390986e-02 -1.61842424e-02 -1.03005744e-02 -4.11299285e-03,  -2.15172970e-04  1.14813334e-03  1.29441438e-03  1.29441438e-03],
    #                         [ 0.00000000e+00  0.00000000e+00  0.00000000e+00  0.00000000e+00,   0.00000000e+00  0.00000000e+00  0.00000000e+00  0.00000000e+00],
    #                         [ 3.28399708e-02  2.23229333e-02  2.36072370e-02  2.59858510e-02,   2.92261558e-02  3.32831834e-02  3.81438303e-02  3.81438303e-02],
    #                         [-7.38070352e-03 -1.80523621e-02 -2.12291970e-02 -2.30951334e-02,  -2.38615639e-02 -2.42688865e-02 -2.44745835e-02 -2.44745835e-02],
    #                         [ 0.00000000e+00  0.00000000e+00  0.00000000e+00  0.00000000e+00,   0.00000000e+00  0.00000000e+00  0.00000000e+00  0.00000000e+00],
    #                         [-1.50954492e-02 -2.02846622e-02 -2.25427248e-02 -2.55210433e-02,  -2.94590465e-02 -3.42423514e-02 -3.97863616e-02 -3.97863616e-02],
    #                         [-0.0203674,   0.00018348,  0.00855485,  0.01141708,  0.01096817,  0.00901214,  0.00741071,  0.00741071],
    #                         [0., 0., 0., 0., 0., 0., 0., 0.],
    #                         [0.0127516,  0.00550233, 0.00525449, 0.00542417, 0.00572963, 0.00625038, 0.00696034, 0.00696034],
    #                         [0.01379611, 0.01612874, 0.01590964, 0.01480092, 0.01299751, 0.01155868, 0.01062865, 0.01062865],
    #                         [0., 0., 0., 0., 0., 0., 0., 0.],
    #                         [8.82350542e-03  1.03775001e-03  3.47087920e-04  1.00110393e-04,
    #                          -2.94254595e-05 - 1.03229594e-04 - 1.46921628e-04 - 1.46921628e-04],
    #                         [1.31595556e-02  1.43779405e-02  1.05773763e-02  6.03089607e-03, 2.46810274e-03
    #                          6.01260857e-04 - 4.61123576e-05 - 4.61123576e-05],
    #                         [0. 0. 0. 0. 0. 0. 0. 0.]
    #                         ])

    for step in range(12000):
        for i in range(int(T/dt) + 1):
            example.solver.set(i, "x", x_traj_init[:, i])
        for i in range(int(T/dt)):
            example.solver.set(i, "u", u_traj_init[:, i])

        # example.solver.set(0, "x", x_init)
        # example.solver.set(0, "u", u_init)
        example.solver.set(0, 'lbx', x_init)
        example.solver.set(0, 'ubx', x_init)
        x_history.append(x_init.copy())
        # self.acadosOcpSolver.set(0, 'x', x_init)
        # self.acadosOcpSolver.set(0, 'u', u_init)
        example.solver.solve()

        uOptAcados = np.ndarray((int(T/dt), 15))
        xOptAcados = np.ndarray((int(T/dt) + 1, 30))
        for i in range(int(T/dt)):
            xOptAcados[i, :] = example.solver.get(i, "x")
            uOptAcados[i, :] = example.solver.get(i, "u")
        xOptAcados[int(T/dt), :] = example.solver.get(int(T/dt), "x")
        x_traj_init = xOptAcados[1:]
        x_traj_init = np.append(x_traj_init, [xOptAcados[-1, :]], axis=0)
        x_traj_init = x_traj_init.transpose()
        u_traj_init = uOptAcados[1:]
        u_traj_init = np.append(u_traj_init, [uOptAcados[-1, :]], axis=0)
        u_traj_init = u_traj_init.transpose()

        x_init[0:15] = x_init[0:15] + dt*x_init[15:] + 0.5*uOptAcados[0, :]*dt**2
        x_init[15:] = x_init[15:] + uOptAcados[0, :]*dt

        if x_init[0] > 10:
            break

        for k in range(len(x_init)):
            with open(os.path.dirname(__file__) + 'track.txt', 'a') as f:
                f.write(str(x_init[k]) + ' ')
        with open(os.path.dirname(__file__) + 'track.txt', 'a') as f:
            f.write('\n')

    plotTrajectory(5, x_history, obstaclePosition)



