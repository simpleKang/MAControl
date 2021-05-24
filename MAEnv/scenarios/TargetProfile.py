import numpy as np
import math
import MAControl.Util.get_random_state as rs
# 这里的 size 以【米】为单位 在 scenario 里边会乘上 0.001 从而以【千米】为单位

# blind angle
blind_angle = [math.pi/3, math.pi/3*2, math.pi, math.pi/3*4]
# visual section
num_section = 10

# UAV
UAV_color = np.array([0.47, 0.79, 0.79])
UAV_H = 100
UAV_Dam = 1
UAV_w = 1000
UAV_size = 4

# target parameters
num_targets = 4
target_size = [12, 12, 12, 12, 12, 12, 12, 12, 12, 12]
target_pos = [[+0.85, +0.30],
              [+0.40, -0.24],
              [-0.85, +0.23],
              [-0.88, +0.51],
              [-0.23, +0.28],
              [-0.14, +0.82],
              [+0.75, -0.05],
              [-0.43, -0.79],
              [-0.38, +0.18],
              [-0.83, -0.31]]
target_movable = [False, False, False, False, False, False, False, False, False, False]
movable_target_color = np.array([0, 0.8, 0])
fixed_target_color = np.array([0.3, 0.2, 0])
target_H = [3, 5, 5, 5, 1, 1, 1, 1, 1, 1]
target_Dam = [1, 2, 1, 5, 0, 1, 2, 1, 0, 1]
target_w = [2, 2, 5, 2, 3, 2, 2, 5, 2, 3]
target_init = [0, 1, 0, 1, 0, 1, 0, 1, 0, 1]

# edge of the battle field (a kind of non-obstacle)
edge = 1
num_grids = 5
grid_size = 20
grid_pos = [[0, 0], [edge, edge], [-edge, edge], [-edge, -edge], [edge, -edge]]
grid_color = np.array([0.25, 0.25, 0.25])
grid_obstacle = False

# 在 core.py 文件里规定了 landmark 有 .obstacle 属性
# True >> 不能飞越的区域   # False >> 仅仅只是标示出来的区域 对于飞行没有限制
# 在 scenarioXXXXX.py 中初次定义各 landmark 时需指定其 .obstacle 取值

# 环境里不存在自然斥力，必须由 agent 做出避障决策才会有避开障碍的效果

# square (a kind of obstacle)
num_square = 0
square_size = 50
square_pos = [[0., 0.],
              [1., 1.],
              [1., -1.],
              [-1., 1.],
              [-1., -1.]]
square_color = np.array([0.25, 0.49, 0.75])
square_obstacle = True

init_state = None
