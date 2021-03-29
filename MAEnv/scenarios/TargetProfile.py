import numpy as np

# UAV
UAV_color = np.array([0.47, 0.79, 0.79])
UAV_H = 100
UAV_Dam = 10
UAV_w = 1000

# target parameters
num_targets = 0
target_size = [2, 2, 2, 2, 2, 2, 2, 2, 2, 2]
target_pos = [[0,    1.5],
              [1.5,  0],
              [1.5,  1.5],
              [1.0,   0.4],
              [0.8,   1.4],
              [1,    1],
              [0.6, 0.2],
              [1.2, 1.8],
              [1.6,  -0.6],
              [0.6,  -1.2]]
target_movable = [False, False, False, False, False, False, False, False, False, False]
movable_target_color = np.array([0, 0.8, 0])
fixed_target_color = np.array([0.3, 0.2, 0])
target_H = [10, 10, 10, 10, 10, 8, 8, 8, 8, 8]
target_Dam = [1, 2, 1, 5, 0, 1, 2, 1, 0, 1]
target_w = [2, 2, 5, 2, 3, 2, 2, 5, 2, 3]

# edge of the battle field (a kind of non-obstacle)
edge = 2
num_grids = 5
grid_size = 0.005
grid_pos = [[0, 0], [edge, edge], [-edge, edge], [-edge, -edge], [edge, -edge]]
grid_color = np.array([0.25, 0.25, 0.25])
grid_obstacle = False

# 在 core.py 文件里规定了 landmark 有 .obstacle 属性
# True >> 不能飞越的区域   # False >> 仅仅只是标示出来的区域 对于飞行没有限制
# 在 scenarioXXXXX.py 中初次定义各 landmark 时需指定其 .obstacle 取值

# 环境里不存在自然斥力，必须由 agent 做出避障决策才会有避开障碍的效果

# square (a kind of obstacle)
num_square = 0
square_size = 0.05
square_pos = [[0., 0.],
              [1., 1.],
              [1., -1.],
              [-1., 1.],
              [-1., -1.]]
square_color = np.array([0.25, 0.49, 0.75])
square_obstacle = True



