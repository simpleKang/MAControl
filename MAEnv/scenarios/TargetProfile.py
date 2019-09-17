import numpy as np

num_targets = 6

target_type = [1, 1, 1, 1, 2, 3]  # 1=发射车 2=雷达 3=补给车
target_value = [2, 2, 2, 2, 10, 5]
target_size = [2, 2, 2, 2, 10, 5]  # 大小与价值相匹配
target_defence = [5, 5, 5, 5, 1, 2]
target_pos = [[-0.5, 0.9], [-0.5, 0.5], [0.0, 0.8], [0.0, 0.4], [-0.9, -0.9], [0.5, -0.5]]

grid_pos = [[0, 0], [-1, 1], [-1, -1], [1, 1], [1, -1]]
grid_color = np.array([0.25, 0.25, 0.25])

agent_color = np.array([0.47, 0.79, 0.79])
