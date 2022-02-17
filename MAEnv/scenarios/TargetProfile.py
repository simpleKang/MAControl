import numpy as np

obstacle_pos = [[3, -2], [3, 3], [5, 0], [5, 2], [7, -2], [7, 3]]

num_targets = 5

target_value = [7, 5, 2, 3, 5]
target_size = [2, 2, 2, 2, 2]
target_defence = [1, 1, 1, 1, 1]
target_pos = [[-0.5, 0.9], [0, 0.9], [0.5, -0.5], [0.0, 0.0], [-0.9, -0.9]]

# 先5个点，后环境中障碍物
grid_pos = [[0, 0], [-10, 10], [-10, -10], [10, 10], [10, -10]]
grid_color = np.array([0.25, 0.25, 0.25])

agent_color = np.array([0.47, 0.79, 0.79])

edge = 2
