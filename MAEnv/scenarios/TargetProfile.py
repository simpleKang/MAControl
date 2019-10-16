import numpy as np

num_targets = 6

target_type = [1, 1, 1, 1, 2, 3]  # 1=发射车 2=雷达 3=补给车
target_value = [2, 2, 2, 2, 10, 5]
target_size = [2, 2, 2, 2, 2, 2]  # 大小相同
target_defence = [5, 5, 5, 5, 1, 2]
target_pos = [list((np.random.choice(200, 2)-100)/100) for i in range(6)]

grid_pos = [[0, 0], [-1, 1], [-1, -1], [1, 1], [1, -1]]
grid_color = np.array([0.25, 0.25, 0.25])

agent_pos_init = [-1.0, -0.9]
agent_color = np.array([0.47, 0.79, 0.79])

