import numpy as np

# target parameters
num_targets = 5
target_value = [7, 5, 2, 3, 5, 6, 8, 1, 3, 4]
target_size = [2, 2, 2, 2, 2, 2, 2, 2, 2, 2]
target_defence = [1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
target_pos = [[0, 0], [3, 2], [6, 8], [-5, 2], [-7, 8], [-2, 5], [-3, -1], [-6, -4], [8, -3], [3, -6]]

# movable target parameters
m_num_targets = 0
m_target_value = [7, 5, 2, 3, 5]
m_target_size = [2, 2, 2, 2, 2]
m_target_defence = [1, 1, 1, 1, 1]
m_target_pos = [[-0.8, -0.9], [-0.4, -0.9], [0, -0.9], [0.4, -0.9], [0.8, -0.9]]
m_target_color = np.array([0, 0.8, 0])

edge = 2
grid_pos = [[0, 0], [edge, edge], [-edge, edge], [-edge, -edge], [edge, -edge]]
grid_color = np.array([0.25, 0.25, 0.25])

agent_color = np.array([0.47, 0.79, 0.79])
# agent_color = np.array([0.9, 0.3, 0.5])
