import numpy as np

# target parameters
num_targets = 3
target_value = [1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
target_size = [2, 2, 2, 2, 2, 2, 2, 2, 2, 2]
target_defence = [5, 3, 1, 1, 1, 1, 1, 1, 1, 1]
target_pos = [[1.2,     1],
              [0.5,  -1.5],
              [-1.7,  0.4],
              [-1,    0.4],
              [-1.4,  1.6],
              [-0.4,    1],
              [-0.6, -0.2],
              [-1.2, -0.8],
              [1.6,  -0.6],
              [0.6,  -1.2]]

# movable target parameters
m_num_targets = 0
m_target_value = [7, 5, 2, 3, 5]
m_target_size = [2, 2, 2, 2, 2]
m_target_defence = [1, 1, 1, 1, 1]
m_target_pos = [[-0.8, -0.9], [-0.4, -0.9], [0, -0.9], [0.4, -0.9], [0.8, -0.9]]
m_target_color = np.array([0, 0.8, 0])

# edge of the battle field
edge = 2
grid_pos = [[0, 0], [edge, edge], [-edge, edge], [-edge, -edge], [edge, -edge]]
grid_color = np.array([0.25, 0.25, 0.25])

agent_color = np.array([0.47, 0.79, 0.79])
# agent_color = np.array([0.9, 0.3, 0.5])
