import numpy as np

# UAV
UAV_color = np.array([0.47, 0.79, 0.79])

# target parameters
num_targets = 5
target_size = [2, 2, 2, 2, 2, 2, 2, 2, 2, 2]
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
target_color = np.array([0, 0.8, 0])

# edge of the battle field
edge = 2
num_grids = 5
grid_size = 0.005
grid_pos = [[0, 0], [edge, edge], [-edge, edge], [-edge, -edge], [edge, -edge]]
grid_color = np.array([0.25, 0.25, 0.25])