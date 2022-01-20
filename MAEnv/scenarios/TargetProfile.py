import numpy as np

num_targets = 60

target_size = [2 for ki in range(60)]  # 大小相同
target_pos = [list((np.random.choice(200, 2)-100)/100) for i in range(60)]

grid_pos = [[0, 0], [-1, 1], [-1, -1], [1, 1], [1, -1]]
grid_color = np.array([0.25, 0.25, 0.25])

agent_pos_init = [-1.0, -1.0]
agent_color = np.array([0.47, 0.79, 0.79])

