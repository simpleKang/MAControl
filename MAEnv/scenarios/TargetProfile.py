import numpy as np

num_targets = 5

target_value = [4, 2, 3, 1, 5]
target_size = [4, 2, 3, 1, 5]  # 大小与价值相匹配
target_defence = [4, 2, 3, 1, 5]  # 防御能力与价值相匹配
target_pos = [[-0.5, 0.9], [0, 0.9], [0.5, -0.5], [0.0, 0.0], [-0.9, -0.9]]

grid_pos = [[0, 0], [-1, 1], [-1, -1], [1, 1], [1, -1]]
grid_color = np.array([0.25, 0.25, 0.25])

agent_color = np.array([0.47, 0.79, 0.79])
