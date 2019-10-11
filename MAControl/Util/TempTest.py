import numpy as np
import math
a = []

a.append([[1, 2, 1],
          [6, 5, 1],
          [2, 8, 1],
          [0, 0, 0],
          [0, 0, 0]])

b = [1, 2, 3, 4, 5, 6, 7, 8, 9]

c = ['wzq', 'xj', 'lj', 'ybb', 'lc']

for i, t in enumerate(c):
    print(i, t)

z = 0

vel = np.array([3,4])
print(np.linalg.norm(vel))
print(vel[1]/vel[0])
intersection = [0]*2
intersection[1] = [1, 2]
intersection.append([3, 4])
print(intersection[1])
print(intersection[1][1])

for i in range(3):
    print(i,'\n')

sm=[0]*2
sm.insert(0,1)
print(sm)