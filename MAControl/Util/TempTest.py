
a = []

a.append([[1, 2, 1],
          [6, 5, 1],
          [2, 8, 1],
          [0, 0, 0],
          [0, 0, 0]])

a.append(a[0].copy())

a[1][3] = [3, 3, 3]

x = 3

y = x

for i in range(5):
    y += 1

z = 0
