import numpy as np
import random

def constrain(x, Min, Max):
    if x < Min:
        return Min
    elif x > Max:
        return Max
    else:
        return x


def snake_single(Up, Down, Wide1, Wide2, Edge, W, D, i):
    wp_list = []
    point = [-W+i*D, -W, 1]
    wp_list.append(point)
    while True:

        point = [point[0], Up, 1]
        wp_list.append(point)

        point = [point[0]+Wide1, Up, 1]
        if point[0] > Edge:
            break
        wp_list.append(point)

        point = [point[0], Down, 1]
        wp_list.append(point)

        point = [point[0]+Wide2, Down, 1]
        if point[0] > Edge:
            break
        wp_list.append(point)

    return wp_list


def point_in_rec(point1, point2, point3, point4, pointk):
    # p1 p2 p3 p4 make a convex quadrilateral, clockwise.
    # p1 p2 p3 p4 构成一个凸四边形 # 顺时针依次取点 #

    p1p2 = point2 - point1
    p2p3 = point3 - point2
    p3p4 = point4 - point3
    p4p1 = point1 - point4

    p1pk = pointk - point1
    p2pk = pointk - point2
    p3pk = pointk - point3
    p4pk = pointk - point4

    term1 = np.dot(p1p2, np.array([p1pk[1], -1*p1pk[0]]))
    term2 = np.dot(p3p4, np.array([p3pk[1], -1*p3pk[0]]))
    term3 = np.dot(p2p3, np.array([p2pk[1], -1*p2pk[0]]))
    term4 = np.dot(p4p1, np.array([p4pk[1], -1*p4pk[0]]))

    return True if term1*term2 >= 0 and term3*term4 >= 0 else False


init_waypoint = []

# init_waypoint.append([[-0.9, -0.9, 1],
#                       [-0.9, 0.9, 1],
#                       [0.9, 0.9, 1],
#                       [0.9, -0.9, 1]])
#
# init_waypoint.append([[-0.6, -0.6, 1],
#                       [-0.6, 0.6, 1],
#                       [0.6, 0.6, 1],
#                       [0.6, -0.6, 1]])
#
# init_waypoint.append([[-0.3, -0.3, 1],
#                       [-0.3, 0.3, 1],
#                       [0.3, 0.3, 1],
#                       [0.3, -0.3, 1]])

N = 10
W = 0.9
D = 0.05
Edge = 1
Up = []
Down = []
Wide1 = []
Wide2 = []

for i in range(N):
    Up.append(W-D*i)
    Down.append(-W+D*(N-1-i))
    Wide1.append(D*(2*(N-i)-1))
    Wide2.append(D*(2*i+1))
    init_waypoint.append(snake_single(Up[i], Down[i], Wide1[i], Wide2[i], Edge, W, D, i))

a = [[0, 1, 2],
     [1, 5, 3],
     [2, 9, 4]]

b = [1, 2, 3, 4]

c = b[1:3]



z = 0
