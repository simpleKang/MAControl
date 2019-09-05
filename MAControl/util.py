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

N = 20
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
     [1, 3, 7, [2, 9]]]

a[1].append([1, 2])




z = 0
