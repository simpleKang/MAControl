
N = 10
W = 0.9
D = 0.05
Edge = 1
Up = []
Down = []
Wide1 = []
Wide2 = []

init_waypoint = []


def snake_single(Up, Down, Wide1, Wide2, Edge, W, D, i):
    wp_list = []
    point = [round(-W+i*D, 2), round(-W, 2), 1]
    wp_list.append(point)
    while True:

        point = [round(point[0], 2), round(Up, 2), 1]
        wp_list.append(point)

        point = [round(point[0]+Wide1, 2), round(Up, 2), 1]
        if point[0] > Edge:
            break
        wp_list.append(point)

        point = [round(point[0], 2), round(Down, 2), 1]
        wp_list.append(point)

        point = [round(point[0]+Wide2, 2), round(Down, 2), 1]
        if point[0] > Edge:
            break
        wp_list.append(point)

    return wp_list


for i in range(N):
    Up.append(round(W-D*i, 2))
    Down.append(round(-W+D*(N-1-i), 2))
    Wide1.append(round(D*(2*(N-i)-1), 2))
    Wide2.append(round(D*(2*i+1), 2))
    init_waypoint.append(snake_single(Up[i], Down[i], Wide1[i], Wide2[i], Edge, W, D, i))


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

