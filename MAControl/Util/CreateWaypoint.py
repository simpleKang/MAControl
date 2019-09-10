
def creat_snake_waypoint_list(waypoint_list, N, W=0.9, D=0.05, Edge=1):
    init_waypoint = []

    Up = []
    Down = []
    Wide1 = []
    Wide2 = []
    for i in range(N):
        Up.append(round(W - D * i, 3))
        Down.append(round(-W + D * (N - 1 - i), 3))
        Wide1.append(round(D * (2 * (N - i) - 1), 3))
        Wide2.append(round(D * (2 * i + 1), 3))
        init_waypoint.append(snake_single(Up[i], Down[i], Wide1[i], Wide2[i], Edge, W, D, i))






def snake_single(Up, Down, Wide1, Wide2, Edge, W, D, i):
    wp_list = []
    point = [round(-W+i*D, 3), round(-W, 3), 1]
    wp_list.append(point)
    while True:

        point = [round(point[0], 3), round(Up, 3), 1]
        wp_list.append(point)

        point = [round(point[0]+Wide1, 3), round(Up, 3), 1]
        if point[0] > Edge:
            break
        wp_list.append(point)

        point = [round(point[0], 3), round(Down, 3), 1]
        wp_list.append(point)

        point = [round(point[0]+Wide2, 3), round(Down, 3), 1]
        if point[0] > Edge:
            break
        wp_list.append(point)

    return wp_list





z = 0
