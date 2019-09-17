
def creat_snake_waypoint_list(waypoint_list, N, i, new_list_index, W=0.9, D=0.05, Edge=1):

    waypoint_list.append([[0 for j in range(3)] for k in range(256)])

    Up = round(W - D * i, 3)
    Down = round(-W + D * (N - 1 - i), 3)
    Wide1 = round(D * (2 * (N - i) - 1), 3)
    Wide2 = round(D * (2 * i + 1), 3)
    new_waypoint_list = snake_single(Up, Down, Wide1, Wide2, Edge, W, D, i)
    waypoint_list[-1][0:len(new_waypoint_list)] = new_waypoint_list[:]
    new_list_index += 1

    return waypoint_list, new_list_index


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
