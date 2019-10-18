import random

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


def create_simple_waypoint_list(waypoint_list, new_list_index, init_point, Edge):

    waypoint_list.append([[0 for j in range(3)] for k in range(256)])
    wp_list = []
    point = [init_point[0], init_point[1], 1]
    wp_list.append(point)
    while True:
        point = [point[0], point[1]+0.2, 1]
        if abs(point[1]) > Edge:
            break
        wp_list.append(point)
    waypoint_list[-1][0:len(wp_list)] = wp_list[:]
    new_list_index += 1


def creat_initial_waypoint_list(pos, vel, Edge=1):

    intersection = [[0]*2 for i in range(4)]
    waypoint_list = [[0]*2]
    vel_arg = round((vel[1]/vel[0]), 2)
    intersection[0] = [pos[0] + (Edge - pos[1]) / vel_arg, 1]
    intersection[1] = [pos[0] - (Edge + pos[1]) / vel_arg, -1]
    intersection[2] = [1, pos[1] + (Edge - pos[0]) * vel_arg]
    intersection[3] = [-1, pos[1] - (Edge + pos[0]) * vel_arg]
    for i in range(4):
        if abs(intersection[i][0]) <= Edge and abs(intersection[i][1]) <= Edge:
            if (intersection[i][0] - pos[0])/ vel[0] > 0:
                waypoint_list[0] = intersection[i]
                break
        else:
            waypoint_list[0] = pos
    current_wplist = 0

    return waypoint_list, current_wplist


def creat_random_edgepoint(pos, Edge):

    candidate_edge = []  #候选边编号
    new_edgepoint = []

    # 判断目前航点所在边编号。上边1,下边2，右边3，左边4
    if pos[0] == Edge:
        current_edge = 3
    elif pos[0] == -1 * Edge:
        current_edge = 4
    elif pos[1] == Edge:
        current_edge = 1
    elif pos[1] == -1 * Edge:
        current_edge = 2
    else:
        current_edge = 1

    # 生成候选边序列，即不包含目前边的边序列
    for i in range(4):
        if i != (current_edge - 1):
            candidate_edge.append(i+1)

    # 候选边序列随机选取边编号，为新航点所在边编号;并在该边上随机选取一点
    new_edge = candidate_edge[random.randint(0, 2)]
    random_point = random.uniform(-1 * Edge, Edge)

    # 根据新航点所在边编号，生成新航点
    if new_edge == 1:
        new_edgepoint = [random_point, Edge]
    elif new_edge == 2:
        new_edgepoint = [random_point, -1 * Edge]
    elif new_edge == 3:
        new_edgepoint = [Edge, random_point]
    elif new_edge == 4:
        new_edgepoint = [-1 * Edge, random_point]

    return new_edgepoint