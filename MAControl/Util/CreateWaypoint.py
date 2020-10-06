import random
import os
import numpy as np
import operator
import MAControl.Util.SignIsSame as sis
_path_ = '/track/vel.txt' if os.name == 'posix' else '\\track\\vel.txt'

# open(os.path.dirname(os.path.dirname(os.path.dirname(__file__))) + _path_, 'w')


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


def creat_veledge_point(pos, vel, cur_vel, edge):

    waypoint_list = list(pos)
    delta = 0.2
    try:
        vel_length = np.linalg.norm(vel)
        if vel_length == 0.:
            vel = cur_vel / np.linalg.norm(cur_vel)
        else:
            vel /= vel_length
        # with open(os.path.dirname(os.path.dirname(os.path.dirname(__file__))) + _path_, 'a') as f:
        #     f.write(str(vel[0]) + ' ' + str(vel[1]) + '\n')
    except:
        print(vel)
    while True:
        waypoint_list[0] += delta * vel[0]
        waypoint_list[1] += delta * vel[1]
        if edge - abs(waypoint_list[0]) < 0.01 or edge - abs(waypoint_list[1]) < 0.01:
            waypoint_list = [round(waypoint_list[0], 3), round(waypoint_list[1], 3)]
            if abs(waypoint_list[0]) > edge:
                if waypoint_list[0] > 0:
                    waypoint_list[0] = edge
                else:
                    waypoint_list[0] = -edge
            if abs(waypoint_list[1]) > edge:
                if waypoint_list[1] > 0:
                    waypoint_list[1] = edge
                else:
                    waypoint_list[1] = -edge
            break

    return waypoint_list


def creat_random_edgepoint(pos, Edge):

    candidate_edge = []  # 候选边编号
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


def creat_reflection_edgepoint(current_point, vel, edge):

    if edge - abs(current_point[0]) < 0.1:
        new_direction = np.array([-vel[0], vel[1]])
    elif edge - abs(current_point[1]) < 0.1:
        new_direction = np.array([vel[0], -vel[1]])
    else:
        raise Exception('There is a weird waypoint!!!')

    waypoint_list = list(current_point).copy()

    delta = 0.7
    x = y = False

    if edge - abs(waypoint_list[0]) < 0.05:
        x = True
    if edge - abs(waypoint_list[1]) < 0.05:
        y = True

    while True:

        waypoint_list[0] += delta * new_direction[0]
        waypoint_list[1] += delta * new_direction[1]

        if x:
            if not sis.sign_is_same(current_point[0], waypoint_list[0]) and edge - abs(waypoint_list[0]) < 0.01:
                waypoint_list = [round(waypoint_list[0], 3), round(waypoint_list[1], 3)]
                break
            elif edge - abs(waypoint_list[1]) < 0.01:
                waypoint_list = [round(waypoint_list[0], 3), round(waypoint_list[1], 3)]
                break

        if y:
            if not sis.sign_is_same(current_point[1], waypoint_list[1]) and edge - abs(waypoint_list[1]) < 0.01:
                waypoint_list = [round(waypoint_list[0], 3), round(waypoint_list[1], 3)]
                break
            elif edge - abs(waypoint_list[0]) < 0.01:
                waypoint_list = [round(waypoint_list[0], 3), round(waypoint_list[1], 3)]
                break

    return waypoint_list
