import numpy as np
import os
import math


def coverage(index):

    def constrain(edge, pn, cell):
        if pn == 'down':
            if edge < cell:
                edge = 0
        elif pn == 'up':
            if edge > cell:
                edge = cell
        return edge

    def cal_cover_rate(area):

        length, width = area.shape
        cover = 0
        for i in range(length):
            for j in range(width):
                if area[i][j] > 0:
                    cover += 1
        cover_rate = cover / (length * width)
        overlap_rate = (np.sum(area) - cover) / (length * width)

        return cover_rate, overlap_rate

    def agent_cover_range(obs):

        selfvel = np.array(obs[0:2])
        selfpos = np.array(obs[2:4])
        selfvelunit = selfvel / np.sqrt(np.dot(selfvel, selfvel))
        selfdir = math.atan2(selfvel[1], selfvel[0])
        d1 = 0.1
        d2 = 0.2
        d3 = 0.2
        xx1 = -d3 / 2 * math.cos(selfdir) - d2 / 2 * math.sin(selfdir) * -1
        xx2 = -d3 / 2 * math.cos(selfdir) + d2 / 2 * math.sin(selfdir) * -1
        xx3 = d3 / 2 * math.cos(selfdir) + d2 / 2 * math.sin(selfdir) * -1
        xx4 = d3 / 2 * math.cos(selfdir) - d2 / 2 * math.sin(selfdir) * -1
        yy1 = -d3 / 2 * math.sin(selfdir) - d2 / 2 * math.cos(selfdir)
        yy2 = -d3 / 2 * math.sin(selfdir) + d2 / 2 * math.cos(selfdir)
        yy3 = d3 / 2 * math.sin(selfdir) + d2 / 2 * math.cos(selfdir)
        yy4 = d3 / 2 * math.sin(selfdir) - d2 / 2 * math.cos(selfdir)
        selfview1 = selfpos + selfvelunit * (d1 + d2 / 2) + np.array([xx1, yy1])
        selfview2 = selfpos + selfvelunit * (d1 + d2 / 2) + np.array([xx2, yy2])
        selfview3 = selfpos + selfvelunit * (d1 + d2 / 2) + np.array([xx3, yy3])
        selfview4 = selfpos + selfvelunit * (d1 + d2 / 2) + np.array([xx4, yy4])

        return selfview1, selfview2, selfview3, selfview4

    def point_in_rec(point1, point2, point3, point4, pointk):
        p1p2 = point2 - point1
        p2p3 = point3 - point2
        p3p4 = point4 - point3
        p4p1 = point1 - point4

        p1pk = pointk - point1
        p2pk = pointk - point2
        p3pk = pointk - point3
        p4pk = pointk - point4

        term1 = np.dot(p1p2, np.array([p1pk[1], -1 * p1pk[0]]))
        term2 = np.dot(p3p4, np.array([p3pk[1], -1 * p3pk[0]]))
        term3 = np.dot(p2p3, np.array([p2pk[1], -1 * p2pk[0]]))
        term4 = np.dot(p4p1, np.array([p4pk[1], -1 * p4pk[0]]))

        return True if term1 * term2 >= 0 and term3 * term4 >= 0 else False

    curdir = os.path.dirname(__file__)
    pardir = os.path.dirname(os.path.dirname(curdir))

    cell = 400
    num = 20
    edge = 2

    txt_name = '/cover_rate-%s-%s-%s.txt' % (str(20), str(4000), str(index))
    open(pardir + txt_name, 'w')

    track = []
    last_cover = []
    for i in range(num):
        track.append(np.loadtxt(pardir + '/track_%d/agent_%d_track.txt' % (index, i)))
        last_cover.append([])

    area = np.zeros((cell, cell))
    area_width = edge*2
    scale = area_width/cell
    iter_range = 0.25

    for l in range(0, np.size(track[-1], 0), 5):

        new_last = []

        for k in range(num):

            new_last.append([])

            x = track[k][l][2]
            y = track[k][l][3]

            width_down = int(constrain(round((edge - y - iter_range) / scale), 'down', 0))
            width_up = int(constrain(round((edge - y + iter_range) / scale), 'up', cell))
            length_down = int(constrain(round((edge + x - iter_range) / scale), 'down', 0))
            length_up = int(constrain(round((edge + x + iter_range) / scale), 'up', cell))

            point1, point2, point3, point4 = agent_cover_range(track[k][l])

            for i in range(width_down, width_up):
                for j in range(length_down, length_up):
                    target = np.array([scale*j-edge, edge-scale*i])
                    if point_in_rec(point1, point2, point3, point4, target):
                        if area[i][j] == 0:
                            area[i][j] = 1
                            new_last[k].append([i, j])
                        elif area[i][j] > 0 and [i, j] in last_cover[k]:
                            new_last[k].append([i, j])
                        elif area[i][j] > 0 and [i, j] not in last_cover[k]:
                            area[i][j] += 1
                            new_last[k].append([i, j])
                        else:
                            raise Exception('Unexpected situation!!!')
        last_cover[:] = new_last[:]
        print('Total ', np.size(track[-1], 0), ' >>> step ', l)

        cover_rate, overlap_rate = cal_cover_rate(area)

        with open(pardir + txt_name, 'a') as c:
            c.write(str(l) + ' ' + str(cover_rate) + ' ' + str(overlap_rate) + '\n')

    print('Finished!')


for z in range(4, 30):
    coverage(z)
