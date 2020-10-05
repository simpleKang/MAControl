import numpy as np
import math


def viewfield(x, y, h, pitch, roll, heading, wview, hview):

    ptch1 = pitch + hview
    ptch2 = pitch - hview
    head1 = heading + wview
    head2 = heading - wview

    #                 heading <-----   (x,y)
    #                                   /|
    #                                  / | h
    #             ( pitch ± hview )   /  |
    # ---------------------------------------------------
    #               ( ± wview )    上述为简化示意图
    #

    p1 = cross_point(roll, ptch1, head1, x, y, h)
    p2 = cross_point(roll, ptch1, head2, x, y, h)
    p3 = cross_point(roll, ptch2, head2, x, y, h)
    p4 = cross_point(roll, ptch2, head1, x, y, h)
    proj = np.array([p1, p2, p3, p4])

    return proj


def cross_point(rll, ptch, head, x, y, h):

    vo = np.array(math.sin(head), math.cos(head), 0)
    rot_rll = np.array([1, 0, 0], [0, math.cos(rll), math.sin(rll)], [0, -math.sin(rll), math.cos(rll)])
    rot_ptch = np.array([math.cos(ptch), 0, -math.sin(ptch)], [0, 1, 0], [math.sin(ptch), 0, math.cos(ptch)])
    vt = np.matmul(rot_ptch, np.matmul(rot_rll, vo))

    #                                 (x,y,h)
    #                                   /
    #                                  /   vt = (a,b,c)
    #                   (x0, y0, 0)   /
    # ---------------------------------------------------
    #

    x0 = x - vt[0] * h / vt[2]
    y0 = y - vt[1] * h / vt[2]
    point0 = np.array([x0, y0])

    return point0
