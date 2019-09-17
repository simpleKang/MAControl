import numpy as np


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
