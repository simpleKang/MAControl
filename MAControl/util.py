def constrain(x, Min, Max):
    if x < Min:
        return Min
    elif x > Max:
        return Max
    else:
        return x


init_waypoint = []

init_waypoint.append([[-0.9, -0.9, 1],
                      [-0.9, 0.9, 1],
                      [0.9, 0.9, 1],
                      [0.9, -0.9, 1]])

init_waypoint.append([[-0.6, -0.6, 1],
                      [-0.6, 0.6, 1],
                      [0.6, 0.6, 1],
                      [0.6, -0.6, 1]])

init_waypoint.append([[-0.3, -0.3, 1],
                      [-0.3, 0.3, 1],
                      [0.3, 0.3, 1],
                      [0.3, -0.3, 1]])
