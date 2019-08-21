import numpy as np


def distance(point, goal):
    x, y = point
    g_x, g_y = goal
    return np.sqrt((x - g_x) * (x - g_x) + (y - g_y) * (y - g_y))


def min_dis_index(point, path, i):
    path = np.array(path)
    delta = point - path
    dis = np.sqrt(np.sum(delta * delta, axis=1))
    return np.argmin(dis) + i


def interpolate_path(path, d=80):
    path = np.array(path)
    num = len(path)
    path_ = []
    for i in range(num-1):
        dis = distance(path[i], path[i+1])
        if dis <= 2 * d:
            path_.append(path[i][np.newaxis, :])
            continue
        else:
            n = int(dis / d)
            x, y = path[i]
            x_, y_ = path[i+1]
            x_add = np.linspace(x, x_, num=2+n, endpoint=True)
            y_add = np.linspace(y, y_, num=2+n, endpoint=True)
            points = np.concatenate((x_add[:, np.newaxis], y_add[:, np.newaxis]), axis=1)
            path_.append(points[:-1, :])
    path_.append(path[-1][np.newaxis, :])
    return np.concatenate(path_, axis=0)
