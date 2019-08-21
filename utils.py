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
