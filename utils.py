import numpy as np
import math


def distance(point, goal):
    x, y = point
    g_x, g_y = goal
    return np.sqrt((x - g_x) * (x - g_x) + (y - g_y) * (y - g_y))


def sigmoid(x):
    return 1 / (1 + math.exp(-x))


def tanh(x):
    s1 = np.exp(x) - np.exp(-x)
    s2 = np.exp(x) + np.exp(-x)
    return (math.exp(x) - math.exp(-x)) / (math.exp(x) + math.exp(-x))


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


def interpolate_point(point1, point2, d=30):
    x1, y1 = point1
    x2, y2 = point2
    dis = distance(point1, point2)
    if dis <= 2 * d:
        return np.array([point1, point2])
    else:
        n = int(dis / d)
        x_add = np.linspace(x1, x2, num=2 + n, endpoint=True)
        y_add = np.linspace(y1, y2, num=2 + n, endpoint=True)
        points = np.concatenate((x_add[:, np.newaxis], y_add[:, np.newaxis]), axis=1)
    return np.concatenate((np.array(point1)[np.newaxis, :], points, np.array(point2)[np.newaxis, :]), axis=0)


def check_path_l(receive, point, path, barrierId, color='blue', id=0, dis_threshold=20, index_=1, only_1=True):
    # import ipdb;ipdb.set_trace()
    infos = receive.get_infos(color, id)
    point1 = point.copy()
    point2 = path[0].copy()
    if only_1:
        N = 1
    else:
        N = len(path)
    for i in range(N):
        for index in range(len(infos)):
            center = infos[index]
            if distance(point1, center) < dis_threshold:
                return False, index_+1
            dx_1 = center[0] - point1[0]
            dy_1 = center[1] - point1[1]
            dx_2 = center[0] - point2[0]
            dy_2 = center[1] - point2[1]
            dx_0 = point1[0] - point2[0]
            dy_0 = point1[1] - point2[1]
            mul_1 = (dx_1) * (-dx_0) + (dy_1) * (-dy_0)
            mul_2 = (dx_2) * (dx_0) + (dy_2) * (dy_0)
            if mul_1 > 0 and mul_2 > 0:
                mid = abs((dx_1) * (-dy_0) - (-dx_0) * (dy_1))
                dist = mid / np.sqrt(dx_0 * dx_0 + dy_0 * dy_0)
            elif mul_1 == 0 and mul_2 != 0:
                dist = np.sqrt(dx_1 * dx_1 + dy_1 * dy_1)
            elif mul_1 != 0 and mul_2 == 0:
                dist = np.sqrt(dx_2 * dx_2 + dy_2 * dy_2)
            elif mul_1 == 0 and mul_2 == 0:
                dist = 0
            elif mul_1 < 0 and mul_2 > 0:
                dist = np.sqrt(dx_1 * dx_1 + dy_1 * dy_1)
            elif mul_2 < 0 and mul_1 > 0:
                dist = np.sqrt(dx_2 * dx_2 + dy_2 * dy_2)
            else:
                dist = 0

            if dist < dis_threshold:
                return False, index_
        point1 = path[i].copy()
        if i == N-1:
            return True, index_
        point2 = path[i+1].copy()
    return True, index_


def check_two_points_l(receive, point1, point2, barrierId, color='blue', id=0, dis_threshold=20):
    # import ipdb;ipdb.set_trace()
    infos = receive.get_infos(color, id)
    for index in range(len(infos)):
        center = infos[index]
        dx_1 = center[0] - point1[0]
        dy_1 = center[1] - point1[1]
        dx_2 = center[0] - point2[0]
        dy_2 = center[1] - point2[1]
        dx_0 = point1[0] - point2[0]
        dy_0 = point1[1] - point2[1]
        mul_1 = (dx_1) * (-dx_0) + (dy_1) * (-dy_0)
        mul_2 = (dx_2) * (dx_0) + (dy_2) * (dy_0)
        if mul_1 > 0 and mul_2 > 0:
            mid = abs((dx_1) * (-dy_0) - (-dx_0) * (dy_1))
            dist = mid / np.sqrt(dx_0 * dx_0 + dy_0 * dy_0)
        elif mul_1 == 0 and mul_2 != 0:
            dist = np.sqrt(dx_1 * dx_1 + dy_1 * dy_1)
        elif mul_1 != 0 and mul_2 == 0:
            dist = np.sqrt(dx_2 * dx_2 + dy_2 * dy_2)
        elif mul_1 == 0 and mul_2 == 0:
            dist = 0
        elif mul_1 < 0 and mul_2 > 0:
            dist = np.sqrt(dx_1 * dx_1 + dy_1 * dy_1)
        elif mul_2 < 0 and mul_1 > 0:
            dist = np.sqrt(dx_2 * dx_2 + dy_2 * dy_2)
        else:
            dist = 0

        if dist < dis_threshold:
            return False
    return True


def check_two_points(receive, point1, point2, barrierId, color, id, dis_threshold=30):
    info = receive.get_infos(color, id)
    select_points = interpolate_point(point1, point2)
    delta = select_points[np.newaxis, ...] - info[:, np.newaxis, :]
    dis = np.sqrt(np.sum(delta * delta, axis=2))
    if (dis < dis_threshold).sum():
        return False
    else:
        return True





