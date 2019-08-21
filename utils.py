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


def check_two_points(receive, point1, point2, barrierId):
    dis_threshold = 30
    for index in range(len(barrierId)):
        receive.get_info(barrierId[index][0], barrierId[index][1])
        center = [receive.robot_info['x'], receive.robot_info['y']]
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


