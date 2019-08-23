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


def select_info(infos, color, robot_id):
    for info in infos:
        if info[3] == robot_id:
            if info[2] == color:
                return info[:5]
    return -1, -1, -1, -1, -100


def min_dis_index(point, path, i):
    path = np.array(path)
    delta = point - path
    dis = np.sqrt(np.sum(delta * delta, axis=1))
    return np.argmin(dis) + i


def make_vel(target, infos, vxs, vys, v=400):
    for i in range(len(target)):
        now_x, now_y, _, _, now_ori = infos[i][:5]
        orientation_need_now = math.atan2((target[i][1] - now_y), (target[i][0] - now_x))
        theta = now_ori - orientation_need_now
        vxs[i+1] = v * math.cos(theta)
        vys[i+1] = v * math.sin(theta)
    return vxs, vys


def check_goals(targets, infos, threshold=30):
    for i in range(len(targets)):
        if distance(infos[i][:2], targets[i][:2]) < threshold:
            targets[i][1] = -targets[i][1]
    return targets


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


def check_path_thread(point, target, infos, dis_threshold=20, index=1, color='blue', id=0):
    point1 = point.copy()
    point2 = target.copy()
    for i in range(len(infos)):
        if infos[i][3] == id:
            if infos[i][2] == color:
                continue
        center = infos[i][:2]
        if distance(point1, center) < dis_threshold:
            return False, index+1
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
            return False, index
    return True, index





