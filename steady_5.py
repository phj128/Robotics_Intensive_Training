from message.send import Send
from message.send_debug import SendDebug
from message.receive import Receive

from global_planner.myRRTmerge import RRT as RRT_MERGE

from local_planner.xy_speed import XY_speed
from local_planner.XY_speed_force import XY_speed as XY_speed_force
from local_planner.XY_speed_force_optimization import XY_speed as XY_speed_force_optimization
from local_planner.X_ori_control import X_ori_speed

from utils import distance, interpolate_path
import numpy as np
from numpy import sqrt
import time
from main import func

def count_path_thread(point, target, infos, color='blue', id=0):
    point1 = point.copy()
    point2 = target.copy()
    road = 0.0
    for info in infos:
        center = info[:2]

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
            dist = mid / sqrt(dx_0 * dx_0 + dy_0 * dy_0)
        elif mul_1 == 0 and mul_2 != 0:
            dist = sqrt(dx_1 * dx_1 + dy_1 * dy_1)
        elif mul_1 != 0 and mul_2 == 0:
            dist = sqrt(dx_2 * dx_2 + dy_2 * dy_2)
        elif mul_1 == 0 and mul_2 == 0:
            dist = 0
        elif mul_1 < 0 and mul_2 > 0:
            dist = sqrt(dx_1 * dx_1 + dy_1 * dy_1)
        elif mul_2 < 0 and mul_1 > 0:
            dist = sqrt(dx_2 * dx_2 + dy_2 * dy_2)
        else:
            dist = 0

        road = road+dist
    return road

def mark(path, barriers, receive, color, robot_id):
    barrierInfo = Update_Barrier_Info(color, robot_id, receive)
    score = 0.0
    for i in range(len(path)-1):
        score = score + count_path_thread(path[i], path[i+1], barrierInfo, color=color, id=robot_id)
        print(score)
    return score


def Update_Barrier_Info(color, robot_id, receive):
    barrierInfo = receive.get_infos(color, robot_id)
    return barrierInfo


def RUN_mark(color, robot_id, barriers, target_x, target_y,  global_p, local_p, receive):
    global_planner = global_p
    local_planner = local_p
    R = 25
    index = 1
    r = R / index
    receive.get_info(color, robot_id)

    score = 0
    path_last = []
    lines_last = []
    path_lines_last = []
    score_list = []
    path_list = []
    flag = 0
    s = time.time()
    for index in range(20):
        global_path = global_planner(target_x, target_y, -target_x, -target_y, barriers,
                                     # receive, color=color, id=robot_id)
                                     receive, color=color, id=robot_id, inflateRadius=R, dis_threshold=R)
        status, tree, lines = global_path.Generate_Path()
        if status:
            flag += 1
        path, path_lines = global_path.Get_Path()
        path, path_lines = global_path.merge()
        score_now = mark(path, barriers, receive, color, robot_id)
        score_list.append(score_now)
        path_list.append(path)
        lines_last.append(lines)
        path_lines_last.append(path_lines)
    score_list = np.array(score_list)
    path_list = np.array(path_list)
    ind = np.argpartition(score_list, -5)[-5:]
    # import ipdb; ipdb.set_trace()
    path_select = path_list[ind]
    path_select = [interpolate_path(path_s) for path_s in path_select]
    lines_last = np.array(lines_last)[ind]
    path_lines_last = np.array(path_lines_last)[ind]
    e = time.time()
    print('cost:', e-s)
    print(flag)
    if flag == 0:
        func(color, robot_id, receive)
        return


    global_path = global_planner(receive.robot_info['x'], receive.robot_info['y'], target_x, target_y, barriers,
                                 # receive, color=color, id=robot_id)
                                 receive, color=color, id=robot_id, inflateRadius=R, dis_threshold=R)
    status, tree, lines = global_path.Generate_Path()
    path, path_lines = global_path.Get_Path()
    path, path_lines = global_path.merge()
    path_start = path
    debug_info = SendDebug('LINE', [lines, path_lines])
    debug_info.send()
    motion = local_planner()
    motion.line_control(path_start, robot_id, color, receive, target_x, target_y, barriers, r,
                        index=index)


    index = 1
    num = 0
    r = R / index
    while True:
        path_last = path_select[num]
        debug_info = SendDebug('LINE', [lines_last[num], path_lines_last[num]])
        debug_info.send()
        receive.get_info(color, robot_id)
        motion = local_planner()
        motion.line_control(path_last, robot_id, color, receive, target_x, target_y, barriers, r,
                                            index=index)
        target_x *= -1
        target_y *= -1
        path_last = path_last[::-1]
        receive.get_info(color, robot_id)
        motion = local_planner()
        motion.line_control(path_last, robot_id, color, receive, target_x, target_y, barriers, r,
                            index=index)
        num += 1
        if num == 5:
            num = 0


if __name__ == "__main__":
    color = 'blue'
    robot_id = 4
    # barriers = [['yellow', 0], ['yellow', 1], ['yellow', 2], ['yellow', 3],
    #             ['yellow', 4], ['yellow', 5], ['yellow', 6], ['yellow', 7],
    #             ['blue', 1], ['blue', 2], ['blue', 5], ['blue', 3],
    #             ['blue', 0], ['blue', 6], ['blue', 7]]
    g_x, g_y = (-250, 150)
    barriers = [['yellow', 0], ['blue', 1], ['blue', 2], ['blue', 3],
                ['blue', 4], ['blue', 5], ['blue', 6], ['blue', 7]]

    i = 0
    global_p = RRT_MERGE
    local_p = X_ori_speed

    receive = Receive()

    while True:
        RUN_mark(color, robot_id, barriers, g_x, g_y, global_p, local_p, receive)

