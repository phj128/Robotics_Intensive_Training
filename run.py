from message.send import Send
from message.send_debug import SendDebug
from message.receive import Receive

from global_planner.myRRT_static import RRT
from global_planner.myRRTstar import RRT as RRT_STAR
from global_planner.myRRTmerge import RRT as RRT_MERGE

from ArtificialPotentialFieldMethod.myAPF import APF

from local_planner.p_control import P_control
from local_planner.xy_control import XY_control

from utils import distance

from time import sleep
import math
import numpy as npi
import time


def run(color, robot_id, barriers, target_x, target_y, global_p, local_p):
    global_planner = global_p
    local_planner = local_p

    time_start = time.time()
    receive = Receive()
    receive.get_info(color, robot_id)
    global_path = global_planner(receive.robot_info['x'], receive.robot_info['y'], target_x, target_y, barriers, receive)
    status, tree, lines = global_path.Generate_Path()
    path, path_lines = global_path.Get_Path()
    print('ori:', len(path))
    path, path_lines = global_path.merge()
    print('nodes:', len(path))
    time_end = time.time()
    print('path cost:', time_end - time_start)
    debug_info = SendDebug('LINE', [lines, path_lines])
    debug_info.send()
    end = time.time()
    print('total cost:', end - time_start)

    motion = local_planner()
    motion.path_control(path, robot_id, color, receive)
    control = Send()
    control.send_msg(robot_id, 0, 0, 0)


def run_while(color, robot_id, barriers, target_x, target_y,  global_p, local_p):
    global_planner = global_p
    local_planner = local_p

    receive = Receive()
    while True:
        time_start = time.time()
        receive.get_info(color, robot_id)
        global_path = global_planner(receive.robot_info['x'], receive.robot_info['y'], target_x, target_y, barriers, receive)
        status, tree, lines = global_path.Generate_Path()
        path, path_lines = global_path.Get_Path()
        print('ori:', len(path))
        path, path_lines = global_path.merge()
        print('nodes:', len(path))
        time_end = time.time()
        print('path cost:', time_end - time_start)
        debug_info = SendDebug('LINE', [lines, path_lines])
        debug_info.send()
        end = time.time()
        print('total cost:', end - time_start)
        if len(path) < 2:
            point = path[0]
        else:
            point = path[1]
        receive.get_info(color, robot_id)
        now_x = receive.robot_info['x']
        now_y = receive.robot_info['y']
        # import ipdb;ipdb.set_trace()
        if distance((now_x, now_y), (target_x, target_y)) > 10:
            motion = local_planner()
            motion.point_control(point, robot_id, color, receive)
            # control = Send()
            # control.send_msg(robot_id, 0, 0, 0)
        else:
            control = Send()
            control.send_msg(robot_id, 0, 0, 0)
            return



if __name__ == '__main__':
    color = 'blue'
    robot_id = 0
    barriers = [['yellow', 0], ['yellow', 1], ['yellow', 2], ['yellow', 3],
                ['yellow', 4], ['yellow', 5], ['yellow', 6], ['yellow', 7]]

    # barriers = [['yellow', 0], ['blue', 1], ['blue', 2], ['blue', 3],
    #             ['blue', 4], ['blue', 5], ['blue', 6], ['blue', 7]]

    i = 0
    global_p = RRT_MERGE
    local_p = XY_control
    RUN = run_while

    while True:
        if i % 2 == 0:
            RUN(color, robot_id, barriers, 200, 200, global_p, local_p)
        else:
            RUN(color, robot_id, barriers, -200, -200, global_p, local_p)
        i += 1
