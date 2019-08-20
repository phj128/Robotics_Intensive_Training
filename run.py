from message.send import Send
from message.send_debug import SendDebug
from message.receive import Receive

from global_planner.myRRT_static import RRT
from global_planner.myRRTstar import RRT as RRT_STAR
from ArtificialPotentialFieldMethod.myAPF import APF

from local_planner.p_control import P_control
from local_planner.xy_control import XY_control

from time import sleep
import math
import numpy as np
import time


if __name__ == '__main__':
    color = 'blue'
    robot_id = 0
    barriers = [['yellow', 0], ['yellow', 1], ['yellow', 2], ['yellow', 3],
                ['yellow', 4], ['yellow', 5], ['yellow', 6], ['yellow', 7]]

    global_planner = RRT_STAR
    local_planner = XY_control

    time_start = time.time()
    receive = Receive()
    receive.get_info(color, robot_id)
    global_path = global_planner(receive.robot_info['x'], receive.robot_info['y'], 200, 200, barriers, receive)
    status, tree, lines = global_path.Generate_Path()
    path, path_lines = global_path.Get_Path()
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
