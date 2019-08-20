from message.send import Send
from message.send_debug import SendDebug
from message.receive import Receive

from global_planner.myRRT_static import RRT
from global_planner.myRRTstar import RRT as RRT_STAR

from local_planner.p_control import P_control

from ArtificialPotentialFieldMethod.myAPF import APF
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
    local_planner = P_control

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






# time_start = time.time()
# receive = Receive()
# receive.get_info('blue', 0)
# my_rrt = RRT_STAR(receive.robot_info['x'], receive.robot_info['y'], 200, 200, receive,
#               [['yellow', 0], ['yellow', 1], ['yellow', 2], ['yellow', 3],
#                ['yellow', 4], ['yellow', 5], ['yellow', 6], ['yellow', 7]])
# status, tree1, lines = my_rrt.Generate_Path()
# path, path_lines = my_rrt.Get_Path()
# time_end = time.time()
# print('path cost:', time_end - time_start)
# send_tree = SendDebug('LINE', [lines, path_lines])
# send_tree.send()
# end = time.time()
# print('total cost:', end - time_start)
#
#
# robot_id = 0
#
# receive.get_info('blue', robot_id)
# now_x = receive.robot_info['x']
# now_y = receive.robot_info['y']
# tree = path
# # import ipdb; ipdb.set_trace()
# # tree = [[now_x, now_y], [now_x, now_y + 10]]
# send = Send()
# print(len(tree))
# rt = []
# vt = []
# zt = []
# for i in range(len(tree)-1):
#     receive.get_info('blue', robot_id)
#     now_x = receive.robot_info['x']
#     now_y = receive.robot_info['y']
#     now_ori = receive.robot_info['ori']
#     error = np.sqrt(np.square(now_x - tree[i+1][0])+np.square(now_y - tree[i+1][1]))
#     print(now_ori)
#
#     if error > 10:
#         step = 0
#         orientation_need_now = -math.atan2((tree[i + 1][1] - now_y), (tree[i + 1][0] - now_x))
#         radians_now = now_ori - orientation_need_now
#         zt.append(now_ori)
#         while abs(radians_now) > 0.1:
#             orientation_need_now = -math.atan2((tree[i + 1][1] - now_y), (tree[i + 1][0] - now_x))
#             r_v = radians_now
#             print(r_v)
#         # import ipdb;ipdb.set_trace()
#             send.send_msg(robot_id, 0, 0, r_v)
#             s = time.time()
#             sleep(0.1)
#             e = time.time()
#             print('time', e-s)
#             receive.get_info('blue', robot_id)
#             now_ori = receive.robot_info['ori']
#             radians_now = now_ori - orientation_need_now
#         while error > 10 or step < 10:
#             v_x = error
#             send.send_msg(robot_id, v_x, 0, 0)
#             sleep(0.1)
#             receive.get_info('blue', robot_id)
#             now_x = receive.robot_info['x']
#             now_y = receive.robot_info['y']
#             error = np.sqrt(np.square(now_x - tree[i + 1][0]) + np.square(now_y - tree[i + 1][1]))
#             step += 1
#
#
#     print(i)
# import ipdb;ipdb.set_trace()