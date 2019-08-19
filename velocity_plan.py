
from send import Send
from time import sleep
from receive import Receive
import math
import numpy as np
from myRRT_static import RRT
from send_debug import SendDebug
import time


time_start = time.time()
receive = Receive()
receive.get_info('blue', 0)
my_rrt = RRT(receive.robot_info['x'], receive.robot_info['y'], 200, 200, receive,
              [['yellow', 0], ['yellow', 1], ['yellow', 2], ['yellow', 3],
               ['yellow', 4], ['yellow', 5], ['yellow', 6], ['yellow', 7]])
status, tree1, lines = my_rrt.Generate_Path()
path, path_lines = my_rrt.Get_Path()
time_end = time.time()
print('path cost:', time_end - time_start)
send_tree = SendDebug('LINE', [lines, path_lines])
send_tree.send()
end = time.time()
print('total cost:', end - time_start)

rotate_velocity_ = 1 #1 radians / second
#dt = 1
v_x = 100
robot_id = 0

receive.get_info('blue', robot_id)
now_x = receive.robot_info['x']
now_y = receive.robot_info['y']
tree = path
import ipdb; ipdb.set_trace()
# tree = [[now_x, now_y], [now_x, now_y + 10]]
send = Send()
print(len(tree))
rt = []
vt = []
zt = []
for i in range(len(tree)-1):
    # receive.get_info('blue', robot_id)
    # x_dist = tree[i+1][0] - tree[i][0]
    # y_dist = tree[i+1][1] - tree[i][1]
    # # y_dist = -y_dist # for real
    # orientation = receive.robot_info['ori']
    # orientation_need = math.atan2(y_dist, x_dist)
    # radians = orientation-orientation_need
    # rotate_time = radians/rotate_velocity
    # if radians > 0:
    #     rotate_velocity = abs(rotate_velocity)
    # else:
    #     rotate_velocity = -abs(rotate_velocity)
    #
    # dist = np.sqrt(np.square(x_dist) + np.square(y_dist))
    # # v_x = dist/dt
    # dt = dist / v_x
    #
    # send.send_msg(robot_id,0,0,rotate_velocity)
    # sleep(abs(rotate_time))
    # send.send_msg(robot_id,v_x,0,0)
    # sleep(dt)
    # send.send_msg(robot_id,0,0,0)

    receive.get_info('blue', robot_id)
    now_x = receive.robot_info['x']
    now_y = receive.robot_info['y']
    now_ori = receive.robot_info['ori']
    error = np.sqrt(np.square(now_x - tree[i+1][0])+np.square(now_y - tree[i+1][1]))
    print(now_ori)

    if error > 10:
        step = 0
        orientation_need_now = -math.atan2((tree[i + 1][1] - now_y), (tree[i + 1][0] - now_x))
        radians_now = now_ori - orientation_need_now
        zt.append(now_ori)
        while abs(radians_now) > 0.1:
            orientation_need_now = -math.atan2((tree[i + 1][1] - now_y), (tree[i + 1][0] - now_x))
            r_v = abs(radians_now)
            print(r_v)
        # import ipdb;ipdb.set_trace()
            if radians_now < 0:
                r_v = r_v
            send.send_msg(robot_id, 0, 0, r_v)
            s = time.time()
            sleep(0.1)
            e = time.time()
            print('time', e-s)
            receive.get_info('blue', robot_id)
            now_ori = receive.robot_info['ori']
            radians_now = now_ori - orientation_need_now
        while error > 10 or step < 10:
            v_x = error
            send.send_msg(robot_id, v_x, 0, 0)
            sleep(0.1)
            receive.get_info('blue', robot_id)
            now_x = receive.robot_info['x']
            now_y = receive.robot_info['y']
            error = np.sqrt(np.square(now_x - tree[i + 1][0]) + np.square(now_y - tree[i + 1][1]))
            step += 1


    print(i)
import ipdb;ipdb.set_trace()

