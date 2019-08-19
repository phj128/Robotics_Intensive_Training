
from send import Send
from time import sleep
from receive import Receive
import math
import numpy as np
tree =
count = 0
rotate_velocity = 40 #1 radians / second
dt = 5
robot_id = 0
for point in tree:
    count = count+1

send = Send()
receive = Receive()
for i in range(count):
    receive.get_info('blue',robot_id)
    x_dist = tree[count + 1][0] - tree[count][0]
    y_dist = tree[count + 1][1] - tree[count][1]
    y_dist = -y_dist # for real
    orientation = receive.robot_info['ori']
    orientation_need = math.atan2(y_dist, x_dist)
    radians = orientation_need-orientation
    rotate_time = radians/rotate_velocity
    if radians > 0:
        rotate_velocity = -abs(rotate_velocity)
    else:
        rotate_velocity = abs(rotate_velocity)
    send.send_msg(robot_id,0,0,rotate_velocity)
    sleep(rotate_time)

    dist = np.sqrt(np.square(x_dist)+np.square(y_dist))
    v_x = dist/dt
    send.send_msg(robot_id,v_x,0,0)
    sleep(dt)
    send.send_msg(robot_id,0,0,0)

    receive.get_info('blue',robot_id)
    now_x = receive.robot_info['x']
    now_y = receive.robot_info['y']
    now_ori = receive.robot_info['ori']
    error = np.sqrt(np.square(now_x-tree[count+1][0])+np.square(now_y-tree[count+1][1]))

    if error > 3:
        orientation_need_now = math.atan2((tree[count+1][1]-now_y),(tree[count+1][0]-now_x))
        radians_now = orientation_need_now - now_ori
        rotate_time_now = radians_now/rotate_velocity
        if radians_now > 0:
            rotate_velocity = -abs(rotate_velocity)
        else:
            rotate_velocity = abs(rotate_velocity)
        send.send_msg(robot_id, 0, 0, rotate_velocity)
        sleep(rotate_time_now)


