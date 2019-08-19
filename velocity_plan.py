
from send import Send
from time import sleep
from receive import Receive
import math
tree =
count = 0
rotate_velocity = 40 #1 radians / second
dt = 0.1
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

    vx = x_dist/dt
    vy = y_dist/dt
    send.send_msg(robot_id,vx,vy,0)
    sleep(dt)
