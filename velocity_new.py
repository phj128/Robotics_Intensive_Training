from send import Send
from time import sleep
from receive import Receive
import math
import numpy as np
from myRRT_static import RRT
from myRRTstar import RRT as RRT_STAR
from send_debug import SendDebug
import time


time_start = time.time()
receive = Receive()
receive.get_info('blue', 0)
my_rrt = RRT_STAR(receive.robot_info['x'], receive.robot_info['y'], 200, 200, receive,
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
tree = path
#一开始车头朝向右边，x正方向，所以v_x对应x_path, v_y对应y_path
#print(tree)
#print(len(tree))
T = 3
ta = 0.5 #固定加速时间
tn = T-2*(0.5*ta + 0.5*ta) #固定匀速运动时间，2.5秒
x_path = [] #n个
y_path = [] #n个
v_x = [0.0]
v_y = [0.0]

for i in range(len(tree)):
    x_path.append(tree[i][0])
    y_path.append(tree[i][1])

#得到匀速运动段的速度
for i in range(len(x_path)-1):
    v_i_iand1 = (x_path[i+1]-x_path[i])/T
    v_x.append(v_i_iand1)
v_x.append(0.0) #得到n+1个速度，首尾两个0
for i in range(len(y_path)-1):
    v_i_iand1 = (y_path[i+1]-y_path[i])/T
    v_y.append(v_i_iand1)
v_y.append(0.0) #得到n+1个速度，首尾两个0

send = Send()
robot_id = 0

for i in range(len(tree)):
    #加速阶段
    send.send_msg(robot_id, 0.75 * v_x[i] + 0.25 * v_x[i + 1], 0.75 * v_y[i] + 0.25 * v_y[i + 1], 0)
    sleep(ta/4)
    send.send_msg(robot_id, 0.5 * v_x[i] + 0.5 * v_x[i + 1], 0.5 * v_y[i] + 0.5 * v_y[i + 1], 0)
    sleep(ta / 4)
    send.send_msg(robot_id, 0.25 * v_x[i] + 0.75 * v_x[i + 1], 0.25 * v_y[i] + 0.75 * v_y[i + 1], 0)
    sleep(ta / 4)
    send.send_msg(robot_id, v_x[i + 1], v_y[i + 1], 0)
    sleep(ta / 4)
    #直线运动阶段
    send.send_msg(robot_id, v_x[i + 1], v_y[i + 1], 0)
    sleep(tn)


'''
now = time.time()
print('command get cost: ', now-end)
print(v_x)
print(len(v_x))
print(t_x_acclerate)
print(len(t_x_acclerate))
print(t_x_normal)
print(len(t_x_normal))

'''
