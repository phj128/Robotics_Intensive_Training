from message.send import Send
from message.receive import Receive
import time
from utils import distance
import math

def runLine(color, robot_id, start_x, start_y, goal_x, goal_y, vmax = 150, threshold=0.3):
    receive = Receive()
    send = Send()

    receive.get_info(color, robot_id)
    now_x = receive.robot_info['x']
    now_y = receive.robot_info['y']
    point = [now_x, now_y]
    now_ori = receive.robot_info['ori']
    error = distance(point, [goal_x, goal_y])
    error_max = distance([start_x, start_y], [goal_x, goal_y])

    while error > 10:
        p = 1
        orientation_need_now = math.atan2((goal_y - now_y), (goal_x - now_x))
        theta = now_ori - orientation_need_now
        if error < error_max * threshold:
            p = error / (threshold * error_max) * math.log(2)
            p = math.exp(p) - 1
        vx_now = vmax * math.cos(theta) * p
        vy_now = vmax * math.sin(theta) * p
        send.send_msg(robot_id, vx_now, vy_now, 0)
        receive.get_info(color, robot_id)
        now_x = receive.robot_info['x']
        now_y = receive.robot_info['y']
        now_ori = receive.robot_info['ori']
        point = [now_x, now_y]
        error = distance(point, [goal_x, goal_y])

if __name__ == "__main__":
    robot_id = 0
    color = 'blue'
    origin = [-250, 180]
    end = [250, -180]
    i = 0
    while True:
        start_time = time.time()
        if i % 2 == 0:
            runLine(color, robot_id, origin[0], origin[1], end[0], end[1])
        else:
            runLine(color, robot_id, end[0], end[1], origin[0], origin[1])
        i += 1
        end_time = time.time()
        print('a circle time:', end_time - start_time)
