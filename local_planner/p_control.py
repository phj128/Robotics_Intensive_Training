from message.send import Send
from message.send_debug import SendDebug
from time import sleep
import math
import numpy as np
import time


class P_control():
    def __init__(self):
        self.send = Send()
        self.debug = SendDebug()


    def path_control(self, path, robot_id, color, receive):
        for i in range(len(path) - 1):
            receive.get_info(color, robot_id)
            now_x = receive.robot_info['x']
            now_y = receive.robot_info['y']
            now_ori = receive.robot_info['ori']
            error = np.sqrt(np.square(now_x - path[i + 1][0]) + np.square(now_y - path[i + 1][1]))
            if error > 10:
                step = 0
                orientation_need_now = -math.atan2((path[i + 1][1] - now_y), (path[i + 1][0] - now_x))
                radians_now = now_ori - orientation_need_now
                while abs(radians_now) > 0.1:
                    orientation_need_now = -math.atan2((path[i + 1][1] - now_y), (path[i + 1][0] - now_x))
                    r_v = radians_now
                    if abs(r_v) < 1:
                        r_v /= abs(r_v)
                    if abs(r_v) < 3:
                        r_v /= abs(r_v)
                        r_v *= 3
                    self.send.send_msg(robot_id, 0, 0, r_v)
                    s = time.time()
                    sleep(0.1)
                    e = time.time()
                    receive.get_info(color, robot_id)
                    now_ori = receive.robot_info['ori']
                    radians_now = now_ori - orientation_need_now
                while error > 10 or step < 10:
                    v_x = error * 2
                    if abs(v_x) > 50:
                        v_x = 50
                    self.send.send_msg(robot_id, v_x, 0, 0)
                    sleep(0.1)
                    receive.get_info(color, robot_id)
                    now_x = receive.robot_info['x']
                    now_y = receive.robot_info['y']
                    error = np.sqrt(np.square(now_x - path[i + 1][0]) + np.square(now_y - path[i + 1][1]))
                    step += 1


    def point_control(self, point, robot_id, color, receive):
        receive.get_info(color, robot_id)
        now_x = receive.robot_info['x']
        now_y = receive.robot_info['y']
        now_ori = receive.robot_info['ori']
        error = np.sqrt(np.square(now_x - point[0]) + np.square(now_y - point[1]))
        if error > 10:
            step = 0
            orientation_need_now = -math.atan2((point[1] - now_y), (point[0] - now_x))
            radians_now = now_ori - orientation_need_now
            while abs(radians_now) > 0.1:
                orientation_need_now = -math.atan2((point[1] - now_y), (point[0] - now_x))
                r_v = radians_now
                self.send.send_msg(robot_id, 0, 0, r_v)
                s = time.time()
                sleep(0.1)
                e = time.time()
                receive.get_info('blue', robot_id)
                now_ori = receive.robot_info['ori']
                radians_now = now_ori - orientation_need_now
            while error > 10 or step < 10:
                v_x = error
                self.send.send_msg(robot_id, v_x, 0, 0)
                sleep(0.1)
                receive.get_info('blue', robot_id)
                now_x = receive.robot_info['x']
                now_y = receive.robot_info['y']
                error = np.sqrt(np.square(now_x - point[0]) + np.square(now_y - point[1]))
                step += 1


