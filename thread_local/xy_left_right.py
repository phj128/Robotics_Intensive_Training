from message.send import Send
from message.send_debug import SendDebug
from time import sleep
from math import sin, cos, atan2
import numpy as np
import time
from utils import distance, interpolate_path, check_two_points_l, check_path_l, sigmoid

PI = 3.1415926


class XY_speed():
    def __init__(self):
        self.send = Send()
        self.debug = SendDebug()
        self.v = 50
        self.threshold = 0.3
        self.time_turn = 0.3
        self.angle_threshold = 5 * PI / 6
        self.up = 60


    def line_control(self, now_x, now_y, now_ori, path, i, N, target_x, target_y, infos=None, color='blue', robot_id=4, threshold=30, index=1):
        point_now = [now_x, now_y]
        error = distance(point_now, path[i + 1])
        error_max = distance(path[i], path[i + 1])
        orientation_need_now = atan2((path[i + 1][1] - now_y), (path[i + 1][0] - now_x))
        theta = now_ori - orientation_need_now
        dis = distance(point_now, [target_x, target_y])
        barriers = infos.copy()
        M = len(infos)
        for y in range(M):
            if barriers[y][2] == color and barriers[y][3] == robot_id:
                barriers.pop(y)
                break
        for t in range(M):
            if distance(point_now, [barriers[t][0], barriers[t][1]]) < 40:
                print('here')
                return 45*cos(now_ori)+60*sin(now_ori), 45*sin(now_ori)-60*cos(now_ori), False
        if dis > 7:
            if dis > 60:
                if error > 20:
                    p = 1
                    dis_now = distance(path[i], path[i + 1])
                    if dis_now < self.up:
                        p = 0.5
                    thresdist = error_max * self.threshold
                    if 3 * thresdist > error > thresdist:
                        p = 0.6 * p
                    if error < thresdist:
                        p = p * error / thresdist

                    vx_now = self.v * cos(theta) * p
                    vy_now = self.v * sin(theta) * p

                    return vx_now, vy_now, False
                else:
                    p = 0.2
                    vx_now = self.v * cos(theta) * p
                    vy_now = self.v * sin(theta) * p
                    return vx_now, vy_now, True
            else:
                p = 0.2
                vx_now = self.v * cos(theta) * p
                vy_now = self.v * sin(theta) * p
                return vx_now, vy_now, True
        else:
            p = 0.05
            vx_now = self.v * cos(theta) * p
            vy_now = self.v * sin(theta) * p
            return vx_now, vy_now, True
