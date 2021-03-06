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
        self.v = 300
        self.threshold = 0.3
        self.time_turn = 0.3
        self.angle_threshold = 5 * PI / 6
        self.up = 60


    def line_control(self, now_x, now_y, now_ori, path, i, N, target_x, target_y, infos=None, color='blue', robot_id=4, threshold=30, index=1):
        point_now = [now_x, now_y]
        if i >= len(path) - 1:
            return 0, 0, False
        error = distance(point_now, path[i + 1])
        error_max = distance(path[i], path[i + 1])
        orientation_need_now = atan2((path[i + 1][1] - now_y), (path[i + 1][0] - now_x))
        theta = now_ori - orientation_need_now
        dis = distance(point_now, [target_x, target_y])
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
