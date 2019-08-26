from message.send import Send
from message.send_debug import SendDebug
from time import sleep
import math
import numpy as np
import time
from utils import distance, interpolate_path, check_two_points_l, check_path_l, sigmoid

PI = 3.1415926


class XY_speed():
    def __init__(self):
        self.send = Send()
        self.debug = SendDebug()
        self.v = 350
        self.threshold = 0.4
        self.time_turn = 0.3
        self.angle_threshold = 5 * PI / 6
        self.up = 60

    # def line_control(self, path, robot_id, color, receive, target_x, target_y, info=None, threshold=30, index=1):
    def line_control(self, now_x, now_y, now_ori, path, i, N, target_x, target_y, infos=None, color='blue', robot_id=4, threshold=30, index=1):
        point_now = [now_x, now_y]
        error = distance(point_now, path[i + 1])
        error_max = distance(path[i], path[i + 1])
        orientation_need_now = math.atan2((path[i + 1][1] - now_y), (path[i + 1][0] - now_x))
        theta = now_ori - orientation_need_now
        if distance(point_now, [target_x, target_y]) > 7:
            thres = 20
            if i == N - 2:
                thres = 7
            if error > thres:
                p = 1
                dis_now = distance(path[i], path[i + 1])
                if dis_now < self.up:
                    p = 0.5
                thresdist = error_max * self.threshold
                if 3 * thresdist > error > thresdist:
                    p = 0.6 * p
                if error < thresdist:
                    p = p * error / thresdist

                vx_now = self.v * math.cos(theta) * p
                vy_now = self.v * math.sin(theta) * p

                return vx_now, vy_now, False
            else:
                p = 0.2
                vx_now = self.v * math.cos(theta) * p
                vy_now = self.v * math.sin(theta) * p
                return vx_now, vy_now, True