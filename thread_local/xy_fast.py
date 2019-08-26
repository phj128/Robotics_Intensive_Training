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
        self.v = 300
        self.p = 0.3


    def line_control(self, now_x, now_y, now_ori, path, i, N, target_x, target_y, infos=None, k1=10, k2=10, v_obstacle_max=400, rr=100, color="blue", robot_id=0):
        point_now = [now_x, now_y]
        error = distance(point_now, path[i+1])
        orientation_need_now = math.atan2((path[i + 1][1] - now_y), (path[i + 1][0] - now_x))
        theta = now_ori - orientation_need_now
        if distance(point_now, [target_x, target_y]) > 30:
            thres = 20
            if i == N-2:
                thres = 7
            if error > thres:
                vx_now = self.v * math.cos(theta)
                vy_now = self.v * math.sin(theta)
                return vx_now, vy_now, False
            else:
                return 0, 0, True
        else:
            if error > 7:
                return self.v * math.cos(theta) * self.p, self.v * math.sin(theta) * self.p, False
            else:
                return 0, 0, True