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
        self.threshold = 0.6
        self.time_turn = 0.3
        self.angle_threshold = 5 * PI / 6
        self.up = 60


    def line_control(self, now_x, now_y, now_ori, path, i, N, target_x, target_y, infos=None, k1=10, k2=10, v_obstacle_max=400, rr=100, color="blue", robot_id=0):
        point_now = [now_x, now_y]
        error = distance(point_now, path[i+1])
        error_max = distance(path[i], path[i+1])
        if error_max < 80:
            error_max = 80
        orientation_need_now = math.atan2((path[i + 1][1] - now_y), (path[i + 1][0] - now_x))
        theta = now_ori - orientation_need_now
        p = 1
        dis_now = distance(path[i], path[i+1])
        if distance(point_now, [target_x, target_y]) < 80:
            p = 0.2
        else:
            if error > 7:
                if dis_now < self.up:
                    p = sigmoid(error/dis_now-1)
                if error < error_max * self.threshold:
                    if i < N - 2:
                        alpha = math.atan2(path[i + 2][1] - path[i + 1][1], path[i + 2][0] - path[i + 1][0])
                        if orientation_need_now > 0:
                            angle = alpha + (PI - orientation_need_now)
                        else:
                            angle = alpha - (PI + orientation_need_now)
                        if angle > PI:
                            angle = 2 * PI - angle
                        if abs(angle) < self.angle_threshold:
                            p = error / ((self.threshold + self.time_turn) * error_max) * math.log(2)
                            p = math.exp(p) - 1
                        else:
                            p = error / (self.threshold * error_max) * math.log(2)
                            p = math.exp(p) - 1
                    else:
                        p = error / (self.threshold * error_max) * math.log(2)
                        p = math.exp(p) - 1
            else:
                p = 0.2
                vx_now = self.v * math.cos(theta) * p
                vy_now = self.v * math.sin(theta) * p
                return vx_now, vy_now, True

        #速度斥力部分
        vx = 0.0
        vy = 0.0
        for index in range(len(infos)):
            if infos[index][3] == robot_id and infos[index][2] == color:
                continue
            elif distance(infos[index][:2], point_now) < rr:
                k = (k1 * infos[index][5] / v_obstacle_max) + k2 * rr / distance(infos[index][:2], point_now)
                gamma = math.atan2(infos[index][1] - now_y, infos[index][0] - now_x)
                vx = vx + k * math.cos(gamma)
                vy = vy + k * math.sin(gamma)
            else:
                continue

        vx_now = (self.v * math.cos(theta)-vx) * p
        vy_now = (self.v * math.sin(theta)-vy) * p
        return vx_now, vy_now, False