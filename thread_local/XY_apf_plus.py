from message.send import Send
from message.send_debug import SendDebug
from time import sleep
from math import sin, cos, atan2, atan
import numpy as np
import time
from utils import distance, interpolate_path, check_two_points_l, check_path_l, sigmoid

PI = 3.1415926


class XY_speed():
    def __init__(self):
        self.send = Send()
        self.debug = SendDebug()
        self.v = 250
        self.threshold = 0.25
        self.time_turn = 0.3
        self.angle_threshold = 5 * PI / 6
        self.up = 60
        self.rtt_distance = 60
        self.wallthreshold = 30
        self.wall_k = 2400
        self.w = 300
        self.h = 200
        self.d_k = 360000
        self.v_k = 0.7
        self.time_threshold = 1


    def line_control(self, now_x, now_y, now_ori, path, ind, N, target_x, target_y, infos=None, color='blue',
                     robot_id=4, threshold=30, index=1, start_time=None, shock_time=None):
        point_now = [now_x, now_y]
        barriers = infos.copy()
        my_info = []
        for i in range(len(barriers)):
            if barriers[i][2]==color and barriers[i][3]==robot_id:
                my_info = barriers[i].copy()
                barriers.pop(i)
                break

        error = distance(point_now, [target_x, target_y])
        error_max = distance([-target_x, -target_y], [target_x, target_y])
        vx_rtt = 0.0 # 机器人x方向受到斥力整合的速度，与速度有关
        vy_rtt = 0.0 # 机器人y方向受到斥力整合的速度，与速度有关
        # vx_rdt = 0.0 # 机器人x方向受到斥力整合的速度，与距离有关
        # vy_rdt = 0.0 # 机器人x方向受到斥力整合的速度，与距离有关
        vx_wall = 0.0 #机器人x方向受墙整合的速度
        dx = 0.0
        vy_wall = 0.0 #机器人y方向受墙整合的速度
        dy = 0.0
        vx_att = 0.0
        vy_att = 0.0

        for barrier in barriers:
            d = distance(point_now, [barrier[0], barrier[1]])
            if d < self.rtt_distance:
                alpha = atan2(barrier[1]-now_y, barrier[0]-now_x)
                v_jingxiang = barrier[5]*cos(PI-barrier[7]+alpha)
                v_qiexiang = barrier[5]*sin(PI-barrier[7]+alpha)
                vx_rtt = vx_rtt + self.v_k * (v_jingxiang * cos(PI - now_ori + alpha) - v_qiexiang * cos(alpha - (PI / 2) - now_ori))
                vy_rtt = vy_rtt + self.v_k * (v_jingxiang * sin(PI - now_ori + alpha) - v_qiexiang * sin(alpha - (PI / 2) - now_ori))
                vx_rtt = vx_rtt - self.d_k*cos(now_ori-alpha)/(d*d)
                vy_rtt = vy_rtt - self.d_k*sin(now_ori-alpha)/(d*d)
        # print(vx_rtt,vy_rtt)
        # return vx_rtt, vy_rtt, False

        if abs(now_x - self.w) <= self.wallthreshold:
            dx = -abs(self.wall_k / (now_x - self.w))
        if abs(now_x+self.w) <= self.wallthreshold:
            dx = abs(self.wall_k / (self.w + now_x))
        if abs(now_y - self.h) <= self.wallthreshold:
            dy = abs(self.wall_k / (self.h - now_y)) # y是正的时候是向右，这里我目前以为靠近上边沿读进的y是正的
        if abs(now_y + self.h) <= self.wallthreshold:
            dy = -abs(self.wall_k / (self.h + now_y))
        vx_wall = dx*cos(now_ori) - dy*sin(now_ori)
        vy_wall = dy*cos(now_ori) - dx*sin(now_ori)

        # return vx_wall, vy_wall, False
        orientation_need_now = atan2((target_y - now_y), (target_x - now_x))
        theta = now_ori - orientation_need_now
        vx_att = self.v*cos(theta)/(error)
        vy_att = self.v*sin(theta)/(error)

        dis = distance(point_now, [target_x, target_y])
        end_time = time.time()
        if dis > 7:
            if dis > 60 and my_info[5] < 20 and (end_time - start_time) > self.time_threshold and (end_time - shock_time) > 3:
                lamda = atan(3/5)+now_ori
                sign = np.random.rand()
                random_v = np.random.randint(300, 400)
                if sign >= 0.5:
                    random_v = -random_v
                return random_v*sin(lamda), -random_v*cos(lamda), True, end_time
                # return vx_rtt + random_v*cos(lamda), vy_rtt - random_v*sin(lamda), True
                # return vx_rtt + np.random.randint(-100, 100), vy_rtt + np.random.randint(-100, 100), True
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

                    vx_now = (self.v * cos(theta) + vx_att + vx_wall + vx_rtt)*p
                    vy_now = (self.v * sin(theta) + vy_att + vy_wall + vy_rtt)*p

                    return vx_now, vy_now, False, start_time
                else:
                    p = 0.4
                    vx_now = (self.v * cos(theta) + vx_att + vx_wall + vx_rtt)*p
                    vy_now = (self.v * sin(theta) + vy_att + vy_wall + vy_rtt)*p
                    return vx_now, vy_now, False, start_time
            else:
                p = 0.4
                vx_now = (self.v * cos(theta) + vx_att + vx_wall + vx_rtt)*p
                vy_now = (self.v * sin(theta) + vy_att + vy_wall + vy_rtt)*p
                return vx_now, vy_now, False, start_time

        else:
            p = 0.05
            vx_now = (self.v * cos(theta) + vx_att + vx_wall + vx_rtt)*p
            vy_now = (self.v * sin(theta) + vy_att + vy_wall + vy_rtt)*p
            return vx_now, vy_now, False, start_time
