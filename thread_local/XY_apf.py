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
        self.v = 500
        self.threshold = 0.3
        self.time_turn = 0.3
        self.angle_threshold = 5 * PI / 6
        self.up = 60
        self.rtt_distance = 40
        self.wallthreshold = 30


    def line_control(self, now_x, now_y, now_ori, path, i, N, target_x, target_y, infos=None, color='blue', robot_id=4, threshold=30, index=1):
        point_now = [now_x, now_y]
        if i >= len(path)-1:
            return 0, 0, False
        barriers = infos.copy()
        for i in range(len(barriers)):
            if barriers[i][2]==color and barriers[i][3]==robot_id:
                barriers.pop(i)
                break

        error = distance(point_now, path[i+1])
        error_max = distance(path[i], path[i+1])
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
                v = barrier[5]*cos(barrier[7]-alpha)
                vx_rtt = vx_rtt + v*cos(now_ori-alpha)/10 + 400*cos(now_ori-alpha)/(d*d)
                vy_rtt = vy_rtt + v*sin(now_ori-alpha)/10 + 400*sin(now_ori-alpha)/(d*d)

        if abs(now_x-300) <= self.wallthreshold:
            dx = 800/(now_x-300)
        if abs(now_x+300) <= self.wallthreshold:
            dx = 800/(300+now_x)
        if abs(now_y - 225) <= self.wallthreshold:
            dy = 800/(225-now_y) # y是正的时候是向右，这里我目前以为靠近上边沿读进的y是正的
        if abs(now_y + 225) <= self.wallthreshold:
            dy = 800/(225+now_y)
        vx_wall = dx*cos(now_ori) + dy*sin(now_ori)
        vy_wall = -dy*cos(now_ori) + dx*sin(now_ori)
        print(vx_wall, vy_wall)

        return vx_wall, vy_wall, False
        orientation_need_now = atan2((path[i + 1][1] - now_y), (path[i + 1][0] - now_x))
        theta = now_ori - orientation_need_now
        vx_att = self.v*cos(theta)/(1+error)
        vy_att = self.v*sin(theta)/(1+error)


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

                    vx_now = (self.v * cos(theta) + vx_att + vx_wall + vx_rtt)*p
                    vy_now = (self.v * sin(theta) + vy_att + vy_wall + vy_rtt)*p

                    return vx_now, vy_now, False
                else:
                    p = 0.2
                    vx_now = (self.v * cos(theta) + vx_att + vx_wall + vx_rtt)*p
                    vy_now = (self.v * sin(theta) + vy_att + vy_wall + vy_rtt)*p
                    return vx_now, vy_now, True
            else:
                p = 0.2
                vx_now = (self.v * cos(theta) + vx_att + vx_wall + vx_rtt)*p
                vy_now = (self.v * sin(theta) + vy_att + vy_wall + vy_rtt)*p
                return vx_now, vy_now, True
        else:
            p = 0.05
            vx_now = (self.v * cos(theta) + vx_att + vx_wall + vx_rtt)*p
            vy_now = (self.v * sin(theta) + vy_att + vy_wall + vy_rtt)*p
            return vx_now, vy_now, True
