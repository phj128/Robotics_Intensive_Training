from message.send import Send
from message.send_debug import SendDebug
from time import sleep
import math
import numpy as np
import time
from utils import distance, interpolate_path, check_two_points_l, check_path_l

PI = math.pi

class XY_PD():
    def __init__(self):
        self.send = Send()
        self.debug = SendDebug()
        self.v = 200
        self.threshold = 0.4
        self.time_turn = 0.3
        self.angle_threshold = 5 * PI / 6
        self.obstacle_distance_threshold = 50


    def line_control(self, path, robot_id, color, receive, info=None, threshold=30, index=1):
        N = len(path)
        for i in range(N - 1):
            receive.get_info(color, robot_id)
            now_x = receive.robot_info['x']
            now_y = receive.robot_info['y']
            now_ori = receive.robot_info['ori']
            point_now = [now_x, now_y]
            error = distance(point_now, path[i+1])
            error_max = distance(point_now, path[i+1])

            now_vx = receive.robot_info['vx']
            now_vy = receive.robot_info['vy']

            print('error:', error)
            while error > 30:
                orientation_need_now = math.atan2((path[i+1][1] - now_y), (path[i+1][0] - now_x))
                theta = now_ori - orientation_need_now
                vx_change = 0.0
                vy_change = 0.0
                dist_punish = 0.0
                for barrier in info:
                    # barrier为单个的list，有两个元素，前一个为字符串，后一个为数字
                    receive.get_info(barrier[0], barrier[1])
                    ob_x = receive.robot_info['x']
                    ob_y = receive.robot_info['y']
                    ob_distance = distance([ob_x, ob_y], [now_x, now_y])
                    ob_ori = receive.robot_info['ori']
                    ob_vx = receive.robot_info['vx']
                    ob_vy = receive.robot_info['vy']
                    if ob_vx == 0 and ob_vy == 0:
                        continue

                    if ob_distance < self.obstacle_distance_threshold:
                        alpha = now_ori - ob_ori
                        vx_change = vx_change + 1.5*ob_vx*math.cos(alpha) - 1.5*ob_vy*math.cos(alpha-PI/2)
                        vy_change = vy_change + 1.5*ob_vy*math.sin(alpha-PI/2) - 1.5*ob_vx*math.sin(alpha)
                        dist_punish = dist_punish + 10/(1+ob_distance)

                vx_now = vx_change + dist_punish
                vy_now = vy_change + dist_punish
                p = 1
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
                vx_now = vx_now + self.v * math.cos(theta) * p
                vy_now = vy_now + self.v * math.sin(theta) * p
                self.send.send_msg(robot_id, vx_now, vy_now, 0)
                receive.get_info(color, robot_id)
                now_x = receive.robot_info['x']
                now_y = receive.robot_info['y']
                now_ori = receive.robot_info['ori']
                point_now = [now_x, now_y]
                error = distance(point_now, path[i+1])
                # print('error:', error)
                if info is not None:
                    start = time.time()
                    status, index = check_path_l(receive, point_now, path[i+1:], info, color=color, id=robot_id,
                                          dis_threshold=threshold, index_=index)
                    # import ipdb;ipdb.set_trace()
                    end = time.time()
                    # print("time:", end - start)
                    if not status:
                        # print(status)
                        return False, index
        return True, index
