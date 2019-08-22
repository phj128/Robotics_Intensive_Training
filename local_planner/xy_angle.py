from message.send import Send
from message.send_debug import SendDebug
from time import sleep
import math
import numpy as np
import time
from utils import distance, interpolate_path, check_two_points_l, check_path_l


PI = 3.1415926


class XY_angle():
    def __init__(self):
        self.send = Send()
        self.debug = SendDebug()
        self.v = 200
        self.threshold = 0.4
        self.time_turn = 0.3
        self.angle_threshold = 5 * PI / 6


    def path_control(self, path, robot_id, color, receive):
        N = len(path)
        for i in range(N-1):
            # 闭环检测部分
            receive.get_info(color, robot_id)
            now_x = receive.robot_info['x']
            now_y = receive.robot_info['y']
            point = [now_x, now_y]
            now_ori = receive.robot_info['ori']
            error = distance(point, path[i+1])
            error_max = distance(path[i], path[i+1])
            # print('error:', error)
            while error > 10:
                p = 1
                orientation_need_now = math.atan2((path[i + 1][1] - now_y), (path[i + 1][0] - now_x))
                theta = now_ori + orientation_need_now
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
                vx_now = self.v * math.cos(theta) * p
                vy_now = self.v * math.sin(theta) * p
                self.send.send_msg(robot_id, vx_now, vy_now, 0)
                receive.get_info(color, robot_id)
                now_x = receive.robot_info['x']
                now_y = receive.robot_info['y']
                now_ori = receive.robot_info['ori']
                point = [now_x, now_y]
                error = distance(point, path[i+1])
                # print('error:', error)
                # self.send.send_msg(robot_id, v_x[i+1], v_y[i+1], 0)
                # sleep(T/100)



    def point_control(self, point, robot_id, color, receive):
        receive.get_info(color, robot_id)
        now_x = receive.robot_info['x']
        now_y = receive.robot_info['y']
        now_ori = receive.robot_info['ori']
        error = np.sqrt(np.square(now_x - point[0]) + np.square(now_y - point[1]))
        print('error:', error)
        index = 0
        while error > 10:
            orientation_need_now = math.atan2((point[1] - now_y), (point[0] - now_x))
            theta = now_ori + orientation_need_now
            if error < 20:
                v = 100
            vx_now = self.v * math.cos(theta)
            vy_now = self.v * math.sin(theta)
            self.send.send_msg(robot_id, vx_now, vy_now, 0)
            receive.get_info(color, robot_id)
            now_x = receive.robot_info['x']
            now_y = receive.robot_info['y']
            now_ori = receive.robot_info['ori']
            error = np.sqrt(np.square(now_x - point[0]) + np.square(now_y - point[1]))
            index += 1


    def line_control(self, path, robot_id, color, receive, info=None):
        N = len(path)
        for i in range(N - 1):
            receive.get_info(color, robot_id)
            now_x = receive.robot_info['x']
            now_y = receive.robot_info['y']
            now_ori = receive.robot_info['ori']
            point_now = [now_x, now_y]
            error = distance(point_now, path[i+1])
            error_max = distance(point_now, path[i+1])
            print('error:', error)
            while error > 30:
                orientation_need_now = math.atan2((path[i+1][1] - now_y), (path[i+1][0] - now_x))
                theta = now_ori - orientation_need_now
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
                vx_now = self.v * math.cos(theta) * p
                vy_now = self.v * math.sin(theta) * p
                self.send.send_msg(robot_id, vx_now, vy_now, 0)
                receive.get_info(color, robot_id)
                now_x = receive.robot_info['x']
                now_y = receive.robot_info['y']
                now_ori = receive.robot_info['ori']
                point_now = [now_x, now_y]
                error = distance(point_now, path[i+1])
                print('error:', error)
                if info is not None:
                    start = time.time()
                    status = check_path_l(receive, point_now, path[i+1:], info, color=color, id=robot_id)
                    # import ipdb;ipdb.set_trace()
                    end = time.time()
                    print("time:", end - start)
                    if not status:
                        print(status)
                        return False
        return True
