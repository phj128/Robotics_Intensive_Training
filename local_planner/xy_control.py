from message.send import Send
from message.send_debug import SendDebug
from time import sleep
import math
import numpy as np
import time
from utils import distance, interpolate_path, check_two_points


class XY_control():
    def __init__(self):
        self.send = Send()
        self.debug = SendDebug()
        self.v = 300


    def path_control(self, path, robot_id, color, receive):
        #一开始车头朝向右边，x正方向，所以v_x对应x_path, v_y对应y_path
        # T = 0.05
        # ta = T / 10  # 固定加速时间
        # tn = T - 2 * (0.5 * ta + 0.5 * ta)  # 固定匀速运动时间，2.5秒
        # x_path = np.array(path)[:, 0]
        # y_path = np.array(path)[:, 1]
        # v_x = np.concatenate(([0.0], (x_path[1:] - x_path[:-1]) / T, [0.0]))
        # v_y = np.concatenate(([0.0], (y_path[1:] - y_path[:-1]) / T, [0.0]))
        # path = interpolate_path(path)
        #得到匀速运动段的速度
        for i in range(len(path)-1):
            # #加速阶段
            # self.send.send_msg(robot_id, 0.75 * v_x[i] + 0.25 * v_x[i + 1], 0.75 * v_y[i] + 0.25 * v_y[i + 1], 0)
            # sleep(ta/4)
            # self.send.send_msg(robot_id, 0.5 * v_x[i] + 0.5 * v_x[i + 1], 0.5 * v_y[i] + 0.5 * v_y[i + 1], 0)
            # sleep(ta / 4)
            # self.send.send_msg(robot_id, 0.25 * v_x[i] + 0.75 * v_x[i + 1], 0.25 * v_y[i] + 0.75 * v_y[i + 1], 0)
            # sleep(ta / 4)
            # self.send.send_msg(robot_id, v_x[i + 1], v_y[i + 1], 0)
            # sleep(ta / 4)
            #
            # #直线运动阶段
            # self.send.send_msg(robot_id, v_x[i + 1], v_y[i + 1], 0)
            # sleep(tn)

            # 闭环检测部分
            receive.get_info(color, robot_id)
            now_x = receive.robot_info['x']
            now_y = receive.robot_info['y']
            now_ori = receive.robot_info['ori']
            error = np.sqrt(np.square(now_x - path[i + 1][0]) + np.square(now_y - path[i + 1][1]))
            print('error:', error)
            while error > 10:
                orientation_need_now = math.atan2((path[i + 1][1] - now_y), (path[i + 1][0] - now_x))
                theta = now_ori + orientation_need_now
                vx_now = self.v * math.cos(theta)
                vy_now = self.v * math.sin(theta)
                self.send.send_msg(robot_id, vx_now, vy_now, 0)
                receive.get_info(color, robot_id)
                now_x = receive.robot_info['x']
                now_y = receive.robot_info['y']
                now_ori = receive.robot_info['ori']
                error = np.sqrt(np.square(now_x - path[i + 1][0]) + np.square(now_y - path[i + 1][1]))
                # self.send.send_msg(robot_id, v_x[i+1], v_y[i+1], 0)
                # sleep(T/100)
            i += 1


    def point_control(self, point, robot_id, color, receive, info=None):
        receive.get_info(color, robot_id)
        now_x = receive.robot_info['x']
        now_y = receive.robot_info['y']
        now_ori = receive.robot_info['ori']
        point_now = [now_x, now_y]
        error = distance(point_now, point)
        print('error:', error)
        while error > 10:
            orientation_need_now = math.atan2((point[1] - now_y), (point[0] - now_x))
            theta = now_ori + orientation_need_now
            vx_now = self.v * math.cos(theta)
            vy_now = self.v * math.sin(theta)
            self.send.send_msg(robot_id, vx_now, vy_now, 0)
            receive.get_info(color, robot_id)
            now_x = receive.robot_info['x']
            now_y = receive.robot_info['y']
            now_ori = receive.robot_info['ori']
            point_now = [now_x, now_y]
            error = distance(point_now, point)
            print('error:', error)


    def line_control(self, point, robot_id, color, receive, info=None):
        receive.get_info(color, robot_id)
        now_x = receive.robot_info['x']
        now_y = receive.robot_info['y']
        now_ori = receive.robot_info['ori']
        point_now = [now_x, now_y]
        error = distance(point_now, point)
        print('error:', error)
        while error > 10:
            orientation_need_now = math.atan2((point[1] - now_y), (point[0] - now_x))
            theta = now_ori + orientation_need_now
            vx_now = self.v * math.cos(theta)
            vy_now = self.v * math.sin(theta)
            self.send.send_msg(robot_id, vx_now, vy_now, 0)
            receive.get_info(color, robot_id)
            now_x = receive.robot_info['x']
            now_y = receive.robot_info['y']
            now_ori = receive.robot_info['ori']
            point_now = [now_x, now_y]
            error = distance(point_now, point)
            print('error:', error)
            if info is not None:
                start = time.time()
                status = check_two_points(receive, point_now, point, info)
                end = time.time()
                print("time:", end - start)
                if not status:
                    return



