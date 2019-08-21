from message.send import Send
from message.send_debug import SendDebug
from time import sleep
import math
import numpy as np
import time
from utils import distance, interpolate_path


class XY_p():
    def __init__(self):
        self.send = Send()
        self.debug = SendDebug()
        self.v = 300
        self.threshold = 0.5


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
            point = [now_x, now_y]
            now_ori = receive.robot_info['ori']
            error = distance(point, path[i+1])
            error_max = distance(path[i], path[i+1])
            print('error:', error)
            while error > 10:
                orientation_need_now = math.atan2((path[i + 1][1] - now_y), (path[i + 1][0] - now_x))
                theta = now_ori + orientation_need_now
                # p = error/error_max
                # if p < self.threshold:
                #     p = self.threshold
                p = 1
                if error < error_max * self.threshold:
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
                print(error)
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
