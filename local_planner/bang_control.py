from message.send import Send
from message.send_debug import SendDebug
from time import sleep
import math
import numpy as np

class Bang_Control():
    def __init__(self):
        self.send = Send()
        self.debug = SendDebug()

    #def path_control(self, path, robot_id, color, receive):

    def point_control(self, point, robot_id, color, receive):
        '''
        此时机器人有本身的速度
        :param point: 下一个目标点
        :param robot_id: 
        :param color: 
        :param receive: 
        :return: 
        '''
        receive.get_info(robot_id, color)
        now_x = receive.robot_info['x']
        now_y = receive.robot_info['y']
        now_ori = receive.robot_info['ori']
        now_vx = receive.robot_info['vx']
        now_vy = receive.robot_info['vy']
        now_w = receive.robot_info['w']
        now_ax = receive.robot_info['ax']
        now_ay = receive.robot_info['ay']

        error = np.sqrt(np.square(now_x - point[0]) + np.square(now_y - point[1]))

        exist_obstacle_dist = [0]
        exist_obstacle_ax = [0]
        exist_obstacle_ay = [0]
        obstacle_color = 'yellow'
        for i in range(8):
            receive.get_info(i, obstacle_color)
            dist = np.sqrt(np.square(receive.robot_info['x'] - point[0]) + np.square(receive.robot_info['y'] - point[1]))
            if dist < 30:
                exist_obstacle_dist.append(dist)
                exist_obstacle_ax.append((100.0 * (point[0]-receive.robot_info['x'])) / (dist ** 2))
                exist_obstacle_ay.append((100.0 * (point[1]-receive.robot_info['y'])) / (dist ** 2))
            else:
                pass
        print('error:', error)
        ax = sum(exist_obstacle_ax)
        ay = sum(exist_obstacle_ay)
        index = 0
        print(ax,ay,now_vx,now_vy)
        while error > 10 or index < 10:
            orientation_need_now = math.atan2((point[1] - now_y), (point[0] - now_x))
            theta = now_ori + orientation_need_now
            vx_need = 150 * math.cos(theta)
            vy_need = 150 * math.sin(theta)
            self.send.send_msg(robot_id, now_vx+ax, now_vy+ay, 0)
            sleep(0.01)
            for i in range(5):
                self.send.send_msg(robot_id, 0.2*i*vx_need+0.2*(5-i)*now_vx, 0.2*i*vx_need+0.2*(5-i)*now_vy, 0)
                sleep(0.002)
            self.send.send_msg(robot_id, vx_need, vy_need, 0)
            sleep(0.01)
            receive.get_info(color, robot_id)
            now_x = receive.robot_info['x']
            now_y = receive.robot_info['y']
            now_ori = receive.robot_info['ori']
            error = np.sqrt(np.square(now_x - point[0]) + np.square(now_y - point[1]))
            index += 1




