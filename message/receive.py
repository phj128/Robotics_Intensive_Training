import socket
from time import sleep
import proto.vision_detection_pb2 as detection
import numpy as np
from math import sqrt, atan2


class Receive():
    def __init__(self):
        self.robot_info = {'x': 0, 'y': 0, 'ori': 0, 'vx': 0, 'vy': 0, 'w': 0, 'ax': 0, 'ay': 0,
                           'r_x': 0, 'r_y': 0, 'r_ori': 0, 'r_vx': 0, 'r_vy': 0, 'r_w': 0}
        self.infos = []
        self.velocity_infos = []

    def get_velocity_infos(self):
        pass

    def convert_info(self, robot):
        self.robot_info['x'] = robot.x / 10
        self.robot_info['y'] = robot.y / 10
        self.robot_info['ori'] = robot.orientation
        self.robot_info['vx'] = robot.vel_x
        self.robot_info['vy'] = robot.vel_y
        self.robot_info['w'] = robot.rotate_vel
        self.robot_info['ax'] = robot.accelerate_x
        self.robot_info['ay'] = robot.accelerate_y
        self.robot_info['r_x'] = robot.raw_x
        self.robot_info['r_y'] = robot.raw_y
        self.robot_info['r_ori'] = robot.raw_orientation
        self.robot_info['r_vx'] = robot.raw_vel_x
        self.robot_info['r_vy'] = robot.raw_vel_y
        self.robot_info['r_w'] = robot.raw_rotate_vel


    def change_info_yellow_velocity(self, robot, color, id):
        if color == 'yellow' and id == robot.robot_id:
            return
        self.infos.append([robot.vel_x, robot.vel_y])


    def change_info_blue_velocity(self, robot, color, id):
        if color == 'blue' and id == robot.robot_id:
            return
        self.infos.append([robot.vel_x, robot.vel_y])

    def change_info_yellow(self, robot, color, id):
        if color == 'yellow' and id == robot.robot_id:
            return
        self.infos.append([robot.x / 10, robot.y / 10, sqrt(
            (robot.vel_x / 10) * (robot.vel_x / 10) + (robot.vel_y / 10) * (robot.vel_y / 10)), color, robot.robot_id])


    def change_info_blue(self, robot, color, id):
        if color == 'blue' and id == robot.robot_id:
            return
        self.infos.append([robot.x/10, robot.y/10, sqrt((robot.vel_x/10)*(robot.vel_x/10)+(robot.vel_y/10)*(robot.vel_y/10)), color, robot.robot_id])


    def get_velocity_info(self, color, id):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("127.0.0.1", 23333))
        data, address = self.sock.recvfrom(4096)
        package = detection.Vision_DetectionFrame()
        package.ParseFromString(data)
        robots_yellow = package.robots_yellow
        robots_blue = package.robots_blue

        if color == 'yellow':
            for robot in robots_yellow:
                if robot.robot_id == id:
                    self.convert_info(robot)
                    break
        else:
            for robot in robots_blue:
                if robot.robot_id == id:
                    self.convert_info(robot)
                    break

    def get_info(self, color, id):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("127.0.0.1", 23333))
        data, address = self.sock.recvfrom(4096)

        package = detection.Vision_DetectionFrame()
        package.ParseFromString(data)

        robots_yellow = package.robots_yellow
        robots_blue = package.robots_blue
        # import ipdb;ipdb.set_trace()

        if color == 'yellow':
            for robot in robots_yellow:
                if robot.robot_id == id:
                    self.convert_info(robot)
                    break
        else:
            for robot in robots_blue:
                if robot.robot_id == id:
                    self.convert_info(robot)
                    break



    def get_infos(self, color, id):
        self.infos = []
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("127.0.0.1", 23333))
        data, address = self.sock.recvfrom(4096)

        package = detection.Vision_DetectionFrame()
        package.ParseFromString(data)

        robots_yellow = package.robots_yellow
        robots_blue = package.robots_blue
        for robot in robots_yellow:
            self.change_info_yellow(robot, color, id)
        for robot in robots_blue:
            self.change_info_blue(robot, color, id)
        return self.infos


    def thread_change_info_yellow(self, robot, infos=None):
        # infos.append([robot.x / 10, robot.y / 10, 'yellow', robot.robot_id, robot.orientation, robot.vel_x/10, robot.vel_y/10])
        v = sqrt((robot.vel_x/10)*(robot.vel_x/10) + (robot.vel_y/10)*(robot.vel_y/10))
        a = sqrt((robot.accelerate_x)*(robot.accelerate_x) + (robot.accelerate_y)*(robot.accelerate_y))
        theta = atan2(robot.vel_y, robot.vel_x)
        # infos.append([robot.x / 10, robot.y / 10, 'yellow', robot.robot_id, robot.orientation, robot.vel_x / 10, robot.vel_y / 10, robot.accelerate_x, robot.accelerate_y])
        return [robot.x / 10, robot.y / 10, 'yellow', robot.robot_id, robot.orientation, v, a, theta]


    def thread_change_info_blue(self, robot, infos=None):
        # infos.append([robot.x / 10, robot.y / 10, 'blue', robot.robot_id, robot.orientation, robot.vel_x/10, robot.vel_y/10])
        v = sqrt((robot.vel_x / 10) * (robot.vel_x / 10) + (robot.vel_y / 10) * (robot.vel_y / 10))
        a = sqrt((robot.accelerate_x/10) * (robot.accelerate_x/10) + (robot.accelerate_y/10) * (robot.accelerate_y/10))
        theta = atan2(robot.vel_y, robot.vel_x)
        return [robot.x / 10, robot.y / 10, 'blue', robot.robot_id, robot.orientation, v, a, theta]


    def thread_infos(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("127.0.0.1", 23333))
        data, address = self.sock.recvfrom(4096)

        package = detection.Vision_DetectionFrame()
        package.ParseFromString(data)

        robots_yellow = package.robots_yellow
        robots_blue = package.robots_blue
        infos_blue = [self.thread_change_info_blue(robot) for robot in robots_blue]
        infos_yellow = [self.thread_change_info_yellow(robot) for robot in robots_yellow]
        # for robot in robots_blue:
        #     infos = self.thread_change_info_blue(robot, infos)
        # for robot in robots_yellow:
        #     infos = self.thread_change_info_yellow(robot, infos)
        # return infos
        return infos_blue + infos_yellow

if __name__ == "__main__":
    receive = Receive()
    while True:
        receive.get_info('blue', 0)
        # import ipdb;ipdb.set_trace()
        print(receive.robot_info['ori'])
        sleep(1)

