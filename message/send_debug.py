import socket
import proto.zss_debug_pb2 as debug_info
from time import sleep
import numpy as np
import math


PI = 3.1415926


def filter_infos(infos, robot_id, color):
    infos_ = infos.copy()
    for i in range(len(infos)):
        if infos[i][3] == robot_id:
            if infos[i][2] == color:
                infos_.pop(i)
                break
    return infos_


class SendDebug():
    def __init__(self, type='LINE', lines=[], color='YELLOW', circles=[], infos=[]):
        '''
        type: 'LINE', 'ARC', 'TEXT', 'ROBOT', 'CURVE', 'POLYGON', 'POINTS'
        lines: lists of line, and the line in lines contain [start_x, start_y, end_x, end_y]
        color: 'WHITE', 'RED', 'ORANGE', 'YELLOW', 'GREEN', 'CYAN', 'BLUE', 'PURPLE', 'GRAY', "BLACK'
        '''
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.address = ('127.0.0.1', 20001)
        self.debug_type = type
        self.color = color
        if len(lines) == 2:
            self.num = len(lines[0]) + len(lines[1])
            self.num_yellow = len(lines[0])
            self.lines = lines[0] + lines[1]
        else:
            self.num = len(lines)
            self.num_yellow = self.num
            self.lines = lines
        # import ipdb;ipdb.set_trace()
        self.circles = circles
        self.infos = filter_infos(infos, 5, 'blue')
        self.draw_circles = []
        self.debug_msg = {'start_x': 0, 'start_y': 0, 'end_x': 100, 'end_y': 100}

    def send(self):
        package = debug_info.Debug_Msgs()
        # msg = package.msgs.add()
        msg = []
        for i in range(self.num):
            msg.append(package.msgs.add())
            if i < self.num_yellow:
                msg[i].color = debug_info.Debug_Msg.YELLOW
            else:
                msg[i].color = debug_info.Debug_Msg.RED
            # color = self.color
            # # choose a color
            # if color == 'WHITE':
            #     msg[i].color = debug_info.Debug_Msg.WHITE
            # elif color == 'RED':
            #     msg[i].color = debug_info.Debug_Msg.RED
            # elif color == 'ORANGE':
            #     msg[i].color = debug_info.Debug_Msg.ORANGE
            # elif color == 'YELLOW':
            #     msg[i].color = debug_info.Debug_Msg.YELLOW
            # elif color == 'GREEN':
            #     msg[i].color = debug_info.Debug_Msg.GREEN
            # elif color == 'CYAN':
            #     msg[i].color = debug_info.Debug_Msg.CYAN
            # elif color == 'BLUE':
            #     msg[i].color = debug_info.Debug_Msg.BLUE
            # elif color == 'PURPLE':
            #     msg[i].color = debug_info.Debug_Msg.PURPLE
            # elif color == 'GRAY':
            #     msg[i].color = debug_info.Debug_Msg.GRAY
            # elif color == 'BLACK':
            #     msg[i].color = debug_info.Debug_Msg.BLACK

            # choose a debug type
            if self.debug_type == 'LINE':
                msg[i].type = debug_info.Debug_Msg.LINE
                line = msg[i].line
                line.FORWARD = True
                line.BACK = False
                # import ipdb;ipdb.set_trace()
                line.start.x = self.lines[i][0]
                line.start.y = self.lines[i][1]
                line.end.x = self.lines[i][2]
                line.end.y = self.lines[i][3]
            elif self.debug_type == 'ARC':
                msg.type = debug_info.Debug_Msg.ARC
                arc = msg.arc
                arc.start = 10
                arc.end = 100
                arc.FILL = True
                arc.rectangle.point1.x = 0
                arc.rectangle.point1.y = 0
                arc.rectangle.point2.x = 100
                arc.rectangle.point2.y = 100
            elif self.debug_type == 'TEXT':
                msg.type = debug_info.Debug_Msg.TEXT
                text = msg.text
                text.text = 'helloworld'
                text.pos.x = 0
                text.pos.y = 100
            elif self.debug_type == 'ROBOT':
                msg.type = debug_info.Debug_Msg.ROBOT
                robot = msg.robot
                robot.pos.x = 200
                robot.pos.y = 200
                robot.dir = 10
            elif self.debug_type == 'CURVE':
                msg.type = debug_info.Debug_Msg.CURVE
                curve = msg.curve
                curve.num = 1
                curve.maxLimit = 1000
                curve.minLimit = 990
            elif self.debug_type == 'POLYGON':
                msg.type = debug_info.Debug_Msg.POLYGON
                polygon = msg.polygon
                polygon.FILL = True
                for i in range(3):
                    point = polygon.vertex.add()
                    if i == 0:
                        point.x = 0
                        point.y = 0
                    elif i == 1:
                        point.x = 20
                        point.y = 20
                    else:
                        point.x = 10
                        point.y = 30
            elif self.debug_type == 'POINTS':
                msg.type = debug_info.Debug_Msg.Points
                points = msg.points
                for i in range(8):
                    point = points.point.add()
                    point.x = 30*i
                    point.y = 50*i
            else:
                print('No this kind of instruction')
                return -1
        for i in range(len(self.circles)):
            # theta = PI / 3
            self.draw_circles.append(package.msgs.add())
            self.draw_circles[i].color = debug_info.Debug_Msg.GREEN
            self.draw_circles[i].type = debug_info.Debug_Msg.ARC
            arc = self.draw_circles[i].arc
            radius = self.circles[i]
            arc.rectangle.point1.x = self.infos[i][0] - radius
            arc.rectangle.point1.y = self.infos[i][1] + radius
            arc.rectangle.point2.x = self.infos[i][0] + radius
            arc.rectangle.point2.y = self.infos[i][1] - radius

            arc.start = 0
            arc.end = 360
            arc.FILL = 1

            # for k in range(6):
            #     self.draw_circles.append(package.msgs.add())
            #     self.draw_circles[6*i+k].color = debug_info.Debug_Msg.GREEN
            #     self.draw_circles[6*i+k].type = debug_info.Debug_Msg.LINE
            #     line = self.draw_circles[6*i+k].line
            #     line.FORWARD = True
            #     line.BACK = False
            #     line.start.x = self.infos[i][0] + self.circles[i] * math.cos(theta*k)
            #     line.start.y = self.infos[i][1] + self.circles[i] * math.sin(theta*k)
            #     line.end.x = self.infos[i][0] + self.circles[i] * math.cos(theta*(k+1))
            #     line.end.y = self.infos[i][1] + self.circles[i] * math.sin(theta*(k+1))

        send_data = package.SerializeToString()
        self.sock.sendto(send_data, ("127.0.0.1", 20001))


if __name__ == '__main__':
    type = 'LINE'
    num = 2
    points = [0, 0, 10, 10]
    lines = []
    lines.append(points)
    for i in range(10):
        points = points.copy()
        points[0] = points[2]
        points[1] = points[3]
        points[2] += np.random.randint(5, 20)
        points[3] += np.random.randint(10, 20)
        lines.append(points)
    send_debug = SendDebug(type, lines)
    send_debug.send()
    sleep(1)
