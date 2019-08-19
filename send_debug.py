import socket
import proto.zss_debug_pb2 as debug_info
from time import sleep
import numpy as np


class SendDebug():
    def __init__(self, type, lines, color='YELLOW'):
        '''
        type: 'LINE', 'ARC', 'TEXT', 'ROBOT', 'CURVE', 'POLYGON', 'POINTS'
        lines: a list of line, and the line in lines contain [start_x, start_y, end_x, end_y]
        color: 'WHITE', 'RED', 'ORANGE', 'YELLOW', 'GREEN', 'CYAN', 'BLUE', 'PURPLE', 'GRAY', "BLACK'
        '''
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.address = ('127.0.0.1', 50001)
        self.debug_type = type
        self.color = color
        self.num = len(lines)
        self.lines = lines
        # import ipdb;ipdb.set_trace()
        self.debug_msg = {'start_x': 0, 'start_y': 0, 'end_x': 100, 'end_y': 100}

    def send(self):
        package = debug_info.Debug_Msgs()
        # msg = package.msgs.add()
        msg = []
        for i in range(self.num):
            msg.append(package.msgs.add())

            color = self.color
            # choose a color
            if color == 'WHITE':
                msg[i].color = debug_info.Debug_Msg.WHITE
            elif color == 'RED':
                msg[i].color = debug_info.Debug_Msg.RED
            elif color == 'ORANGE':
                msg[i].color = debug_info.Debug_Msg.ORANGE
            elif color == 'YELLOW':
                msg[i].color = debug_info.Debug_Msg.YELLOW
            elif color == 'GREEN':
                msg[i].color = debug_info.Debug_Msg.GREEN
            elif color == 'CYAN':
                msg[i].color = debug_info.Debug_Msg.CYAN
            elif color == 'BLUE':
                msg[i].color = debug_info.Debug_Msg.BLUE
            elif color == 'PURPLE':
                msg[i].color = debug_info.Debug_Msg.PURPLE
            elif color == 'GRAY':
                msg[i].color = debug_info.Debug_Msg.GRAY
            elif color == 'BLACK':
                msg[i].color = debug_info.Debug_Msg.BLACK

            # choose a debug type
            if self.debug_type == 'LINE':
                msg[i].type = debug_info.Debug_Msg.LINE
                line = msg[i].line
                line.FORWARD = True
                line.BACK = False
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
