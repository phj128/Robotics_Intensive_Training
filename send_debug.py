import socket
import proto.zss_debug_pb2 as debug_info
from time import sleep

class SendDebug():
    def __init__(self):
        self.sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
        self.address = ('127.0.0.1',50001)
        self.debug_type = 'LINE'
        self.debug_msg = {'start_x':0, 'start_y':0, 'end_x':10, 'end_y':10,
                          }

    def set_point(self):
        package = debug_info.Debug_Msgs()
        msg = package.msgs.add()
        msg.color = debug_info.Debug_Msg.YELLOW
        if self.debug_type=='LINE':
            msg.type = debug_info.Debug_Msg.LINE
            line = msg.line
            line.FORWARD = True
            line.BACK = False
            line.start.x = self.debug_msg['start_x']
            line.start.y = self.debug_msg['start_y']
            line.end.x = self.debug_msg['end_x']
            line.end.y = self.debug_msg['end_y']
            senddata = package.SerializeToString()
            self.sock.sendto(senddata, ("127.0.0.1", 20001))
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
            senddata = package.SerializeToString()
            self.sock.sendto(senddata, ("127.0.0.1", 20001))
        elif self.debug_type == 'TEXT':
            msg.type = debug_info.Debug_Msg.TEXT
            text = msg.text
            text.text = 'helloworld'
            text.pos.x = 0
            text.pos.y = 100
            senddata = package.SerializeToString()
            self.sock.sendto(senddata, ("127.0.0.1", 20001))
        elif self.debug_type == 'ROBOT':
            msg.type = debug_info.Debug_Msg.ROBOT
            robot = msg.robot
            robot.pos.x = 200
            robot.pos.y = 200
            robot.dir = 10
            senddata = package.SerializeToString()
            self.sock.sendto(senddata, ("127.0.0.1", 20001))
        elif self.debug_type == 'CURVE':
            msg.type = debug_info.Debug_Msg.CURVE
            curve = msg.curve
            curve.num = 1
            curve.maxLimit = 1000
            curve.minLimit = 990
            senddata = package.SerializeToString()
            self.sock.sendto(senddata, ("127.0.0.1", 20001))
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
            senddata = package.SerializeToString()
            self.sock.sendto(senddata, ("127.0.0.1", 20001))

        elif self.debug_type == 'POINTS':
            msg.type = debug_info.Debug_Msg.Points
            points = msg.points
            for i in range(8):
                point = points.point.add()
                point.x = 30*i
                point.y = 50*i
                senddata = package.SerializeToString()
                self.sock.sendto(senddata, ("127.0.0.1", 20001))
        else:
            print('No this kind of instruction')



if __name__ == '__main__':
    senddebug = SendDebug()
    senddebug.set_point()
    sleep(1)
