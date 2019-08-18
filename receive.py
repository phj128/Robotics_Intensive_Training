import socket
from time import sleep
import proto.vision_detection_pb2 as detection


class Receive():
    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("127.0.0.1", 23333))
        self.robot_info = [0, 0, 0, 0, 0, 0]
        self.ball_info = [0, 0, 0, 0]


    def Get_Info(self, color, robotID):
        data, address = self.sock.recvfrom(4096)
        sleep(0.001)

        package = detection.Vision_DetectionFrame()
        package.ParseFromString(data)

        robots_yellow = package.robots_yellow
        robots_blue = package.robots_blue
        import ipdb;
        ipdb.set_trace()
        # 0 is yellow
        if color is 0:
            for robot in robots_yellow:
                if robot.robot_id == robotID:
                    self.robot_info[0] = robot.x
                    self.robot_info[1] = robot.y
                    self.robot_info[2] = robot.orientation
                    self.robot_info[3] = robot.vel_x
                    self.robot_info[4] = robot.vel_y
                    self.robot_info[5] = robot.rotate_vel
        else:
            for robot in robots_blue:
                if robot.robot_id == robotID:
                    self.robot_info[0] = robot.x
                    self.robot_info[1] = robot.y
                    self.robot_info[2] = robot.orientation
                    self.robot_info[3] = robot.vel_x
                    self.robot_info[4] = robot.vel_y
                    self.robot_info[5] = robot.rotate_vel

        balls = package.balls
        self.ball_info[0] = balls.vel_x
        self.ball_info[1] = balls.vel_y
        self.ball_info[2] = balls.x
        self.ball_info[3] = balls.y


if __name__ == "__main__":
    while True:
        receive = Receive()
        receive.Get_Info('yellow', 0)
        print(receive.robot_info)
        print(receive.ball_info)
        sleep(1)

