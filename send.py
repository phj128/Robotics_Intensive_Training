import socket
from time import sleep
import grSim_Packet_pb2 as grSim_Pkg

class Send:
    def __init__(self):
        self.address = ('127.0.0.1',20011)
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def Send_Command(self, robot_ID=0, veltangent=0, velnormal=0, velangular=0):
        package = grSim_Pkg.grSim_Packet()
        commands = package.commands

        commands.timestamp = 0
        commands.isteamyellow = True
        robot_commands = commands.robot_commands.add()

        robot_commands.id = robot_ID
        robot_commands.kickspeedx = 0
        robot_commands.kickspeedz = 0
        robot_commands.veltangent = veltangent
        robot_commands.velnormal = velnormal
        robot_commands.velangular = velangular
        robot_commands.spinner = 0
        robot_commands.wheelsspeed = False

        self.socket.sendto(package.SerializeToString(),self.address)

    def Set_Ball(self, x=0, y=0, vx=0, vy=0):
        package = grSim_Pkg.grSim_Packet()
        replacement = package.replacement

        ball = replacement.ball
        ball.x = x
        ball.y = y
        ball.vx = vx
        ball.vy = vy

        self.socket.sendto(package.SerializeToString(),self.address)

    def MoveTo(self, x, y):


    def Set_Robot(self, robot_ID=0, x=0, y=0, vx=0, vy=0):
        package = grSim_Pkg.grSim_Packet()
        replacement = package.replacement

        robots = replacement.robots.add()
        robots.id = robot_ID
        robots.yellowteam = True
        robots.x = x
        robots.y = y
        robots.vx = vx
        robots.vy = vy

        self.socket.sendto(package.SerializeToString(),self.address)