import socket
from time import sleep
import proto.zss_cmd_pb2 as control

class Send():
    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        # self.sock.bind(("127.0.0.1", 50001))
        self.address = ("127.0.0.1", 50001)
        self.robot_status_dic = {'robot_id': 0, 'infrared': False, 'flat_kick': False, 'chip_kick': False}
        self.robot_cmd_dic = {'delay': 1, 'robot_id': 0, 'velocity_x': 0, 'velocity_y': 0, 'velocity_r': 0,
                          'kick': False,  'power': 1, 'dribbler_spin': 1, 'current_angle': 0, 'target_angle': 0}

    def send_msg(self, id_we_send, v_x, v_y, v_r, d=10000):
        robots_cmd = control.Robots_Command()
        robots_cmd.delay = d
        robot_cmd = robots_cmd.command.add()
        robot_cmd.robot_id = id_we_send
        robot_cmd.velocity_x = v_x
        robot_cmd.velocity_y = v_y
        robot_cmd.velocity_r = v_r
        robot_cmd.kick = self.robot_cmd_dic['kick']
        robot_cmd.power = self.robot_cmd_dic['power']
        robot_cmd.dribbler_spin = self.robot_cmd_dic['dribbler_spin']
        robot_cmd.current_angle = self.robot_cmd_dic['current_angle']
        robot_cmd.target_angle = self.robot_cmd_dic['target_angle']

        send_data = robots_cmd.SerializeToString()
        self.sock.sendto(send_data, self.address)


if __name__ == "__main__":
    vx = 100
    vy = 100
    robot_id = 0
    while True:
        send = Send()
        send.send_msg(robot_id, vx, vy, 3)
        sleep(1)
        vx /= 2
        vy /= 2
'''

package = control.Robots_Status.robots_status
print(package)'''