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


    def send_all(self, ids, v_x, v_y, v_r, d=10000):
        '''
        '''
        robots_cmd = control.Robots_Command()
        robots_cmd.delay = d
        robot_cmd = []
        for i in range(len(ids)):
            robot_cmd.append(robots_cmd.command.add())
            robot_cmd[i].robot_id = ids[i]
            robot_cmd[i].velocity_x = v_x[i]
            robot_cmd[i].velocity_y = v_y[i]
            robot_cmd[i].velocity_r = v_r[i]
            robot_cmd[i].kick = self.robot_cmd_dic['kick']
            robot_cmd[i].power = self.robot_cmd_dic['power']
            robot_cmd[i].dribbler_spin = self.robot_cmd_dic['dribbler_spin']
            robot_cmd[i].current_angle = self.robot_cmd_dic['current_angle']
            robot_cmd[i].target_angle = self.robot_cmd_dic['target_angle']

        send_data = robots_cmd.SerializeToString()
        self.sock.sendto(send_data, self.address)


if __name__ == "__main__":
    vx = 0
    vy = 0
    robot_id = 0
    w = 0.5236
    send = Send()
    while True:
        send.send_msg(robot_id, vx, vy, w)
        sleep(6)
        w = 0
'''

package = control.Robots_Status.robots_status
print(package)'''