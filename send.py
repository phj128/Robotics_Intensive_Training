import socket
from time import sleep
import proto.zss_debug_pb2 as control

class Send():
    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("127.0.0.1", 50001))
        self.robot_status_dic = {'robot_id': 0, 'infrared': False, 'flat_kick': False, 'chip_kick': False}
        self.robot_cmd_dic = {'delay': 1, 'robot_id': 0, 'velocity_x': 0, 'velocity_y': 0, 'velocity_r': 0,
                          'kick': False,  'power': 1, 'dribbler_spin': 1, 'current_angle': 0, 'target_angle': 0}

    def send_msg(self, id_we_send, v_x, v_y, v_r):

        robot_status = control.Robot_Status()
        robotid = 0
        for robotid in range(8):
            robot_status.robot_id = robotid
            robot_status.infrared = self.robot_status_dic['infrared']
            robot_status.flat_kick = self.robot_status_dic['flat_kick']
            robot_status.chip_kick = self.robot_status_dic['chip_kick']
            senddata = control.Robots_Status().SerializeToString()
            self.sock.sendto(senddata,('127.0.0.1',50001))

            #robotid = robotid + 1

        robot_cmd = control.Robot_Command()
        robotiid = 0
        for robotiid in range(8):
            if robotiid == id_we_send:
                #robot_cmd.delay = self.robot_cmd_dic['delay']
                robot_cmd.robot_id = robotiid
                robot_cmd.velocity_x = v_x
                robot_cmd.velocity_y = v_y
                robot_cmd.velocity_r = v_r
                robot_cmd.kick = self.robot_cmd_dic['kick']
                robot_cmd.power = self.robot_cmd_dic['power']
                robot_cmd.dribbler_spin = self.robot_cmd_dic['dribbler_spin']
                robot_cmd.current_angle = self.robot_cmd_dic['current_angle']
                robot_cmd.target_angle = self.robot_cmd_dic['target_angle']
            else:
                #robot_cmd.delay = self.robot_cmd_dic['delay']
                robot_cmd.robot_id = robotiid
                robot_cmd.velocity_x = self.robot_cmd_dic['velocity_x']
                robot_cmd.velocity_y = self.robot_cmd_dic['velocity_y']
                robot_cmd.velocity_r = self.robot_cmd_dic['velocity_r']
                robot_cmd.kick = self.robot_cmd_dic['kick']
                robot_cmd.power = self.robot_cmd_dic['power']
                robot_cmd.dribbler_spin = self.robot_cmd_dic['dribbler_spin']
                robot_cmd.current_angle = self.robot_cmd_dic['current_angle']
                robot_cmd.target_angle = self.robot_cmd_dic['target_angle']

            senddata = control.Robots_Command().SerializeToString()
            self.sock.sendto(senddata,('127.0.0.1',50001))
            robotiid = robotiid + 1


if __name__ == "__main__":
    while True:
        send = Send()
        send.send_msg(4,1,2,3)
        sleep(1)
'''

package = control.Robots_Status.robots_status
print(package)'''