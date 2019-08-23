from message.send import Send
from message.receive import Receive
from time import sleep

if __name__ == "__main__":
    receive = Receive()
    send = Send()
    color = 'blue'
    robot_id = 0
    send.send_msg(robot_id, 100, 100, 0, power=1.0, d = 300000)
    sleep(3)
    receive.get_info(color, robot_id)
    print("vx = :", receive.robot_info['vx'])
    print("vy = :", receive.robot_info['vy'])

    send.send_msg(robot_id, 10, 20, 0, power=0.0, d = 1000000)
    sleep(10)
    receive.get_info(color, robot_id)
    print("vx = :", receive.robot_info['vx'])
    print("vy = :", receive.robot_info['vy'])
