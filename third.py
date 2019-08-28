from message.send import Send
from message.send_debug import SendDebug
from message.receive import Receive

from thread_global.RRTfinal import RRT as RRT
from thread_global.RRTfinal_move import RRT as RRT_move

from thread_local.xy_speed_force_optimization import XY_speed as XY_speed_force_optimization
from thread_local.xy_left_right import XY_speed as XY_left_right
from thread_local.xy_left_right_spring import XY_speed as XY_left_right_spring
from thread_local.XY_apf import XY_speed as XY_apf

from utils import select_info, distance, check_path_thread

import time, threading
from time import sleep

global infos
global path, path_lines, tree, lines, circles
global target_x, target_y
global x, y, ori
global vx, vy
global i
global color, id
global threshold
global status_coll, status, finish, shock
global barrier
global start_time, end_time

barrier = []
circles = []

color = 'blue'
id = 0
start_time = 0

def receive_module():
    global infos
    global x, y, ori
    global i
    global color, id
    global threshold
    global vx, vy
    global start_time
    x, y, ori = 0, 0, 0
    vx = 0
    vy = 0
    threshold = 30
    i = 0
    infos = []
    receive = Receive()
    start_time = time.time()
    while True:
        try:
            infos = receive.thread_infos()
            x, y, _, _, ori = select_info(infos, color, id)
        except:
            continue


def global_module():
    global infos
    global target_x
    global target_y
    global path, path_lines, tree, lines, circles
    global x, y
    global i
    global status_coll, status
    global lock
    global barrier
    target_x, target_y = -250, 150
    path, path_lines, tree, lines = [[target_x, target_y], [-target_x, -target_y]], [[target_x, target_y, -target_x, -target_y]], [], []


def local_module():
    global x, y, ori
    global vx, vy
    global i
    global target_x, target_y
    global path
    global status_coll, status, finish, shock
    global start_time
    local_planner = XY_apf
    shock = False
    while True:
        try:
            if distance((x, y), (target_x, target_y)) > 7:
                motion = local_planner()
                vx, vy, shock = motion.line_control(x, y, ori, path, i, 2, target_x, target_y, infos=infos,
                                                     color=color, robot_id=id, start_time=start_time)
            else:
                target_x = -target_x
                target_y = -target_y
                start_time = time.time()
        except:
            continue


def send_module():
    # vx, vy = 10, 10
    global shock, end_time
    while True:
        try:
            send = Send()
            send.send_msg(id, vx, vy, 0)
            if shock:
                sleep(0.3)
                shock = False
                print('shock')
            else:
                sleep(0.008)
        except:
            continue


def debug_module():
    global lines, path_lines, circles, barrier
    while True:
        # try:
        # debug_info = SendDebug('LINE', [[], path_lines], infos=barrier, circles=circles, id=id, color_rob=color)
        debug_info = SendDebug('LINE', [[], path_lines])
        debug_info.send()
        # except:
        #     continue


if __name__ == '__main__':
    thread1 = threading.Thread(target=receive_module)
    thread2 = threading.Thread(target=global_module)
    thread3 = threading.Thread(target=local_module)
    thread4 = threading.Thread(target=send_module)
    thread5 = threading.Thread(target=debug_module)
    lock = threading.Lock()

    thread1.start()
    thread2.start()
    thread3.start()
    thread4.start()
    thread5.start()

    thread1.join()
    thread2.join()
    thread3.join()
    thread4.join()
    thread5.join()
