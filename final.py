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
global status_coll, status, finish
global barrier

barrier = []
circles = []

color = 'blue'
id = 0


def receive_module():
    global infos
    global x, y, ori
    global i
    global color, id
    global threshold
    global vx, vy
    x, y, ori = 0, 0, 0
    vx = 0
    vy = 0
    threshold = 30
    i = 0
    infos = []
    receive = Receive()
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
    path, path_lines, tree, lines = [[x, y], [target_x, target_y]], [], [], []
    global_planner = RRT
    status_coll = False
    status = False
    finish = False
    index = 1
    R = 30
    i = 0
    while True:
        try:
            N = len(path)
            if i > N - 2:
                i = N - 2
            status_coll, index = check_path_thread([x, y], path[i + 1], infos, R / index, color=color, id=id)
            if not status or not status_coll:
                lock.acquire()
                start = time.time()
                if index > 5:
                    index = 5
                global_path = global_planner(x, y, target_x, target_y, infos, color=color, robot_id=id, inflateRadius=R/index, dis_threshold=R/index)
                status = global_path.Generate_Path()
                if not status:
                    index += 1
                else:
                    index = 1
                path, path_lines = global_path.Get_Path()
                i = 0
                end = time.time()
                print('time cost:', end - start)
                lock.release()
                if status:
                    sleep(0.4)
            else:
                continue
        except:
            continue


def local_module():
    global x, y, ori
    global vx, vy
    global i
    global target_x, target_y
    global path
    global status_coll, status, finish
    local_planner = XY_left_right
    finish = False
    while True:
        try:
            if distance((x, y), (target_x, target_y)) > 7:
                N = len(path)
                if N <= 1:
                    status = False
                    vx, vy = 40, 40
                    continue
                if N == 2:
                    i = 0
                if i < N - 1:
                    motion = local_planner()
                    vx, vy, finish = motion.line_control(x, y, ori, path, i, N, target_x, target_y, infos=infos,
                                                         color=color, robot_id=id)
                if finish:
                    i += 1
                    finish = False
            else:
                status = False
                target_x = -target_x
                target_y = -target_y
                i = 0
        except:
            continue


def send_module():
    # vx, vy = 10, 10
    while True:
        try:
            send = Send()
            send.send_msg(id, vx, vy, 0)
            sleep(0.005)
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
