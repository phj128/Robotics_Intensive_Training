from message.send import Send
from message.send_debug import SendDebug
from message.receive import Receive

from thread_global.myRRTmerge import RRT as thread_RRT
from thread_global.RRTmerge_predic import RRT as RRT_pred
from thread_global.RRTmerge_circle_v import RRT as RRT_circle_v
from thread_global.RRTmerge_circle_v_a import RRT as RRT_circle_v_a
from thread_global.RRTmerge_circle_v_a_v import RRT as RRT_circle_v_a_v

from thread_local.xy_speed import XY_speed
from thread_local.xy_speed_force import XY_speed as XY_speed_force

from utils import select_info, distance, check_path_thread, make_vel, check_goals

import time, threading

global infos
global path, path_lines, tree, lines
global target_x, target_y
global x, y, ori
global vx, vy
global i
global color, id
global threshold
global status_coll, status, finish
global ids, vxs, vys
global simu_targets
global radius


id = 5
ids = [id, 0, 1, 2, 3, 4, 6]
vxs = [0, 30, 30, 30, 30, 30, 30]
vys = [0, 30, 30, 30, 30, 30, 30]
radius = []


def receive_module():
    global infos
    global x, y, ori
    global i
    global color, id
    global threshold
    global vx, vy
    x, y = 0, 0
    vx = 0
    vy = 0
    threshold = 30
    i = 0
    infos = []
    receive = Receive()
    color = 'blue'
    id = 5
    while True:
        infos = receive.thread_infos()
        x, y, _, _, ori = select_info(infos, color, id)


def global_module():
    global infos
    global target_x
    global target_y
    global path, path_lines, tree, lines
    global x, y
    global i
    global status_coll, status
    global radius
    path, path_lines, tree, lines = [], [], [], []
    target_x, target_y = 250, -150
    global_planner = RRT_circle_v_a_v
    status_coll = False
    status = False
    index = 1
    R = 30
    while True:
        time_start = time.time()
        if index > 3:
            index = 3
        global_path = global_planner(x, y, target_x, target_y, infos, color=color, robot_id=id, inflateRadius=R/index)
        status, tree, lines, radius = global_path.Generate_Path()
        if not status:
            index += 1
        else:
            index = 1
        path_, path_lines = global_path.Get_Path()
        print('ori:', len(path_))
        path, path_lines = global_path.merge()
        print('nodes:', len(path))
        time_end = time.time()
        print('path cost:', time_end - time_start)
        i = 0


def local_module():
    global x, y, ori
    global vx, vy
    global i
    global target_x, target_y
    global path
    global status_coll, status, finish
    local_planner = XY_speed_force
    finish = False
    while True:
        try:
            if distance((x, y), (target_x, target_y)) > 30:
                N = len(path)
                if N <= 1:
                    status = False
                    vxs[0], vys[0] = 30, 30
                    continue
                if N == 2:
                    i = 0
                if i < N - 1:
                    motion = local_planner()
                    vx, vy, finish = motion.line_control(x, y, ori, path, i, N, target_x, target_y, infos=infos,
                                                         color=color, robot_id=id)
                    vxs[0] = vx
                    vys[0] = vy
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
    global ids, vxs, vys
    while True:
        send = Send()
        send.send_all(ids, vxs, vys, [0, 0, 0, 0, 0, 0, 0])
        print('vx', vxs[0])
        print('vy', vys[0])


def debug_module():
    global lines, path_lines
    while True:
        debug_info = SendDebug('LINE', [lines, path_lines], circles=radius, infos=infos)
        debug_info.send()


def simulation_module():
    global simu_targets
    global vxs, vys
    global ids
    global x, y
    simu_targets = [[-150, 150], [-75, 150], [0, 150], [75, 150], [150, 150], [1, 1]]
    while True:
        # try:
        vxs, vys = make_vel(simu_targets, infos, vxs, vys)
        simu_targets = check_goals(simu_targets, infos, x, y)
        # except:
        #     continue



if __name__ == '__main__':
    thread1 = threading.Thread(target=receive_module)
    thread2 = threading.Thread(target=global_module)
    thread3 = threading.Thread(target=local_module)
    thread4 = threading.Thread(target=send_module)
    thread5 = threading.Thread(target=debug_module)
    thread6 = threading.Thread(target=simulation_module)

    thread1.start()
    time.sleep(0.1)
    thread2.start()
    time.sleep(0.1)
    thread3.start()
    time.sleep(0.1)
    thread4.start()
    time.sleep(0.1)
    thread6.start()
    time.sleep(1)
    thread5.start()

    thread1.join()
    thread2.join()
    thread3.join()
    thread4.join()
    thread5.join()
    thread6.join()
