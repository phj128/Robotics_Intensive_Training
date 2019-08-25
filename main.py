from message.send import Send
from message.send_debug import SendDebug
from message.receive import Receive

from global_planner.myRRT_static import RRT
from global_planner.myRRTstar import RRT as RRT_STAR
from global_planner.myRRTmerge import RRT as RRT_MERGE
from global_planner.A_star import AStar as A_star

from ArtificialPotentialFieldMethod.myAPF import APF

from local_planner.p_control import P_control
from local_planner.xy_control import XY_control
from local_planner.xy_near import XY_near
from local_planner.xy_p import XY_p
from local_planner.xy_angle import XY_angle
from local_planner.xy_speed import XY_speed

from run import run
from run import run_while
from run import run_line
from run import run_shrink
import time


if __name__ == '__main__':
    color = 'blue'
    robot_id = 4
    # barriers = [['yellow', 0], ['yellow', 1], ['yellow', 2], ['yellow', 3],
    #             ['yellow', 4], ['yellow', 5], ['yellow', 6], ['yellow', 7],
    #             ['blue', 1], ['blue', 2], ['blue', 5], ['blue', 3],
    #             ['blue', 0], ['blue', 6], ['blue', 7]]
    g_x, g_y = (-250, 150)
    barriers = [['yellow', 0], ['blue', 1], ['blue', 2], ['blue', 3],
                ['blue', 4], ['blue', 5], ['blue', 6], ['blue', 7]]

    i = 0
    global_p = RRT_MERGE
    local_p = XY_speed
    RUN = run_shrink

    receive = Receive()

    while True:
        if i % 2 == 0:
            RUN(color, robot_id, barriers, g_x, g_y, global_p, local_p, receive)
        else:
            RUN(color, robot_id, barriers, -g_x, -g_y, global_p, local_p, receive)
        i += 1