import numpy as np
from message.send_debug import SendDebug
from numpy.random import randint
from numpy import arange, concatenate, newaxis
import time
from math import sin, cos
from time import sleep
from message.receive import Receive
from math import sqrt, atan2, sin, cos


def filter_infos(infos, robot_id, color, computing_time=0.10):
    infos_ = infos.copy()
    self_message = []
    i = 0
    for info in infos:
        if info[3] == robot_id:
            if info[2] == color:
                self_message = info.copy()
                infos_.pop(i)
                break
        i += 1
    return infos_, self_message


class RRT:
    # get the start node and the final node
    def __init__(self, start_x, start_y, goal_x, goal_y, infos, step=10, robot_id=0, color='blue',
                 inflateRadius=20, dis_threshold=30, limitation=500):
        self.lines = []
        self.draw = False
        self.step = step
        self.inflateRadius = inflateRadius  # inflate radius # 是一个列表
        self.changeable_radius = [] # 是一个列表
        self.limitation = limitation  # the max number of nodes

        self.startNode = [0, 0, 0, 0, 0]  # x, y, index, parentIndex, depth
        self.goalNode = [0, 0, 0, 0, 0]

        self.startNode[0] = start_x
        self.startNode[1] = start_y
        self.startNode[2] = 0
        self.startNode[3] = -1
        self.startNode[4] = 0

        self.goalNode[0] = goal_x
        self.goalNode[1] = goal_y
        self.goalNode[2] = self.limitation
        self.goalNode[3] = 0  # if find a path, update parent index
        self.goalNode[4] = 0
        self.dis_threshold = dis_threshold
        infos, self_message = filter_infos(infos, robot_id, color)
        self.self_message = self_message
        self.barrierInfo = infos # x, y, color ,id, ori, v, a
        self.tree = []
        self.restree = []
        self.tree.append(self.startNode)
        self.Update_barrier_radius()

    def Update_barrier_radius(self):
        self.changeable_radius = [0.02*info[5]+0.02*self.self_message[5]+0.00125*info[6] for info in self.barrierInfo]
        # for index in range(len(self.barrierInfo)):
        #     # v = math.sqrt(self.barrierInfo[index][5]*self.barrierInfo[index][5] + self.barrierInfo[index][6]*self.barrierInfo[index][6])
        #     # a = math.sqrt(self.barrierInfo[index][7]*self.barrierInfo[index][7] + self.barrierInfo[index][8]*self.barrierInfo[index][8])
        #     v = self.barrierInfo[index][5]
        #     a = self.barrierInfo[index][6]
        #     v_self = self.self_message[5]
        #     self.changeable_radius.append(0.02*v + 0.02*v_self + 0.00125*a)

    # function: generate a random node in the map
    def Generate_Qrand(self):
        Qrand = [0, 0]
        if randint(0, 5) > 3:
            Qrand[0] = self.goalNode[0]
            Qrand[1] = self.goalNode[1]
        else:
            Qrand[0] = randint(-300, 300)
            Qrand[1] = randint(-200, 200)

        return Qrand
    '''
    def CheckTwoPoints(self, point1, point2):
        if point1[0] == point2[0]: #两点间无斜率
            A = -1/point1[0]
            B = 0
            C = 1
        else:
            k = (point1[1]-point2[1])/(point1[0]-point2[0])
            b = point1[1] - k * point1[0]
            A = k
            B = -1
            C = b
        for i in range(len(self.barrierInfo)):
            fenzi = A*self.barrierInfo[i][0] + B*self.barrierInfo[i][1] + C
            fenmu = np.sqrt(A*A + B*B)
            dist = abs(fenzi/fenmu)
            if dist <= self.dis_threshold:
                return False
        return True
    '''
    def CheckTwoPoints(self, real_point1, real_point2):
        point1 = real_point1.copy()
        point2 = real_point2.copy()
        for i in range(len(self.barrierInfo)):
            center = [self.barrierInfo[i][0], self.barrierInfo[i][1]]
            dx_1 = center[0] - point1[0]
            dy_1 = center[1] - point1[1]
            dx_2 = center[0] - point2[0]
            dy_2 = center[1] - point2[1]
            dx_0 = point1[0] - point2[0]
            dy_0 = point1[1] - point2[1]
            mul_1 = (dx_1) * (-dx_0) + (dy_1) * (-dy_0)
            mul_2 = (dx_2) * (dx_0) + (dy_2) * (dy_0)
            if mul_1 > 0 and mul_2 > 0:
                mid = abs((dx_1) * (-dy_0) - (-dx_0) * (dy_1))
                dist = mid / (sqrt(dx_0 * dx_0 + dy_0 * dy_0))
            elif mul_1 == 0 and mul_2 != 0:
                dist = sqrt(dx_1 * dx_1 + dy_1 * dy_1)
            elif mul_1 != 0 and mul_2 == 0:
                dist = sqrt(dx_2 * dx_2 + dy_2 * dy_2)
            elif mul_1 == 0 and mul_2 == 0:
                dist = 0
            elif mul_1 < 0 and mul_2 > 0:
                dist = sqrt(dx_1 * dx_1 + dy_1 * dy_1)
            elif mul_2 < 0 and mul_1 > 0:
                dist = sqrt(dx_2 * dx_2 + dy_2 * dy_2)
            else:
                dist = 0

            if dist < (self.dis_threshold + self.changeable_radius[i]):
                return False

        return True


    def check_two_points(self, real_point1, real_point2):
        num = 60
        point1 = real_point1.copy()
        point2 = real_point2.copy()
        info = np.array(self.barrierInfo)[:, :2]
        select_points = np.zeros([num, 2])
        select_points[:, 0] = np.linspace(0, point2[0] - point1[0], num, endpoint=True)
        select_points[:, 1] = np.linspace(0, point2[1] - point1[1], num, endpoint=True)
        select_points[:, 0] += point1[0]
        select_points[:, 1] += point1[1]
        delta = select_points[np.newaxis, ...] - info[:, np.newaxis, :]
        dis = np.sqrt(np.sum(delta * delta, axis=2))
        return (dis < self.dis_threshold).sum()


    # function: calculate Euclidean distance between all existed nodes and Qrand
    def Calculate_Distance(self, node1_x, node1_y, node2_x, node2_y):
        return sqrt((node1_x - node2_x) * (node1_x - node2_x) + (node1_y - node2_y) * (node1_y - node2_y))
        #return abs(node1_x - node2_x)+abs(node1_y - node2_y)

    # function: find the nearest node to Qrand
    def Find_Qnear(self, Qrand):
        minDis = np.inf
        minIndex = np.inf
        for i in range(len(self.tree)):
            node = self.tree[i]
            distance = self.Calculate_Distance(Qrand[0], Qrand[1], node[0], node[1])
            if distance < minDis:
                minDis = distance
                minIndex = i
        return self.tree[minIndex]

    # function: generate Qnext and add it into the tree
    def BornQnext(self, Qrand, Qnear):
        Qnext = [0, 0, 0, 0, 0]
        theta = atan2(Qrand[1] - Qnear[1], Qrand[0] - Qnear[0])
        Qnext[0] = Qnear[0] + self.step * cos(theta)
        Qnext[1] = Qnear[1] + self.step * sin(theta)
        Qnext[2] = len(self.tree)
        Qnext[3] = Qnear[2]
        Qnext[4] = Qnear[4]+self.Calculate_Distance(Qnext[0], Qnext[1], Qnear[0], Qnear[1])

        if self.CheckStatus(Qnext) is True:
            # self.shave_rrt(Qnext)  # 若追求效率，则可删掉这句话
            self.tree.append(Qnext)
            # draw a line
            line = [Qnear[0], Qnear[1], Qnext[0], Qnext[1]]
            self.lines.append(line)
            # import ipdb;ipdb.set_trace()
            if self.draw:
                if len(self.lines) == 1:
                    send_tree = SendDebug('LINE', self.lines)
                    send_tree.send()
                else:
                    # import ipdb;ipdb.set_trace()
                    send_tree = SendDebug('LINE', [self.lines, []])
                    send_tree.send()

            if self.CheckGoal(Qnext) is True:
                self.goalNode[2] = len(self.tree)
                self.goalNode[3] = Qnext[2]
                self.tree.append(self.goalNode)
                return True

        return False

    # function: shave the path
    def shave_rrt(self, Qnext):
        index = -1
        radius = 18
        maxvalue = 2000
        for i in range(len(self.tree)):
            distance = self.Calculate_Distance(Qnext[0], Qnext[1], self.tree[i][0], self.tree[i][1])
            if distance < radius and self.tree[i][4]+distance < maxvalue:
                maxvalue = self.tree[i][4]+distance
                index = i
        if index != -1:  # 应该判断一下两点之间有无障碍物,这个函数还没写
            Qnext[4] = maxvalue
            Qnext[3] = self.tree[index][2]  # 把最近点的节点编号记为Qnext的父节点，即改变父亲

    def merge(self):
        current = self.restree.shape[0] - 1
        # import ipdb;ipdb.set_trace()
        while current > 0:
            for index_ in range(current-1):
                index = current - index_ - 2
                # print(self.check_two_points(self.restree[index], self.restree[current]))
                if not self.CheckTwoPoints(self.restree[index], self.restree[current]):
                    self.restree[current, 3] = index + 1
                    break
                self.restree[current, 3] = index
            current = int(self.restree[current, 3].copy())

        path = []
        path_lines = []
        point = self.restree[-1]
        parent_x, parent_y, _, parent_id = point
        path.append([parent_x, parent_y])
        for _ in range(len(self.restree)):
            parent_id = int(parent_id)
            if parent_id == -1:
                break
            point = self.restree[parent_id]
            x, y, _, parent_id = point
            path.append([x, y])
            path_lines.append([x, y, parent_x, parent_y])
            parent_x = x
            parent_y = y
        return path, path_lines


    # function: update barrier status
    # suppose that all barrier robots is yellow
    # todo


    # function: check new node status
    def CheckStatus(self, Qnext):

        #self.Update_Barrier_Info()#update the information of barriers
        for index in range(len(self.barrierInfo)):
            barrier = self.barrierInfo[index]
            # import ipdb;ipdb.set_trace()
            distance = self.Calculate_Distance(Qnext[0], Qnext[1], barrier[0], barrier[1])
            if distance < self.changeable_radius[index]+self.inflateRadius:
                return False

        return True

    # function: check if we find the goal
    def CheckGoal(self, Qnext):
        distance = self.Calculate_Distance(Qnext[0], Qnext[1], self.goalNode[0], self.goalNode[1])
        if distance < 7:
            return True


    def Generate_Path(self):
        i = 0
        status = False
        while i < self.limitation and status is False:
            Qrand = self.Generate_Qrand()
            Qnear = self.Find_Qnear(Qrand)
            status = self.BornQnext(Qrand, Qnear)
            i += 1
        return status



    def Get_Path(self):
        # get the final path, a list of points and a list of lines, from start to end
        path = []
        point = self.tree[-1]
        parent_x, parent_y, _, parent_id, _ = point
        path.append([parent_x, parent_y])
        for _ in range(len(self.tree)):
            if parent_id == -1:
                break
            point = self.tree[parent_id]
            x, y, _, parent_id, _ = point
            path.append([x, y])
        id = arange(0, len(path))
        p_id = arange(-1, len(path)-1)
        self.restree = concatenate((np.array(path), id[:, newaxis], p_id[:, newaxis]), axis=1)
        return self.merge()

if __name__ == '__main__':
    time_start = time.time()
    receive = Receive()

    my_rrt = RRT(-290, -220, 290, 220, receive,
              [['yellow', 0], ['yellow', 1], ['yellow', 2], ['yellow', 3],
               ['yellow', 4], ['yellow', 5], ['yellow', 6], ['yellow', 7]])
    status, tree, lines = my_rrt.Generate_Path()
    path, path_lines = my_rrt.Get_Path()

    time_end = time.time()
    print('path cost:', time_end - time_start)

    send_tree = SendDebug('LINE', [lines, path_lines])
    send_tree.send()
    end = time.time()
    print('total cost:', end - time_start)
    print(path)
