import numpy as np
from message.send_debug import SendDebug
import random
import time
from time import sleep
from message.receive import Receive


class RRT:
    # get the start node and the final node
    def __init__(self, start_x, start_y, goal_x, goal_y, barrierId, receive, step=10, inflateRadius=30, dis_threshold=40, limitation=10000):
        self. lines = []

        self.step = step
        self.inflateRadius = inflateRadius  # inflate radius
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
        self.dis_threshold=dis_threshold
        self.barrierId = barrierId
        self.barrierInfo = np.zeros((len(self.barrierId), 5))  # x, y, r, v_x, v_y
        self.tree = []
        self.restree=[]
        self.tree.append(self.startNode)
        self.receive = receive
        self.Update_Barrier_Info()  # update the information of barriers

    # function: generate a random node in the map
    def Generate_Qrand(self):
        Qrand = [0, 0]
        if random.randint(0, 5) > 2:
            Qrand[0] = self.goalNode[0]
            Qrand[1] = self.goalNode[1]
        else:
            Qrand[0] = np.random.randint(-300, 300)
            Qrand[1] = np.random.randint(-225, 225)

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
        #point1[1] = -point1[1]
        point2 = real_point2.copy()
        #point2[1] = -point2[1]
        for i in range(len(self.barrierInfo)):
            center = [self.barrierInfo[i][0], self.barrierInfo[i][1]]
            mul_1 = (center[0] - point1[0]) * (point2[0] - point1[0]) + (center[1] - point1[1]) * (
                        point2[1] - point1[1])
            mul_2 = (center[0] - point2[0]) * (point1[0] - point2[0]) + (center[1] - point2[1]) * (
                        point1[1] - point2[1])
            if mul_1 > 0 and mul_2 > 0:
                mid = abs((center[0] - point1[0]) * (point2[1] - point1[1]) - (point2[0] - point1[0]) * (
                            center[1] - point1[1]))
                dist = mid/(np.sqrt(np.square(point2[0] - point1[0]) + np.square(point2[1] - point1[1])))
            elif mul_1 == 0 and mul_2 != 0:
                dist = np.sqrt(np.square(center[0] - point1[0]) + np.square(center[1] - point1[1]))
            elif mul_1 != 0 and mul_2 == 0:
                dist = np.sqrt(np.square(center[0] - point2[0]) + np.square(center[1] - point2[1]))
            elif mul_1 == 0 and mul_2 == 0:
                dist = 0
            elif mul_1 < 0 and mul_2 > 0:
                dist = np.sqrt(np.square(center[0] - point1[0]) + np.square(center[1] - point1[1]))
            elif mul_2 < 0 and mul_1 > 0:
                dist = np.sqrt(np.square(center[0] - point2[0]) + np.square(center[1] - point2[1]))
            else:
                dist = 0

            if dist < self.dis_threshold:
                return False

        return True



    # function: calculate Euclidean distance between all existed nodes and Qrand
    def Calculate_Distance(self, node1_x, node1_y, node2_x, node2_y):
        return np.sqrt((node1_x - node2_x) ** 2 + (node1_y - node2_y) ** 2)
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
        theta = np.arctan2(Qrand[1] - Qnear[1], Qrand[0] - Qnear[0])
        Qnext[0] = Qnear[0] + self.step * np.cos(theta)
        Qnext[1] = Qnear[1] + self.step * np.sin(theta)
        Qnext[2] = len(self.tree)
        Qnext[3] = Qnear[2]
        Qnext[4] = Qnear[4]+self.Calculate_Distance(Qnext[0], Qnext[1], Qnear[0], Qnear[1])

        if self.CheckStatus(Qnext) is True:
            # self.shave_rrt(Qnext)  # 若追求效率，则可删掉这句话
            self.tree.append(Qnext)
            # draw a line
            line = [Qnear[0], Qnear[1], Qnext[0], Qnext[1]]
            self.lines.append(line)

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
                index=i
        if index != -1:  # 应该判断一下两点之间有无障碍物,这个函数还没写
            Qnext[4] = maxvalue
            Qnext[3] = self.tree[index][2]  # 把最近点的节点编号记为Qnext的父节点，即改变父亲

    def merge(self):
        current = self.restree.shape[0] - 1
        # import ipdb;ipdb.set_trace()
        while self.restree[current, 3] != -1:
            for index in range(current-1):
                print(self.CheckTwoPoints(self.restree[index], self.restree[current]))
                if self.CheckTwoPoints(self.restree[index], self.restree[current]):
                    self.restree[current, 3] = self.restree[index, 2].copy()
                    break
            current = int(self.restree[current, 3].copy())
        # import ipdb; ipdb.set_trace()
        path = []
        path_lines = []
        point = self.restree[-1]
        parent_x, parent_y, _, parent_id = point
        path.append([parent_x, parent_y])
        for i in range(len(self.restree)):
            parent_id = int(parent_id)
            if parent_id == -1:
                break
            point = self.restree[parent_id]
            x, y, _, parent_id = point
            path.append([x, y])
            path_lines.append([x, y, parent_x, parent_y])
            parent_x = x
            parent_y = y
        return path[::-1], path_lines[::-1]


    # function: update barrier status
    # suppose that all barrier robots is yellow
    # todo
    def Update_Barrier_Info(self):
        #只需要barrierTd不包含自身ID即可，？？？可能包含也可以
        receive = self.receive

        for index in range(len(self.barrierId)):
            receive.get_info(self.barrierId[index][0],self.barrierId[index][1])
            self.barrierInfo[index][0] = receive.robot_info['x']
            self.barrierInfo[index][1] = receive.robot_info['y']


    # function: check new node status
    def CheckStatus(self, Qnext):

        #self.Update_Barrier_Info()#update the information of barriers
        for index in range(len(self.barrierInfo)):
            barrier = self.barrierInfo[index]
            # import ipdb;ipdb.set_trace()
            distance = self.Calculate_Distance(Qnext[0], Qnext[1], barrier[0], barrier[1])
            if distance < self.inflateRadius:
                return False

        return True

    # function: check if we find the goal
    def CheckGoal(self, Qnext):
        distance = self.Calculate_Distance(Qnext[0], Qnext[1], self.goalNode[0], self.goalNode[1])
        if distance < self.inflateRadius:
            return True

    def Generate_Path(self):
        i = 0
        status = False
        while i < self.limitation and status is False:
            Qrand = self.Generate_Qrand()
            Qnear = self.Find_Qnear(Qrand)
            status = self.BornQnext(Qrand, Qnear)
            i += 1
        return status, self.tree, self.lines


    def Get_Path(self):
        # get the final path, a list of points and a list of lines, from start to end
        path = []
        path_lines = []
        point = self.tree[-1]
        parent_x, parent_y, _, parent_id, _ = point
        path.append([parent_x, parent_y])
        for i in range(len(self.tree)):
            if parent_id == -1:
                break
            point = self.tree[parent_id]
            x, y, _, parent_id, _ = point
            path.append([x, y])
            path_lines.append([x, y, parent_x, parent_y])
            parent_x = x
            parent_y = y
        path = path[::-1]
        id = np.arange(0, len(path))
        p_id = np.arange(-1, len(path)-1)
        self.restree = np.concatenate((np.array(path), id[:, np.newaxis], p_id[:, np.newaxis]), axis=1)
        return path, path_lines[::-1]

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
