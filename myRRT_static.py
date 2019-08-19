import numpy as np
from send_debug import SendDebug
import random
import time
from time import sleep
from receive import Receive


class RRT:
    # get the start node and the final node
    def __init__(self, start_x, start_y, goal_x, goal_y, receive, barrierId, step=10, inflateRadius=30, limitation=10000):
        self. lines = []

        self.step = step
        self.inflateRadius = inflateRadius  # inflate radius
        self.limitation = limitation  # the max number of nodes

        self.startNode = [0, 0, 0, 0]  # x, y, index, parentIndex
        self.goalNode = [0, 0, 0, 0]

        self.startNode[0] = start_x
        self.startNode[1] = start_y
        self.startNode[2] = 0
        self.startNode[3] = -1

        self.goalNode[0] = goal_x
        self.goalNode[1] = goal_y
        self.goalNode[2] = self.limitation
        self.goalNode[3] = 0  # if find a path, update parent index

        self.barrierId = barrierId
        self.barrierInfo = np.zeros((len(self.barrierId), 5))  # x, y, r, v_x, v_y
        self.tree = []
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
        Qnext = [0, 0, 0, 0]
        theta = np.arctan2(Qrand[1] - Qnear[1], Qrand[0] - Qnear[0])
        Qnext[0] = Qnear[0] + self.step * np.cos(theta)
        Qnext[1] = Qnear[1] + self.step * np.sin(theta)
        Qnext[2] = len(self.tree)
        Qnext[3] = Qnear[2]

        if self.CheckStatus(Qnext) is True:
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

    # function: RRT path planning again
    def Reset(self, start_x, start_y, goal_x, goal_y, barrier_num):
        self.startNode[0] = start_x
        self.startNode[1] = start_y
        self.startNode[2] = 0
        self.startNode[3] = -1

        self.goalNode[0] = goal_x
        self.goalNode[1] = goal_y
        self.goalNode[2] = 3000  # the max number of nodes
        self.goalNode[3] = 0  # if find a path, update parent index

        self.tree = []
        self.tree.append(self.startNode)

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
        parent_x, parent_y, _, parent_id = point
        path.append([parent_x, parent_y])
        for i in range(len(self.tree)):
            if parent_id == -1:
                break
            point = self.tree[parent_id]
            x, y, _, parent_id = point
            path.append([x, y])
            path_lines.append([x, y, parent_x, parent_y])
            parent_x = x
            parent_y = y
        return path[::-1], path_lines[::-1]

if __name__ == '__main__':
    time_start = time.time()
    receive = Receive()

    my_rrt = RRT(0, 0, 200, 200, receive,
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

    
