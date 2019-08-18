import numpy as np
from debugModule import DebugModule
import random
from time import sleep
from receive import Receive

class RRT:
    # get the start node and the final node
    def __init__(self, start_x, start_y, goal_x, goal_y, barrierId, step, inflateRadius, limitation):
        self.debug = DebugModule('127.0.0.1', 20001)
        self.step = step
        # self.barrierStatus = np.zeros((barrier_num, 4))  # record x,y,v_x,v_y,radius of barrier
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
        self.barrierInfo = np.zeros((len(self.barrierId),5)) # x,y,v_x,v_y
        self.tree = []
        self.tree.append(self.startNode)

    # function: generate a random node in the map
    def Generate_Qrand(self):
        Qrand = [0, 0]
        if random.randint(0, 5) == 0:
            Qrand[0] = self.goalNode[0]
            Qrand[1] = self.goalNode[1]
        else:
            Qrand[0] = np.random.randint(-600, 600) / 100
            Qrand[1] = np.random.randint(-450, 450) / 100

        return Qrand

    # function: calculate Euclidean distance between all existed nodes and Qrand
    def Calculate_Distance(self, node1_x, node1_y, node2_x, node2_y):
        return np.sqrt((node1_x - node2_x) ** 2 + (node1_y - node2_y) ** 2)

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

        test = [[-300, 300, 300, -300], [300, -300, 400, 400]]

        if self.CheckStatus(Qnext) is True:
            self.tree.append(Qnext)

            # for nodeNum in range(1,len(self.tree)):
            # 	self.debug.Set_Line(self.tree[nodeNum][0],self.tree[nodeNum][1],self.tree[self.tree[nodeNum][3]][0],self.tree[self.tree[nodeNum][3]][1])

            for node in range(len(test)):
                self.debug.Set_Line(test[node][0], test[node][1], test[node][2], test[node][3])

            if self.CheckGoal(Qnext) is True:
                self.goalNode[2] = len(self.tree)
                self.goalNode[3] = Qnext[2]
                self.tree.append(self.goalNode)
                return True
        return False

    # function: update barrier status
    # suppose that all barrier robots is yellow
    # todo
    def Update_Barrier_Info(self,barrierId):
        receive = Receive()

        for index in range(len(self.barrierId)):
            receive.Get_Info(self.barrierId[index][0],self.barrierId[index][1])
            self.barrierInfo[index] = receive.robot_info



    # function: check new node status
    def CheckStatus(self, Qnext):
        # for i in range(len(self.barrierStatus)):
        #     barrier = self.barrierStatus[i]
        #     distance = self.Calculate_Distance(Qnext[0],Qnext[1],barrier[0],barrier[1])
        #     if distance > self.inflateRadius+barrier[3]:
        #         return False
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
        return status