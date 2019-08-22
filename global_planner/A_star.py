import numpy as np
from message.send_debug import SendDebug
import random
import time
from time import sleep
from message.receive import Receive

class Point():
    def __init__(self, x, y):
        self.x = x
        self.y = y
    def __eq__(self, other):
        if self.x == other.x and self.y == other.y:
            return True
        return False
    def __str__(self):
        return "x:" + str(self.x) + ", y:" + str(self.y)

class AStar:
    class Node:
        def __init__(self, point, endPoint, g=0):
            self.point = point
            self.father = None
            self.g = g
            self.h = (abs(endPoint.x - point.x) + abs(endPoint.y - point.y)) * 10

    def __init__(self, start_x, start_y, goal_x, goal_y, barrierId, receive, offset_x=10, offset_y=10, color='blue', id=0, inflateRadius=30):
        self.width = 350
        self.height = 250
        self.color = color
        self.robot_id = id
        self.barrierId = barrierId
        self.barrierInfo = np.zeros((len(self.barrierId), 5))  # x, y, r, v_x, v_y
        self.receive = receive
        self.Update_Barrier_Info()  # update the information of barriers

        self.startpoint = Point(start_x, start_y)
        self.endpoint = Point(goal_x, goal_y)
        self.openlist = []
        self.closelist = []
        self.inflateRadius = inflateRadius
        self.dis_threshold = inflateRadius
        self.offsetx = offset_x
        self.offsety = offset_y

    def getMinNode(self):
        '''
        获得openlist中F值最小的点
        :return: Node
        '''
        currentNode = self.openlist[0]
        for node in self.openlist:
            if (node.g + node.h) < (currentNode.g + currentNode.h):
                currentNode = node
        return currentNode

    def pointInCloseList(self, point):
        for node in self.closelist:
            if node.point == point:
                return True
        return False

    def pointInOpenList(self, point):
        for node in self.openlist:
            if node.point == point:
                return node
        return None

    def endPointInCloseList(self):
        for node in self.closelist:
            dist =  np.sqrt(np.square(node.point.x-self.endpoint.x) + np.square(node.point.y - self.endpoint.y))
            if dist < 40:
                return node
        return None

    def searchNear(self, minF, offsetX, offsetY):
        '''
        搜索节点周围的点
        :param minF: F值最小的点
        :param offsetX: 坐标偏移量
        :param offsetY: 坐标偏移量
        :return:
        '''
        print('searching')
        #越界检测
        if minF.point.x + offsetX <= -self.width or minF.point.x + offsetX >= self.width or minF.point.y + offsetY <= -self.height or minF.point.y + offsetY >=self.height:
            print('越界')
            return
        #如果是障碍，则不管
        if self.checkIfObstacle(minF) == True:
            print('遇到障碍')
            return
        #如果在closelist中，则忽略
        currentpoint = Point(minF.point.x + offsetX, minF.point.y + offsetY)
        if self.pointInCloseList(currentpoint):
            return
        #设置单位花费
        if offsetX == 0 or offsetY == 0:
            step = 10
        else:
            step = 14
        #不在openlist中则加入openlist中
        currentNode = self.pointInOpenList(currentpoint)
        if not currentNode:
            currentNode1 = AStar.Node(currentpoint, self.endpoint, minF.g + step)
            currentNode1.father = minF
            self.openlist.append(currentNode1)
            return
        #如果在openlist中，则判断minF到当前节点的G是否更小
        if minF.g + step < currentNode.g:
            currentNode.g = minF.g + step
            currentNode.father = minF

    def Generate_Path(self):
        startnode = AStar.Node(self.startpoint, self.endpoint)

        self.openlist.append(startnode)
        while True:
            #print('created')
            minF = self.getMinNode()
            #print(minF)
            self.closelist.append(minF)
            self.openlist.remove(minF)
            #print(self.openlist)
            self.searchNear(minF, 0, -self.offsety)
            self.searchNear(minF, 0, self.offsety)
            self.searchNear(minF, -self.offsetx, 0)
            self.searchNear(minF, self.offsetx, 0)
            self.searchNear(minF, self.offsetx, self.offsety)
            self.searchNear(minF, -self.offsetx, self.offsety)
            self.searchNear(minF, self.offsetx, -self.offsety)
            self.searchNear(minF, -self.offsetx, -self.offsety)
            point = self.endPointInCloseList()
            #print('point: ', point)
            if point:
                print('find!')
                cPoint = point
                pathlist = []
                lines = []
                ppoint = cPoint.father
                try:
                    lines.append([cPoint.point.x, cPoint.point.y, ppoint.point.x, ppoint.point.y])
                except:
                    lines.append([cPoint.point.x, cPoint.point.y, self.startpoint.x, self.startpoint.y])
                while True:
                    if cPoint.father:
                        pathlist.append([cPoint.point.x, cPoint.point.y])
                        ppoint = cPoint
                        cPoint = cPoint.father
                        try:
                            lines.append([cPoint.point.x, cPoint.point.y, ppoint.point.x, ppoint.point.y])
                        except:
                            cPoint = cPoint
                    else:
                        pathlist.append([self.startpoint.x, self.startpoint.y])
                        self.pathlist = list(reversed(pathlist))
                        return True, self.pathlist, lines
            if len(self.openlist) == 0:
                return False, [], []

    def Get_Path(self):
        try:
            self.pathlist = self.pathlist
            path = self.pathlist.copy()
            id = np.arange(0, len(path))
            p_id = np.arange(-1, len(path) - 1)
            self.restree = np.concatenate((np.array(path), id[:, np.newaxis], p_id[:, np.newaxis]), axis=1)
        except:
            return [], []
        return self.pathlist, []


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
                dist = mid/(np.sqrt(np.square(-dx_0) + np.square(-dy_0)))
            elif mul_1 == 0 and mul_2 != 0:
                dist = np.sqrt(np.square(dx_1) + np.square(dy_1))
            elif mul_1 != 0 and mul_2 == 0:
                dist = np.sqrt(np.square(dx_2) + np.square(dy_2))
            elif mul_1 == 0 and mul_2 == 0:
                dist = 0
            elif mul_1 < 0 and mul_2 > 0:
                dist = np.sqrt(np.square(dx_1) + np.square(dy_1))
            elif mul_2 < 0 and mul_1 > 0:
                dist = np.sqrt(np.square(dx_2) + np.square(dy_2))
            else:
                dist = 0

            if dist < self.dis_threshold:
                return False

        return True


    def merge(self):
        try:
            self.pathlist = self.pathlist
        except:
            return [], []

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
        # import ipdb; ipdb.set_trace()

        # current = self.restree.shape[0] - 1
        # while current > 0:
        #     index = current - 1
        #     print(self.CheckTwoPoints(self.restree[index], self.restree[current]))
        #     current -= 1

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


    def Update_Barrier_Info(self):
        #只需要barrierTd不包含自身ID即可，？？？可能包含也可以
        receive = self.receive
        self.barrierInfo = receive.get_infos(self.color, self.robot_id)

        # for index in range(len(self.barrierId)):
        #     receive.get_info(self.barrierId[index][0],self.barrierId[index][1])
        #     self.barrierInfo[index][0] = receive.robot_info['x']
        #     self.barrierInfo[index][1] = receive.robot_info['y']

    def checkIfObstacle(self, node):
        for i in range(len(self.barrierInfo)):
            dist = np.sqrt(np.square(self.barrierInfo[i][0]-node.point.x) + np.square(self.barrierInfo[i][1]-node.point.y))
            if dist < self.inflateRadius:
                return True #有障碍物
        return False

if __name__ == "__main__":
    receive = Receive()
    astar = AStar(0,0,200,200,[['yellow', 0], ['yellow', 1], ['yellow', 2], ['yellow', 3],
                   ['yellow', 4], ['yellow', 5], ['yellow', 6], ['yellow', 7]], receive)
    s = time.time()
    print('now')
    path = astar.Generate_Path()
    e = time.time()
    print('time cost:', e-s)
    print(path)
#




