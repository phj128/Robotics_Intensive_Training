import random
import math
import copy
import time
from message.receive import Receive

class Node(object):
    """
    RRT Node
    """

    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None


class RRT(object):
    """
    Class for RRT Planning
    """

    def __init__(self, s_x, s_y, g_x, g_y, barrierId, receive, rand_area=[300, 225]):
        self.start = Node(s_x, s_y)
        self.end = Node(g_x, g_y)
        self.chang_rand = rand_area[0]
        self.kuan_rand = rand_area[1]
        self.expandDis = 30
        self.goalSampleRate = 0.3
        self.maxIter = 500
        self.barrierId = barrierId
        self.receive = receive
        self.color = 'blue'
        self.robot_id = 0
        self.obstacleList = self.Update_Barrier_Info()
        self.nodeList = [self.start]
        self.size = 30


    def Update_Barrier_Info(self):
        # 只需要barrierTd不包含自身ID即可，？？？可能包含也可以
        receive = self.receive
        obstacle_list = self.receive.get_infos(self.color, self.robot_id)
        return obstacle_list

    def random_node(self):
        """
        产生随机节点
        :return:
        """
        node_x = random.uniform(-self.chang_rand, self.chang_rand)
        node_y = random.uniform(-self.kuan_rand, self.kuan_rand)
        node = [node_x, node_y]

        return node

    @staticmethod
    def get_nearest_list_index(node_list, rnd):
        """
        :param node_list:
        :param rnd:
        :return:
        """
        d_list = [(node.x - rnd[0]) ** 2 + (node.y - rnd[1]) ** 2 for node in node_list]
        min_index = d_list.index(min(d_list))
        return min_index

    @staticmethod
    def collision_check(new_node, obstacle_list):
        size = 30
        a = 1
        for (ox, oy) in obstacle_list:
            dx = ox - new_node.x
            dy = oy - new_node.y
            d = math.sqrt(dx * dx + dy * dy)
            if d <= size:
                a = 0  # collision

        return a  # safe

    def Generate_Path(self):
        status = True
        i = 0
        while True:
            # Random Sampling
            if random.random() > self.goalSampleRate:
                rnd = self.random_node()
            else:
                rnd = [self.end.x, self.end.y]

            # Find nearest node
            min_index = self.get_nearest_list_index(self.nodeList, rnd)
            # print(min_index)

            # expand tree
            nearest_node = self.nodeList[min_index]

            # 返回弧度制
            theta = math.atan2(rnd[1] - nearest_node.y, rnd[0] - nearest_node.x)

            new_node = copy.deepcopy(nearest_node)
            new_node.x += self.expandDis * math.cos(theta)
            new_node.y += self.expandDis * math.sin(theta)
            new_node.parent = min_index

            if not self.collision_check(new_node, self.obstacleList):
                continue

            self.nodeList.append(new_node)

            # check goal
            dx = new_node.x - self.end.x
            dy = new_node.y - self.end.y
            d = math.sqrt(dx * dx + dy * dy)
            if d <= self.expandDis:
                print("Goal!!")
                break
            i += 1
            if i > self.maxIter:
                status = False
                break

        lines = []
        path = [[self.end.x, self.end.y]]
        s_node = self.end
        last_index = -1
        while self.nodeList[last_index].parent is not None:
            node = self.nodeList[last_index]
            path.append([node.x, node.y])
            last_index = node.parent
            lines.append([node.x, node.y, s_node.x, s_node.y])
            s_node = node
        path.append([self.start.x, self.start.y])
        lines.append([self.start.x, self.start.y, node.x, node.y])
        if not status:
            return status, path[::-1][:-1], lines[::-1][:-1]
        return status, path[::-1], lines[::-1]


if __name__ == '__main__':
    print("start RRT path planning")

    obstacle_list = [
        (5, 1, 1),
        (3, 6, 2),
        (3, 8, 2),
        (1, 1, 2),
        (3, 5, 2),
        (9, 5, 2)]
    obstacle_list = [['yellow', 0], ['yellow', 1], ['yellow', 2], ['yellow', 3],
                ['yellow', 4], ['yellow', 5], ['yellow', 6], ['yellow', 7],
                ['blue', 1], ['blue', 2], ['blue', 3], ['blue', 4],
                ['blue', 5], ['blue', 6], ['blue', 7]]
    receive = Receive()
    start = time.time()
    # Set Initial parameters
    rrt = RRT(0,0, 200,200, rand_area=[300, 225], barrierId = obstacle_list, receive=receive)
    status, path, lines = rrt.Generate_Path()
    # import ipdb;ipdb.set_trace()
    end = time.time()
    print("cost", end - start)
    print(path)