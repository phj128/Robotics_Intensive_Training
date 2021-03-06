"""
lbx
"""
import math
import random
# from matplotlib import pyplot as plt
# from matplotlib.patches import Circle
import numpy as np
import time


class Vector2d():
    """
    2维向量, 支持加减, 支持常量乘法(右乘)
    """

    def __init__(self, x, y):
        self.deltaX = x
        self.deltaY = y
        self.length = -1
        self.direction = [0, 0]
        self.vector2d_share()

    def vector2d_share(self):
        if type(self.deltaX) == type(list()) and type(self.deltaY) == type(list()):
            deltaX, deltaY = self.deltaX, self.deltaY
            self.deltaX = deltaY[0] - deltaX[0]
            self.deltaY = deltaY[1] - deltaX[1]
            self.length = math.sqrt(self.deltaX ** 2 + self.deltaY ** 2) * 1.0
            if self.length > 0:
                self.direction = [self.deltaX / self.length, self.deltaY / self.length]
            else:
                self.direction = None
        else:
            self.length = math.sqrt(self.deltaX ** 2 + self.deltaY ** 2) * 1.0
            if self.length > 0:
                self.direction = [self.deltaX / self.length, self.deltaY / self.length]
            else:
                self.direction = None

    def __add__(self, other):
        """
        + 重载
        :param other:
        :return:
        """
        vec = Vector2d(self.deltaX, self.deltaY)
        vec.deltaX += other.deltaX
        vec.deltaY += other.deltaY
        vec.vector2d_share()
        return vec

    def __sub__(self, other):
        vec = Vector2d(self.deltaX, self.deltaY)
        vec.deltaX -= other.deltaX
        vec.deltaY -= other.deltaY
        vec.vector2d_share()
        return vec

    def __mul__(self, other):
        vec = Vector2d(self.deltaX, self.deltaY)
        vec.deltaX *= other
        vec.deltaY *= other
        vec.vector2d_share()
        return vec

    def __truediv__(self, other):
        return self.__mul__(1.0 / other)

    def __repr__(self):
        return 'Vector deltaX:{}, deltaY:{}, length:{}, direction:{}'.format(self.deltaX, self.deltaY, self.length,
                                                                             self.direction)

def get_info(info, receive, color, id):
    receive = receive
    obstacles = receive.get_infos(color, id)
    return obstacles


def filter_infos(infos, robot_id, color, computing_time=0.10):
    infos_ = infos.copy()
    self_message = []
    for i in range(len(infos)):
        if infos[i][3] == robot_id:
            if infos[i][2] == color:
                self_message = infos_[i]
                infos_.pop(i)
                break
    return infos_, self_message



class APF():
    """
    人工势场寻路
    """

    def __init__(self, s_x, s_y, g_x, g_y, info, k_att=30, k_rep=8000, rr=80,
                 step_size=10, max_iters=500, goal_threshold=10, att_threshold=50, dis_threshold=30, color='blue', robot_id=0, inflateRadius=30):
        """
        :param s_x, s_y: 起点
        :param g_x, g_y: 终点
        :param info: 障碍物列表，会转化为每个元素为Vector2d对象
        :param k_att: 引力系数
        :param k_rep: 斥力系数
        :param rr: 斥力作用范围
        :param step_size: 步长
        :param max_iters: 最大迭代次数
        :param goal_threshold: 离目标点小于此值即认为到达目标点
        """
        self.s_p = [s_x, s_y]
        self.g_p = [g_x, g_y]
        self.start = Vector2d(s_x, s_y)
        self.current_pos = Vector2d(s_x, s_y)
        self.goal = Vector2d(g_x, g_y)
        self.color = color
        self.id = robot_id
        obstacles, _ = filter_infos(info, robot_id, color)
        self.barrierInfo = np.array(obstacles)[:, :2]
        self.dis_threshold = dis_threshold
        self.obstacles = [Vector2d(OB[0], OB[1]) for OB in obstacles]
        self.k_att = k_att
        self.k_rep = k_rep
        self.rr = rr  # 斥力作用范围
        self.step_size = step_size
        self.max_iters = max_iters
        self.iters = 0
        self.goal_threashold = goal_threshold
        self.att_threshold = att_threshold
        self.path = list()
        self.is_path_plan_success = False

        self.delta_t = 0.01

    def attractive(self):
        """
        引力计算
        :return: 引力
        """
        if (self.goal - self.current_pos).length < self.att_threshold:
            att = (self.goal - self.current_pos) * self.k_att  # 方向由机器人指向目标点
            return att
        else:
            att = (self.goal - self.current_pos) * self.k_att
            temp = self.att_threshold * (2*(self.goal - self.current_pos).length - self.att_threshold)
            att.deltaX = att.deltaX * temp / att.length
            att.deltaY = att.deltaY * temp / att.length
            att.length = temp
            return att

    def repulsion(self):
        """
        斥力计算, 改进斥力函数, 解决不可达问题
        :return: 斥力大小
        """
        rep = Vector2d(0, 0)  # 所有障碍物总斥力
        for obstacle in self.obstacles:
            # obstacle = Vector2d(0, 0)
            obs_to_rob = self.current_pos - obstacle
            rob_to_goal = self.goal - self.current_pos
            if (obs_to_rob.length > self.rr):  # 超出障碍物斥力影响范围
                continue
            else:
                rep_1 = Vector2d(obs_to_rob.direction[0], obs_to_rob.direction[1]) * self.k_rep * (
                        1.0 / obs_to_rob.length - 1.0 / self.rr) / (obs_to_rob.length ** 2) * (rob_to_goal.length ** 2)
                rep_2 = Vector2d(rob_to_goal.direction[0], rob_to_goal.direction[1]) * self.k_rep * ((1.0 / obs_to_rob.length - 1.0 / self.rr) ** 2) * rob_to_goal.length
                rep += (rep_1+rep_2)
        return rep

    def CheckTwoPoints(self, real_point1, real_point2):
        point1 = real_point1.copy()
        point2 = real_point2.copy()
        for i in range(len(self.barrierInfo)):
            center = np.array([self.barrierInfo[i][0], self.barrierInfo[i][1]]).astype('float')
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
        # import ipdb;ipdb.set_trace()
        return path[::-1], path_lines[::-1]

    def Generate_Path(self):
        """
        path plan
        :return:
        """
        while (self.iters < self.max_iters and (self.current_pos - self.goal).length > self.goal_threashold):
            f_vec = self.attractive() + self.repulsion()
            self.current_pos += Vector2d(f_vec.direction[0], f_vec.direction[1]) * self.step_size
            self.iters += 1
            self.path.append([self.current_pos.deltaX, self.current_pos.deltaY])
        if len(self.path) < 2:
            self.path = [self.s_p, self.g_p]

        if (self.current_pos - self.goal).length <= self.goal_threashold:
            self.is_path_plan_success = True
        return self.is_path_plan_success, [], []

    def Get_Path(self):
        # get the final path, a list of points and a list of lines, from start to end
        path_lines = []
        path = []
        num = len(self.path)
        for i in range(num - 1):
            x, y = self.path[i].copy()
            x_, y_ = self.path[i + 1].copy()
            path_lines.append([x, y, x_, y_])
            path.append([x, y, i, i - 1])
        path.append([self.path[-1][0], self.path[-1][1], num-1, num-2])
        self.restree = np.array(path).astype('float')
        # import ipdb;ipdb.set_trace()
        return np.array(self.path), path_lines


if __name__ == '__main__':
    # 相关参数设置
    k_att, k_rep = 1.0, 0.8
    rr = 3
    step_size, max_iters, goal_threashold = .2, 500, .2  # 步长0.5寻路1000次用时4.37s, 步长0.1寻路1000次用时21s
    step_size_ = 2

    # 设置起点终点
    start, goal = (0, 0), (15, 15)

    # 障碍物设置
    obs = [[1, 4], [2, 4], [3, 3], [6, 1], [6, 7], [10, 6], [11, 12], [14, 14]]
    # obs = [[1, 4, 1], [2, 4, 1], [3, 3, 1], [6, 1, 1], [6, 7, 1], [10, 6, 1], [11, 12, 1], [14, 14, 1]]
    print('obstacles: {0}'.format(obs))
    for i in range(0):
        obs.append([random.uniform(2, goal[1] - 1), random.uniform(2, goal[1] - 1)])

    # 调用人工势场
    apf = APF(start, goal, obs, k_att, k_rep, rr, step_size, max_iters, goal_threashold)
    apf.generate_path()
    if apf.is_path_plan_success:
        path = apf.path
        path_ = []
        i = int(step_size_ / step_size)
        while (i < len(path)):
            path_.append(path[i])
            i += int(step_size_ / step_size)

        if path_[-1] != path[-1]:  # 添加最后一个点
            path_.append(path[-1])
        print('planed path points:{}'.format(path_))
        print('path plan success')

    else:
        print('path plan failed')
