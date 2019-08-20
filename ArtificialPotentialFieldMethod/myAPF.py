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

def get_info(info, receive):
    obstacles = []
    for index in range(len(info)):
        receive.get_info(info[index][0], info[index][1])
        x, y = receive.robot_info['x'], receive.robot_info['y']
        obstacles.append([x, y])
    return obstacles


class APF():
    """
    人工势场寻路
    """

    def __init__(self, s_x, s_y, g_x, g_y, info, receive, k_att=3, k_rep=8000, rr=80,
                 step_size=10, max_iters=500, goal_threshold=10, att_threshold=50):
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
        self.start = Vector2d(s_x, s_y)
        self.current_pos = Vector2d(s_x, s_y)
        self.goal = Vector2d(g_x, g_y)
        obstacles = get_info(info, receive)
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
                pass
            else:
                rep_1 = Vector2d(obs_to_rob.direction[0], obs_to_rob.direction[1]) * self.k_rep * (
                        1.0 / obs_to_rob.length - 1.0 / self.rr) / (obs_to_rob.length ** 2) * (rob_to_goal.length ** 2)
                rep_2 = Vector2d(rob_to_goal.direction[0], rob_to_goal.direction[1]) * self.k_rep * ((1.0 / obs_to_rob.length - 1.0 / self.rr) ** 2) * rob_to_goal.length
                rep +=(rep_1+rep_2)
        return rep

    def Generate_Path(self):
        """
        path plan
        :return:
        """
        self.is_path_plan_success = False
        while (self.iters < self.max_iters and (self.current_pos - self.goal).length > self.goal_threashold):
            f_vec = self.attractive() + self.repulsion()
            self.current_pos += Vector2d(f_vec.direction[0], f_vec.direction[1]) * self.step_size
            self.iters += 1
            self.path.append([self.current_pos.deltaX, self.current_pos.deltaY])

        if (self.current_pos - self.goal).length <= self.goal_threashold:
            self.is_path_plan_success = True
            return self.is_path_plan_success, [], []

    def Get_Path(self):
        # get the final path, a list of points and a list of lines, from start to end
        path_lines = []
        for i in range(len(self.path) - 1):
            x, y = self.path[i].copy()
            x_, y_ = self.path[i + 1].copy()
            path_lines.append([x, y, x_, y_])
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
