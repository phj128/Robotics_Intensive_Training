import numpy as np
import time
from message.send import Send
from message.send_debug import SendDebug
from message.receive import Receive



class DynamicWindow:
    def __init__(self, start_x, start_y, start_ori, goal_x, goal_y, barrierID, receive, dt = 1, inflateRaius = 20):
        self.dt = dt
        self.inflateRadius = inflateRaius
        self.Kinetic_model = [200, -200, 3, 100, 1, 5, 0.1] # 最高速度vx，最小速度，最高转速，最高加速度，最高转加速度，速度分辨率，角速度分辨率
        self.status = np.array([start_x, start_y, start_ori, 0, 0]) # 初期状态
        self.evalParam = np.array([6.8, 0.8]) # 目标代价增益，速度代价增益
        self.prediction_time = 5 # 向前预估三秒
        self.goal = [goal_x, goal_y]
        self.barrierId = barrierID
        self.barrierInfo = np.zeros((len(self.barrierId), 5))  # x, y, r, v_x, v_y
        self.receive = receive
        self.Update_Barrier_Info()  # update the information of barriers
        self.maxtry = 1000

    def Update_Barrier_Info(self):
        #只需要barrierTd不包含自身ID即可，？？？可能包含也可以
        receive = self.receive

        for index in range(len(self.barrierId)):
            receive.get_info(self.barrierId[index][0],self.barrierId[index][1])
            self.barrierInfo[index][0] = receive.robot_info['x']
            self.barrierInfo[index][1] = receive.robot_info['y']

    def motion(self, x, u, dt):
        x[0] = x[0] + u[0] * np.cos(x[2]) * dt # x方向位移
        x[1] = x[1] + u[0] * np.sin(x[2]) * dt # y方向位移
        x[2] = x[2] + u[1] * dt # 朝向角
        x[3] = u[0] # 速度
        x[4] = u[1] # 角速度
        return x

    def calc_dynamic_window(self, x):
        '''
        位置空间集合
        :param x: 位置速度状态
        :return: 返回速度的交集
        '''
        #车辆能达到的最小最大速度，最小最大转速
        vs = [self.Kinetic_model[1], self.Kinetic_model[0], -self.Kinetic_model[2], self.Kinetic_model[2]]
        #一个采样周期内最大最小变化
        vd = [x[3]-self.Kinetic_model[3]*self.dt, x[3]+self.Kinetic_model[3]*self.dt,
              x[4]-self.Kinetic_model[4]*self.dt, x[4]+self.Kinetic_model[4]*self.dt]
        vr = [np.maximum(vs[0], vd[0]), np.minimum(vs[1], vd[1]), np.maximum(vs[2], vd[2]), np.minimum(vs[3], vd[3])]
        return vr

    def calc_trajectory(self, x_init, v, w):
        '''
        预测prediction time中的轨迹
        :param x: 运动模型，五个元素，x方向位移，y方向位移，朝向，速度，角速度
        :param v: 
        :param w: 
        :return: 
        '''
        x = np.array(x_init)
        trajectory = np.array(x)
        time = 0
        while time <= self.prediction_time:
            x = self.motion(x, [v,w], self.dt)
            trajectory = np.vstack((trajectory, x)) # 垂直堆叠，新的在上面
            time = time + self.dt
        return trajectory

    def calc_cost_to_goal(self, trajectory):
        '''
        得到轨迹到目标点的代价
        :param trajectory: x的堆叠，最上面是最新的
        :return: 轨迹到目标点的欧式距离
        '''
        dx = self.goal[0] - trajectory[-1, 0]
        dy = self.goal[1] - trajectory[-1, 1]
        goal_dist = np.sqrt(dx*dx + dy*dy)
        cost = self.evalParam[0]*goal_dist
        return cost

    def calc_cost_to_obstacle(self, trajectory):
        '''
        计算预测到障碍物最小距离
        :return: 返回值越小越好，返回0说明不碰撞
        '''
        min_r = float("inf")
        for ii in range(0, len(trajectory[:, 1])): # 轨迹上的y方向位移
            for i in range(len(self.barrierInfo)):
                ox = self.barrierInfo[i][0]
                oy = self.barrierInfo[i][1]
                dx = trajectory[ii, 0] - ox
                dy = trajectory[ii, 1] - oy
                r = np.sqrt(dx**2 + dy**2)
                if r <= self.inflateRadius:
                    return float("inf") #碰撞
                if min_r >= r:
                    min_r = r
        return 1.0/min_r  # 越小越好

    def calc_final_input(self, x, u, vr):
        '''
        计算采样空间评价函数，选择最合适的一个作为输出的
        :param x: 位置空间
        :param u: 速度空间
        :param vr: 速度空间交集
        :return: 
        '''
        x_init = x[:]
        min_cost = 100000.0
        min_u = u
        best_trajectory = np.array([x])
        for v in np.arange(vr[0], vr[1], self.Kinetic_model[5]):
            for w in np.arange(vr[2], vr[3], self.Kinetic_model[6]):
                trajectory = self.calc_trajectory(x_init, v, w)
                goal_cost = self.calc_cost_to_goal(trajectory)
                speed_cost = self.evalParam[1]*(self.Kinetic_model[0] - trajectory[-1,3])
                obstacle_cost = self.calc_cost_to_obstacle(trajectory)
                final_cost = goal_cost + speed_cost + obstacle_cost

                if min_cost >= final_cost:
                    min_cost = final_cost
                    min_u = [v,w]
                    best_trajectory = trajectory

        return min_u, best_trajectory

    def DWA_control(self, x, u):
        vr = self.calc_dynamic_window(x)
        u, trajectory = self.calc_final_input(x,u,vr)
        return u, trajectory

    def Generate_control(self):
        x = self.status # 初始化位置空间
        u = np.array([0.0,0.0])
        trajectory = np.array(x)

        for i in range(self.maxtry):
            u, best_trajectory = self.DWA_control(x, u)
            x = self.motion(x, u, self.dt)
            trajectory = np.vstack((trajectory,x))

            if np.sqrt((x[0]-self.goal[0])**2 + (x[1]-self.goal[1])**2) <= 20:
                break

        if i == self.maxtry-1:
            print('cant find !')

        return trajectory

if __name__ == '__main__':
    color = 'blue'
    robot_id = 0
    receive = Receive()
    receive.get_info(color, robot_id)
    start_x = receive.robot_info['x']
    start_y = receive.robot_info['y']
    start_ori = receive.robot_info['ori']
    time_start = time.time()
    my_dwa = DynamicWindow(start_x, start_y, start_ori, 200, 200, [['yellow', 0], ['yellow', 1], ['yellow', 2], ['yellow', 3], ['yellow', 4], ['yellow', 5], ['yellow', 6], ['yellow', 7]], receive)
    trajectory = my_dwa.Generate_control()
    time_end = time.time()
    print('time_cost:', time_end-time_start)
    print(trajectory)



