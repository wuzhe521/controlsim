import math
import matplotlib.pyplot as plt
import numpy as np
# import imageio


# 车辆参数信息
L = 2.9                     # 轴距[m]
Lf = L / 2.0                # 车辆中心点到前轴的距离[m]
Lr = L - Lf                 # 车辆终点到后轴的距离[m]
W = 2.0                     # 宽度[m]
LF = 3.8                    # 后轴中心到车头距离[m]
LB = 0.8                    # 后轴中心到车尾距离[m]
TR = 0.5                    # 轮子半径[m]
TW = 0.5                    # 轮子宽度[m]
WD = W                      # 轮距[m]
Iz = 2250.0                 # 车辆绕z轴的转动惯量[kg/m2]
Cf = 1600.0 * 2.0           # 前轮侧偏刚度[N/rad]
Cr = 1700.0 * 2.0           # 后轮侧偏刚度[N/rad]
m = 1500.0                  # 车身质量[kg]
LENGTH = LB + LF            # 车辆长度[m]
MWA = np.radians(30.0)      # 最大轮转角(Max Wheel Angle)[rad]


def normalize_angle(angle):
    a = math.fmod(angle + np.pi, 2 * np.pi)
    if a < 0.0:
        a += (2.0 * np.pi)
    return a - np.pi


class DynamicBicycleModel:

    """
    动态自行车模型类，用于模拟车辆的运动学动力学特性
    该模型考虑了车辆的纵向速度、侧向速度、横摆角速度等状态变量
    """
    def __init__(self, x=0.0, y=0.0, yaw=0.0, vx=0.01, vy=0.0, omega=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.vx = vx
        self.vy = vy
        self.omega = omega
        self.delta = 0.0

    def update(self, a, delta, dt=0.1):
        # 限制转向角在最大转向角范围内
        self.delta = np.clip(delta, -MWA, MWA)
        # 更新车辆位置(x, y)
        # 使用当前速度和航向角计算新的x坐标
        self.x = self.x + self.vx * math.cos(self.yaw) * dt - self.vy * math.sin(self.yaw) * dt
        # 使用当前速度和航向角计算新的y坐标
        self.y = self.y + self.vx * math.sin(self.yaw) * dt + self.vy * math.cos(self.yaw) * dt
        # 更新航向角(yaw)
        self.yaw = self.yaw + self.omega * dt
        # 将航向角标准化到[-pi, pi]范围内
        self.yaw = normalize_angle(self.yaw)
        # 计算前轮和后轮的侧向力
        # 计算前轮侧向力，考虑侧偏角和转向角
        f_cf = Cf * normalize_angle(self.delta - math.atan2((self.vy + Lf * self.omega) / self.vx, 1.0))
        # 计算后轮侧向力，考虑侧偏角
        f_cr = Cr * math.atan2((Lr * self.omega-self.vy) / self.vx, 1.0)

        
        # 计算前轮和后轮的纵向力和侧向力分量
        f_xf = f_cf * math.sin(self.delta)  # 前轮纵向力
        f_yf = f_cf * math.cos(self.delta)  # 前轮侧向力
        f_yr = f_cr  # 后轮侧向力
        # 更新车辆速度和角速度
        # 更新x方向速度(纵向速度)
        self.vx = self.vx + (a - f_xf / m + self.vy * self.omega) * dt
        # 更新y方向速度(侧向速度)
        self.vy = self.vy + ((f_yr + f_yf) / m - self.vx * self.omega) * dt
        # 更新角速度(横摆角速度)
        self.omega = self.omega + (Lf * f_yf - f_yr * Lr) / Iz * dt


def draw_vehicle(x, y, yaw, delta, ax, color='black'):
    vehicle_outline = np.array(
        [[-LB, LF, LF, -LB, -LB],
         [W / 2, W / 2, -W / 2, -W / 2, W / 2]])

    wheel = np.array([[-TR, TR, TR, -TR, -TR],
                      [TW / 2, TW / 2, -TW / 2, -TW / 2, TW / 2]])

    rr_wheel = wheel.copy()  # 右后轮
    rl_wheel = wheel.copy()  # 左后轮
    fr_wheel = wheel.copy()  # 右前轮
    fl_wheel = wheel.copy()  # 左前轮
    rr_wheel[1, :] += WD/2
    rl_wheel[1, :] -= WD/2

    # 方向盘旋转
    rot1 = np.array([[np.cos(delta), -np.sin(delta)],
                     [np.sin(delta), np.cos(delta)]])
    # yaw旋转矩阵
    rot2 = np.array([[np.cos(yaw), -np.sin(yaw)],
                     [np.sin(yaw), np.cos(yaw)]])
    fr_wheel = np.dot(rot1, fr_wheel)
    fl_wheel = np.dot(rot1, fl_wheel)
    fr_wheel += np.array([[L], [-WD / 2]])
    fl_wheel += np.array([[L], [WD / 2]])

    fr_wheel = np.dot(rot2, fr_wheel)
    fr_wheel[0, :] += x
    fr_wheel[1, :] += y
    fl_wheel = np.dot(rot2, fl_wheel)
    fl_wheel[0, :] += x
    fl_wheel[1, :] += y
    rr_wheel = np.dot(rot2, rr_wheel)
    rr_wheel[0, :] += x
    rr_wheel[1, :] += y
    rl_wheel = np.dot(rot2, rl_wheel)
    rl_wheel[0, :] += x
    rl_wheel[1, :] += y
    vehicle_outline = np.dot(rot2, vehicle_outline)
    vehicle_outline[0, :] += x
    vehicle_outline[1, :] += y

    ax.plot(fr_wheel[0, :], fr_wheel[1, :], color)
    ax.plot(rr_wheel[0, :], rr_wheel[1, :], color)
    ax.plot(fl_wheel[0, :], fl_wheel[1, :], color)
    ax.plot(rl_wheel[0, :], rl_wheel[1, :], color)

    ax.plot(vehicle_outline[0, :], vehicle_outline[1, :], color)
    # ax.axis('equal')


if __name__ == "__main__":

    vehicle = DynamicBicycleModel(0.0, 0.0, 0.0, 1.0, 1.0, 0.0)
    trajectory_x = []
    trajectory_y = []
    fig = plt.figure()

    # 保存动图用
    # i = 0
    # image_list = []  # 存储图片
    plt.figure(1)
    for i in range(500):
        plt.cla()
        plt.gca().set_aspect('equal', adjustable='box')
        vehicle.update(0, np.pi / 10)
        draw_vehicle(vehicle.x, vehicle.y, vehicle.yaw, vehicle.delta, plt)
        trajectory_x.append(vehicle.x)
        trajectory_y.append(vehicle.y)
        plt.plot(trajectory_x, trajectory_y, 'blue')
        plt.xlim(-15, 12)
        plt.ylim(-2.5, 21)
        plt.pause(0.001)
    #     i += 1
    #     if (i % 5) == 0:
    #         plt.savefig("temp.png")
    #         image_list.append(imageio.imread("temp.png"))
    #
    # imageio.mimsave("display.gif", image_list, duration=0.1)
