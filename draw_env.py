from vehicle_model import vehicle_model
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches


def left_lane(x):
    return 1.75 + 0.02 * x - 0.001 * x**2


def right_lane(x):
    return -1.75 + 0.02 * x - 0.001 * x**2


def draw_lanes():
    left_lane_points = [(x, left_lane(x)) for x in range(0, 50)]
    right_lane_points = [(x, right_lane(x)) for x in range(0, 50)]
    return left_lane_points, right_lane_points


def Coordinate_transform(x, y, x0, y0, angle):
    rotate_x = (x) * np.cos(angle) + (y) * np.sin(angle)
    rotate_y = -(x) * np.sin(angle) + (y) * np.cos(angle)
    tran_x  = rotate_x + x0
    tran_y = rotate_y + y0
    return tran_x, tran_y

if __name__ == "__main__":
    left_lane_points, right_lane_points = draw_lanes()

    E0Y = vehicle_model("E0Y", 0.0, 0.0, 10.0, 0.5, -5, -5)

    fig, ax = plt.subplots()
    # set x-axis from -10 to 10
    ax.set_xlim(-10, 100)
    ax.set_ylim(-10, 10)
    ax.set_aspect("equal")
    
    plt.ion()  # 开启 交互模式
    
    points = E0Y.position()
    print(points)
    rect = patches.Polygon(
        points, linewidth=0.1, edgecolor="blue", facecolor="red", alpha=1
    )
    ax.add_patch(rect)
    x = np.linspace(0, 100, 10)
    ly = left_lane(x)
    ry = right_lane(x)
    g_lyX, g_lyY = Coordinate_transform(x, ly, E0Y.X, E0Y.Y, -E0Y.angle)
    g_ryX, g_ryY = Coordinate_transform(x, ry, E0Y.X, E0Y.Y, -E0Y.angle)
    plt.scatter(g_lyX, g_lyY, s=1)
    plt.scatter(g_ryX, g_ryY, s=1)
    
    for i in range(10):
        rect = patches.Polygon(
            E0Y.position(), linewidth=2, edgecolor="blue", facecolor="lightblue", alpha=0.7
        )
        ax.add_patch(rect)
        E0Y.kinematic_Update(0, 0.25)
        plt.pause(0.1)
    # 关闭交互模式
    plt.ioff()
    plt.show()
