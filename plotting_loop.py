from utils.mpManager import MPManager
import time
from matplotlib import pyplot as plt
from utils.Obstacle import *
from matplotlib.patches import Circle as C
from matplotlib.patches import Rectangle as R
from IPython import display
from utils.const import *
from math import cos, sin, tan, pi
import numpy as np
from scipy.spatial.transform import Rotation as Rot

obstacles = [Circle(0,0.2,0.15)]


def rot_mat_2d(angle):
    """
    Create 2D rotation matrix from an angle

    Parameters
    ----------
    angle :

    Returns
    -------
    A 2D rotation matrix

    Examples
    --------
    >>> angle_mod(-4.0)


    """
    return Rot.from_euler('z', angle).as_matrix()[0:2, 0:2]

def plot_arrow(x, y, yaw, length=0.1, width=0.05, fc="r", ec="k"):
    """Plot arrow."""
    if not isinstance(x, float):
        for (i_x, i_y, i_yaw) in zip(x, y, yaw):
            plot_arrow(i_x, i_y, i_yaw)
    else:
        plt.arrow(x, y, length * cos(yaw), length * sin(yaw),
                  fc=fc, ec=ec, head_width=width, head_length=width, alpha=0.4)


def plot_car(x, y, yaw):
    car_color = '-k'
    c, s = cos(yaw), sin(yaw)
    rot = rot_mat_2d(-yaw)
    car_outline_x, car_outline_y = [], []
    for rx, ry in zip(CAR_VRX, CAR_VRY):
        converted_xy = np.stack([rx, ry]).T @ rot
        car_outline_x.append(converted_xy[0]+x)
        car_outline_y.append(converted_xy[1]+y)

    arrow_x, arrow_y, arrow_yaw = c * -0.1 + x, s * -0.1 + y, yaw
    plot_arrow(arrow_x, arrow_y, arrow_yaw)

    plt.plot(car_outline_x, car_outline_y, car_color)

def plotting_loop(manager_mp: MPManager):
    poses_local = []
    obstacle_data_local = []
    robot_data_local = []
    plan_local = []
    goal_data_local = []

    while True:
        time.sleep(1)

        poses_local_unstable = manager_mp.poses[:]
        robot_data_local_unstable = manager_mp.robot_data[:]
        plan_local_unstable = manager_mp.plan_mp[:]
        obstacle_data_local_unstable = manager_mp.obstacle_data[:]
        goal_data_local_unstable = manager_mp.goal_data[:]

        if (len(poses_local_unstable) > 0):
            poses_local= poses_local_unstable

        if (len(robot_data_local_unstable) > 0):
            robot_data_local = robot_data_local_unstable

        if (len(obstacle_data_local_unstable) > 0):
            obstacle_data_local = obstacle_data_local_unstable

        if (len(plan_local_unstable) >0):
            plan_local = plan_local_unstable
            
        if (len(goal_data_local_unstable) >0):
            goal_data_local = goal_data_local_unstable

        if (not(len(poses_local) > 0 and len(robot_data_local) > 0 and len(obstacle_data_local) > 0 and len(plan_local) > 0 and len(goal_data_local) > 0)):
            print("Failed to plot, empty data...")
            continue

        # Plot robot data
        plt.clf()
        ax = plt.gca()
        for obstacle in obstacles:
            if isinstance(obstacle,Circle):
                patch = C((obstacle.center[0], obstacle.center[1]), obstacle.radius, fill=False, edgecolor='blue')
            elif isinstance(obstacle,Rectangle):
                patch = R((obstacle.origin[0],obstacle.origin[1]),obstacle.width,obstacle.height,fill=True,edgecolor='pink')
            ax.add_patch(patch)

        #plt.subplot(1,3,1)
        # print(poses_local)
        plt.plot(np.array(poses_local)[:,0],np.array(poses_local)[:,1])
        # plt.plot(robot_data_local[0], robot_data_local[1],'k',marker='+')
        # plt.quiver(robot_data_local[0],robot_data_local[1],0.1*np.cos(robot_data_local[2]),0.1*np.sin(robot_data_local[2]))
        plot_car(robot_data_local[0],robot_data_local[1],robot_data_local[2])

        plt.plot(goal_data_local[0],goal_data_local[1],'x',markersize=5)
        plt.quiver(goal_data_local[0],goal_data_local[1],0.1*np.cos(goal_data_local[2]),0.1*np.sin(goal_data_local[2]))

        np_obstacle_data = np.array(obstacle_data_local)
        plt.plot(np_obstacle_data[:,0],np_obstacle_data[:,1],'ko',markersize=5)
        plt.xlim(-1,1)
        plt.ylim(-1,1)
        plt.xlabel('x-position (m)')
        plt.ylabel('y-position (m)')
        plt.grid()

        plan_x_data = []
        plan_y_data = []
        plan_th_data = []

        for i in range(len(plan_local)):
            plan_x_data = np.append(plan_x_data, plan_local[i][0])
            plan_y_data = np.append(plan_y_data, plan_local[i][1])
            plan_th_data = np.append(plan_th_data, plan_local[i][2])

        plt.quiver(plan_x_data,plan_y_data,0.1*np.cos(plan_th_data),0.1*np.sin(plan_th_data), color="r")

        #plt.plot(obstacles[:,0],obstacles[:,1],'bo',markersize=15,alpha=0.2)
        plt.xlim(-1,1)
        plt.ylim(-1,1)
        plt.xlabel('x-position (m)')
        plt.ylabel('y-position (m)')
        plt.grid()

        plt.pause(0.0001)
        plt.show(block=False)

        display.clear_output(wait=True)
        display.display(plt.gcf())
        
        print("loop 3")