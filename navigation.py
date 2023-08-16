import numpy as np
from matplotlib import pyplot as plt
from IPython import display

#from RRTC import RRTC
from Obstacle import *
from matplotlib.patches import Circle as C
import math

class DiffDriveRobot:

    def __init__(self, inertia=5, dt=0.1, drag=0.2, wheel_radius=0.05, wheel_sep=0.15):

        # States
        self.x = 0.0  # x-position
        self.y = 0.0  # y-position
        self.th = 0.0  # orientation

        # Wheel States (Wheel Velocity, Inputs)
        self.wl = 0.0  # rotational velocity left wheel
        self.wr = 0.0  # rotational velocity right wheel

        # Constants
        self.I = inertia
        self.d = drag
        self.dt = dt

        self.r = wheel_radius
        self.l = wheel_sep

    # Should be replaced by motor encoder measurement which measures how fast wheel is turning
    # Here, we simulate the real system and measurement
    def motor_simulator(self, w, duty_cycle):

        torque = self.I * duty_cycle

        if (w > 0):
            w = min(w + self.dt * (torque - self.d * w), 3)
        elif (w < 0):
            w = max(w + self.dt * (torque - self.d * w), -3)
        else:
            w = w + self.dt * (torque)

        return w

    # Veclocity motion model
    def base_velocity(self, wl, wr):

        v = (wl * self.r + wr * self.r) / 2.0

        w = (wl * self.r - wr * self.r) / self.l

        return v, w

    # Kinematic motion model
    def pose_update(self, duty_cycle_l, duty_cycle_r):

        self.wl = self.motor_simulator(self.wl, duty_cycle_l)
        self.wr = self.motor_simulator(self.wr, duty_cycle_r)

        v, w = self.base_velocity(self.wl, self.wr)

        self.x = self.x + self.dt * v * np.cos(self.th)
        self.y = self.y + self.dt * v * np.sin(self.th)
        self.th = self.th + w * self.dt

        return self.x, self.y, self.th

class RobotController:
    
    def __init__(self,Kp=0.1,Ki=0.01,wheel_radius=0.02, wheel_sep=0.1):
        
        self.Kp = Kp
        self.Ki = Ki
        self.r = wheel_radius
        self.l = wheel_sep
        self.e_sum_l = 0
        self.e_sum_r = 0
        
    def p_control(self,w_desired,w_measured,e_sum):
        
        duty_cycle = min(max(-1,self.Kp*(w_desired-w_measured) + self.Ki*e_sum),1)
        
        e_sum = e_sum + (w_desired-w_measured)
        
        return duty_cycle, e_sum
        
        
    def drive(self,v_desired,w_desired,wl,wr):
        
        wl_desired = (v_desired + self.l*w_desired/2)/self.r
        wr_desired = (v_desired - self.l*w_desired/2)/self.r
        
        duty_cycle_l,self.e_sum_l = self.p_control(wl_desired,wl,self.e_sum_l)
        duty_cycle_r,self.e_sum_r = self.p_control(wr_desired,wr,self.e_sum_r)
        
        return duty_cycle_l, duty_cycle_r
    
class TentaclePlanner:
    
    def __init__(self,dt=0.1,steps=5,alpha=1,beta=0.1):
        
        self.dt = dt
        self.steps = steps
        # Tentacles are possible trajectories to follow
        self.tentacles = [(0.0,1.0),(0.0,-1.0),(0.1,1.0),(0.1,-1.0),(0.1,0.5),(0.1,-0.5),(0.1,0.0),(-0.1,1.0),(-0.1,-1.0),(-0.1,0.5),(-0.1,-0.5),(0.1,0.0),(-0.1,0.0),(0.0,0.0)]
        
        self.alpha = alpha
        self.beta = beta
    
    # Play a trajectory and evaluate where you'd end up
    def roll_out(self,v,w,goal_x,goal_y,goal_th,x,y,th, obstacles):
        
        for j in range(self.steps):
        
            x = x + self.dt*v*np.cos(th)
            y = y + self.dt*v*np.sin(th)
            th = (th + w*self.dt)
            
            if (self.check_collision(x,y,obstacles)):
                return np.inf
        
        # Wrap angle error -pi,pi
        e_th = goal_th-th
        e_th = np.arctan2(np.sin(e_th),np.cos(e_th))
        
        cost = self.alpha*((goal_x-x)**2 + (goal_y-y)**2) + self.beta*(e_th**2)
        
        return cost
    
    def check_collision(self,x,y,obstacles):
        min_dist = np.min(np.sqrt((x-obstacles[:,0])**2+(y-obstacles[:,1])**2))
        
        if (min_dist < 0.05):
            return True
        return False
        
    
    # Choose trajectory that will get you closest to the goal
    def plan(self,goal_x,goal_y,goal_th,x,y,th,obstacles):
        
        costs =[]
        for v,w in self.tentacles:
            costs.append(self.roll_out(v,w,goal_x,goal_y,goal_th,x,y,th,obstacles))
        
        best_idx = np.argmin(costs)
        best_costs = np.min(costs)

        print(self.tentacles[best_idx])

        return self.tentacles[best_idx], best_costs

# This is NEW! (We coded this)
class Map:
    def __init__(self, width = 2, height = 2, true_obstacles = []):
        # Constants
        self.width = width
        self.height = height
        self.obstacle_dots = None
        self.true_obstacles = true_obstacles
        self.obstacle_size = 0.05
        
        self.initialize = True

    def update(self, robot_x, robot_y, robot_th):
        # Ultrasonic Distance
        ultrasonic_distance = self.check_ultrasonic(robot_x, robot_y, robot_th)
        
        # Generate obstacles
        self.generate_obstacle(robot_x, robot_y, robot_th, ultrasonic_distance)

    # Simulation
    def check_ultrasonic(self, robot_x, robot_y, robot_th):
        # Draw a line from robot to check true_obstacles
        curr_x = robot_x
        curr_y = robot_y
        distance = 0
        increment = 0.01
        hit = False
        
        while (not hit):
            # Increment curr_x and curr_y
            curr_x = curr_x + np.cos(robot_th) * increment
            curr_y = curr_y + np.sin(robot_th) * increment
            distance = distance + increment

            # Check curr_x and curr_y
            min_dist = np.min(np.sqrt((curr_x-self.true_obstacles[:,0])**2+(curr_y-self.true_obstacles[:,1])**2))
            if (min_dist < self.obstacle_size
                or np.abs(curr_x) > self.width / 2
                or np.abs(curr_y) > self.height / 2):
                # print(min_dist)
                # print(curr_x)
                # print(curr_y)
                # print(distance)
                hit = True

        return distance
    
    def generate_obstacle(self, robot_x, robot_y, robot_th, distance):
        # Append to Obstacle_dots
        obs_x = robot_x + distance * np.cos(robot_th)
        obs_y = robot_y + distance * np.sin(robot_th)

        point = np.array([[obs_x, obs_y]])

        if (self.initialize):
            self.obstacle_dots = np.array(point)
            self.initialize = False
        else:
            # print(self.obstacle_dots)
            # print(point)
            self.obstacle_dots= np.concatenate((self.obstacle_dots, point), axis=0)

        # Get rid of any obstacles in line of sight 
        # TODO


obstacles = 2*np.random.rand(20,2)-1
robot = DiffDriveRobot(inertia=10, dt=0.1, drag=2, wheel_radius=0.05, wheel_sep=0.15)
controller = RobotController(Kp=1.0,Ki=0.15,wheel_radius=0.05,wheel_sep=0.15)
planner = TentaclePlanner(dt=0.1,steps=10,alpha=1,beta=1e-9)
map = Map(1.5, 1.5, obstacles)
# print(type(obstacles))
plt.figure(figsize=(15,9))

poses = []
velocities = []
duty_cycle_commands = []
costs_vec = []

goal_x = 1.5*np.random.rand()- 1.5/2
goal_y = 1.5*np.random.rand()- 1.5/2
goal_th = 2*np.pi*np.random.rand()-np.pi

goal = np.array([goal_x, goal_y, goal_th])
start = np.array([robot.x, robot.y, robot.th])

#rrtc = RRTC(start = start, goal=goal, obstacle_list=obstacles, width=map.width, height = map.height, expand_dis=0.2, path_resolution=0.05)
#plan = rrtc.planning()

# Needs plotting on map:
# Map Size
# Mapped Obstacles (map.obstacles)
# True Obstacles in different color (obstacles)
# Goal
# Start
# Position
# RRT Plan (rrt_plan)
# Unknown vs Known map thing? (Stretch goal)

all_obstacles = []
for i in range(19):
    all_obstacles.append(Circle(np.random.rand(1)-1, np.random.rand(1)-1, 0.05))

for i in range(200):
    # Map Generation for obstacles
    map.update(robot.x, robot.y, robot.th)

    # Example motion using controller 
    tentacle,cost = planner.plan(goal_x,goal_y,goal_th,robot.x,robot.y,robot.th, map.obstacle_dots)
    v,w=tentacle
    
    duty_cycle_l,duty_cycle_r = controller.drive(v,w,robot.wl,robot.wr)
    
    # Simulate robot motion - send duty cycle command to controller
    x,y,th = robot.pose_update(duty_cycle_l,duty_cycle_r)
    
    # Log data
    poses.append([x,y,th])
    duty_cycle_commands.append([duty_cycle_l,duty_cycle_r])
    velocities.append([robot.wl,robot.wr])
    costs_vec.append(cost)
    
    # Plot robot data
    plt.clf()
    ax = plt.gca()
    for obstacle in all_obstacles:
        circle = C((obstacle.center[0], obstacle.center[1]), obstacle.radius, fill=False, edgecolor='blue')
        ax.add_patch(circle)


    #plt.subplot(1,3,1)
    plt.plot(np.array(poses)[:,0],np.array(poses)[:,1])
    plt.plot(x,y,'k',marker='+')
    plt.quiver(x,y,0.1*np.cos(th),0.1*np.sin(th))
    plt.plot(goal_x,goal_y,'x',markersize=5)
    plt.quiver(goal_x,goal_y,0.1*np.cos(goal_th),0.1*np.sin(goal_th))

    plt.plot(map.obstacle_dots[:,0],map.obstacle_dots[:,1],'ko',markersize=5)
    plt.xlim(-1,1)
    plt.ylim(-1,1)
    plt.xlabel('x-position (m)')
    plt.ylabel('y-position (m)')
    plt.grid()

    #plt.subplot(1,3,2)
    plt.plot(np.array(poses)[:,0],np.array(poses)[:,1])
    plt.plot(x,y,'k',marker='+')
    plt.quiver(x,y,0.1*np.cos(th),0.1*np.sin(th))
    plt.plot(goal_x,goal_y,'x',markersize=5)
    plt.quiver(goal_x,goal_y,0.1*np.cos(goal_th),0.1*np.sin(goal_th))


    #plt.plot(obstacles[:,0],obstacles[:,1],'bo',markersize=15,alpha=0.2)
    plt.xlim(-1,1)
    plt.ylim(-1,1)
    plt.xlabel('x-position (m)')
    plt.ylabel('y-position (m)')
    plt.grid()
    
    #plt.subplot(3,3,3)
    #plt.plot(np.arange(i+1)*robot.dt,np.array(duty_cycle_commands))
    #plt.xlabel('Time (s)')
    #plt.ylabel('Duty cycle')
    #plt.grid()
    
    #plt.subplot(3,3,6)
    #plt.plot(np.arange(i+1)*robot.dt,np.array(velocities))
    #plt.xlabel('Time (s)')
    #plt.ylabel('Wheel $\omega$')
    #plt.legend(['Left wheel', 'Right wheel'])
    #plt.grid()

    #plt.subplot(3,3,9)
    #plt.plot(np.arange(i+1)*robot.dt,np.array(costs_vec))
    #plt.xlabel('Time (s)')
    #plt.ylabel('Costs')
    #plt.grid()

    plt.pause(0.05)
    plt.show(block=False)
    
    
    display.clear_output(wait=True)
    display.display(plt.gcf())
    
    