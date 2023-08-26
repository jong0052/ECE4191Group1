import numpy as np
from matplotlib import pyplot as plt
from IPython import display

from RRTC import RRTC
from Obstacle import *
from matplotlib.patches import Circle as C
from matplotlib.patches import Rectangle as R
import math
import serial
import time

class DiffDriveRobot:

    def __init__(self, inertia=5, dt=0.1, drag=0.2, wheel_radius=0.05, wheel_sep=0.15,x=0,y=0,th=0):

        # States
        self.x = x  # x-position
        self.y = y  # y-position
        self.th = th  # orientation

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
    def pose_update(self, duty_cycle_l = 0, duty_cycle_r = 0, wl = 0, wr = 0):

        if (simulation):
            self.wl = self.motor_simulator(self.wl, duty_cycle_l)
            self.wr = self.motor_simulator(self.wr, duty_cycle_r)
        else:
            self.wl = wl
            self.wr = wr

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
        
        wl_goal = (v_desired + self.l*w_desired/2)/self.r
        wr_goal = (v_desired - self.l*w_desired/2)/self.r
        
        duty_cycle_l,self.e_sum_l = self.p_control(wl_goal,wl,self.e_sum_l)
        duty_cycle_r,self.e_sum_r = self.p_control(wr_goal,wr,self.e_sum_r)
        
        return duty_cycle_l, duty_cycle_r, wl_goal, wr_goal
    
class TentaclePlanner:
    
    def __init__(self,dt=0.1,steps=5,alpha=1,beta=0.1, reverse=False):
        
        self.dt = dt
        self.steps = steps
        # Tentacles are possible trajectories to follow
        if (reverse):
            self.tentacles = [(0.0,1.0),(0.0,-1.0),(0.1,1.0),(0.1,-1.0),(0.1,0.5),(0.1,-0.5),(0.1,0.0),(0.0,0.0),(-0.1,1.0),(-0.1,-1.0),(-0.1,0.5),(-0.1,-0.5),(-0.1,0.0)]
        else:
            #self.tentacles = [(0.0,1.0),(0.0,-1.0),(0.1,1.0),(0.1,-1.0),(0.1,0.5),(0.1,-0.5),(0.1,0.0),(0.0,0.0)]
            self.tentacles = []
            for v in range(0,11, 1):
                for w in range(-10,11, 1):
                    self.tentacles.append((v/100, w/10))

            # print(self.tentacles)
  
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

        # print(self.tentacles[best_idx])

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

        if simulation:
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
                # min_dist = np.min(np.sqrt((curr_x-self.true_obstacles[:,0])**2+(curr_y-self.true_obstacles[:,1])**2))
                if (not self.is_collision_free(curr_x, curr_y)
                    or np.abs(curr_x) > self.width / 2
                    or np.abs(curr_y) > self.height / 2):
                    # print(min_dist)
                    # print(curr_x)
                    # print(curr_y)
                    # print(distance)
                    hit = True
        else:
            # TODO: Ultrasonic Reader
            distance = 0

        return distance
    
    def is_collision_free(self, x, y):
        """
        Determine if nearby_node (new_node) is in the collision-free space.
        """
        points = np.vstack((x, y)).T
        for obs in self.true_obstacles:
            in_collision = obs.is_in_collision_with_points(points)
            if in_collision:
                return False
        
        return True  # safe

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

    def get_obstacle_list(self):
        obstacle_list = []

        if (not self.initialize):
            for dot in self.obstacle_dots:
                obstacle_list.append(Circle(dot[0], dot[1], self.obstacle_size))

        return obstacle_list

class Serializer:
    def __init__(self):
        self.ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
        self.ser.reset_input_buffer()
        self.data = SerialData()

    def read(self):
        line = self.ser.readline().decode('utf-8').rstrip()
        serial_data = self.decode_string(line)

        return serial_data
    
    def write(self):
        self.ser.write(self.encode_string())

    def decode_string(self,input_string):
        # Extracting numbers from the input_string using string manipulation
        # Line Format: "Wheels: [wl, wr]"
        if not input_string.startswith("Wheels"):
            raise ValueError("Input string must start with 'Wheels'")
    
        try:
            start_idx = input_string.index("[") + 1
            end_idx = input_string.index("]")
            wl_current, wr_current, wl_goal, wr_goal = map(int, input_string[start_idx:end_idx].split(','))
        except ValueError:
            raise ValueError("Invalid input format")

        # Creating a SerialData object with the extracted numbers
        self.data.update(wl_current=wl_current, wr_current=wr_current, wl_goal=wl_goal, wr_goal=wr_goal)
        return self.data
    
    def encode_string(self):
        return f"Wheels: [{self.data.wl_current},{self.data.wr_current},{self.data.wl_goal},{self.data.wr_goal}]".encode("utf-8")

class SerialData:
    """
    Data for Serial Connection between Arduino and PI

    All Velocities in RPM
    """
    def __init__(self, wl_current=0, wr_current=0, wl_goal=0, wr_goal=0):
        self.update(wl_current, wr_current, wl_goal, wr_goal)

    def update(self, wl_current=None, wr_current=None, wl_goal=None, wr_goal=None):
        if wl_current is not None:
            self.wl_current = wl_current
        if wr_current is not None:
            self.wr_current = wr_current
        if wl_goal is not None:
            self.wl_goal = wl_goal
        if wr_goal is not None:
            self.wr_goal = wr_goal

if __name__ == '__main__':
    obstacles = [Rectangle((-0.4,0),0.9,0.1)]
    #obstacles = [Circle(0.5,0.5,0.05),Circle(-0.5, -0.5, 0.05), Circle(-0.5, 0.5, 0.05), Circle(0.5, -0.5, 0.05)]
    robot = DiffDriveRobot(inertia=10, dt=0.1, drag=2, wheel_radius=0.05, wheel_sep=0.15,x=-0.4,y=-0.4,th=0)
    controller = RobotController(Kp=1.0,Ki=0.15,wheel_radius=0.05,wheel_sep=0.15)
    tentaclePlanner = TentaclePlanner(dt=0.1,steps=10,alpha=1,beta=1e-9)
    map = Map(1.5, 1.5, obstacles)

    plt.figure(figsize=(15,9))

    poses = []
    velocities = []
    duty_cycle_commands = []
    costs_vec = []

    goal_x = 0.6
    goal_y = 0.6
    goal_th = 0

    final_goal = np.array([goal_x, goal_y, goal_th])
    start = np.array([robot.x, robot.y, robot.th])
    expand_dis = 0.05
    path_resolution = 0.01
    rrtc = RRTC(start = start, goal=final_goal, obstacle_list=map.get_obstacle_list(), width=map.width, height = map.height, expand_dis=expand_dis, path_resolution=path_resolution, max_points=200)
    rrt_plan = rrtc.planning()
    rrt_plan_index = 0

    fail_combo = 0

    plotting = True
    simulation = True
    
    if (not simulation):
        serializer = Serializer()

    while True:
        # Map Generation for obstacles
        map.update(robot.x, robot.y, robot.th)

        temp_goal = rrt_plan[rrt_plan_index]

        # print(map.obstacle_dots)

        # Example motion using controller 
        tentacle,cost = tentaclePlanner.plan(temp_goal[0],temp_goal[1],temp_goal[2],robot.x,robot.y,robot.th, map.obstacle_dots)
        v,w=tentacle
        
        duty_cycle_l,duty_cycle_r,wl_goal,wr_goal = controller.drive(v,w,robot.wl,robot.wr)
        
        # Simulate robot motion - send duty cycle command to controller
        x,y,th = robot.pose_update(duty_cycle_l,duty_cycle_r,wl_goal,wr_goal)
        
        # Have we reached temp_goal?
        if (cost < 1e-2):
            if (rrt_plan_index < len(rrt_plan) - 1):
                rrt_plan_index += 1
                # print("New Goal: ")
                # print(rrt_plan[rrt_plan_index])

        # Is car going to get stuck? Replan.
        if (v == 0 and w == 0):
            fail_combo += 1

        rrtc.obstacle_list = map.get_obstacle_list()

        if (not rrtc.is_collision_free_path(rrt_plan)):
            final_goal = np.array([goal_x, goal_y, goal_th])
            start = np.array([robot.x, robot.y, robot.th])

            # print("why are u runnin")
            rrtc = RRTC(start = start, goal=final_goal, obstacle_list=map.get_obstacle_list(), width=map.width, height = map.height, expand_dis=expand_dis, path_resolution=path_resolution)
            new_rrt_plan = rrtc.planning()

            if (len(new_rrt_plan) > 0):
                rrt_plan = new_rrt_plan
                rrt_plan_index = 0

        # Log data
        poses.append([x,y,th])
        duty_cycle_commands.append([duty_cycle_l,duty_cycle_r])
        velocities.append([robot.wl,robot.wr])
        costs_vec.append(cost)
        
        if not simulation:
            # Read Data 
            # serializer.read()
            wl_goal_rpm = wl_goal * 60 / (2 * math.pi)
            wr_goal_rpm = wr_goal * 60 / (2 * math.pi)
            serializer.data.update(wl_goal=wl_goal_rpm, wr_goal=wr_goal_rpm)
            serializer.write()

        if plotting:
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
            plt.quiver(goal_x,goal_y,0.01*np.cos(goal_th),0.01*np.sin(goal_th))
        
            plan_x_data = []
            plan_y_data = []
            plan_th_data = []

            for i in range(len(rrt_plan)):
                plan_x_data = np.append(plan_x_data, rrt_plan[i][0])
                plan_y_data = np.append(plan_y_data, rrt_plan[i][1])
                plan_th_data = np.append(plan_th_data, rrt_plan[i][2])

            plt.quiver(plan_x_data,plan_y_data,0.1*np.cos(plan_th_data),0.1*np.sin(plan_th_data), color="r")

            #plt.plot(obstacles[:,0],obstacles[:,1],'bo',markersize=15,alpha=0.2)
            plt.xlim(-1,1)
            plt.ylim(-1,1)
            plt.xlabel('x-position (m)')
            plt.ylabel('y-position (m)')
            plt.grid()

            plt.pause(0.05)
            plt.show(block=False)
            
            display.clear_output(wait=True)
            display.display(plt.gcf())
        
        # time.sleep(1)
        
        
        