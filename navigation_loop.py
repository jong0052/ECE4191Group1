import numpy as np
from matplotlib import pyplot as plt
from IPython import display
from main import simulation, plotting
from utils.mpManager import MPManager

from utils.AStar import AStar
from utils.Obstacle import *
from matplotlib.patches import Circle as C
from matplotlib.patches import Rectangle as R
import math
import time

from multiprocessing import Value
from multiprocessing.managers import ListProxy

# obstacles = [Rectangle((-0.15,0.0),0.3,0.4)]
obstacles = [Circle(0,0.2,0.15)]
# obstacles = []

class DiffDriveRobot:

    def __init__(self, inertia=5, drag=0.2, wheel_radius=0.055, wheel_sep=0.15,x=0,y=0,th=0):
        
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
        #self.dt=dt
        self.last = [time.time()]

        self.r = wheel_radius
        self.l = wheel_sep

    # Should be replaced by motor encoder measurement which measures how fast wheel is turning
    # Here, we simulate the real system and measurement
    def motor_simulator(self, w, duty_cycle, dt):
        torque = self.I * duty_cycle
        # print(torque)

        if (w > 0):
            w = min(w + dt * (torque - self.d * w), 3)
        elif (w < 0):
            w = max(w + dt * (torque - self.d * w), -3)
        else:
            w = w + dt * (torque)
        # print(f"dt: {dt} w: {w}")
        return w

    # Veclocity motion model
    def base_velocity(self, wl, wr):

        v = (wl * self.r + wr * self.r) / 2.0

        w = (wr * self.r - wl * self.r) / self.l

        return v, w

    # Kinematic motion model
    def pose_update(self, duty_cycle_l = 0, duty_cycle_r = 0, wl = 0, wr = 0):
        # print(self.last)
        dt = time.time() - self.last[-1]

        # print(duty_cycle_l)
        # print(duty_cycle_r)

        if (simulation): 
            self.wl = self.motor_simulator(self.wl, duty_cycle_l, dt)
            self.wr = self.motor_simulator(self.wr, duty_cycle_r, dt)
        else:
            self.wl = wl
            self.wr = wr
            # print(f"Current Wheel: {self.wl}, {self.wr}")

        v, w = self.base_velocity(self.wl, self.wr)

        self.x = self.x + dt * v * np.cos(self.th)
        self.y = self.y + dt * v * np.sin(self.th)
        self.th = self.th + w * dt

        self.last.append(time.time())
        self.last.pop(0)
        # print(self.last)
        # print(dt)
        # print(self.wl)
        # print(self.wr)

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
        
        wl_goal = (v_desired - self.l*w_desired/2)/self.r
        wr_goal = (v_desired + self.l*w_desired/2)/self.r
        
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
                   self.tentacles.append((v/200, w/10))
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

class Map:
    def __init__(self, width = 2, height = 2, true_obstacles = []):
        # Constants
        self.width = width
        self.height = height
        self.obstacle_dots = np.array([[100, 100]])
        self.true_obstacles = true_obstacles
        self.obstacle_size = 0.15
        self.obstacle_closest_threshold = 0.05

        self.initialize = True

    def update(self, robot_x, robot_y, robot_th, usLeft_update,usFront_update,  usRight_update, us_left=100, us_front=100, us_right=100):
        # Ultrasonic Distance
        if simulation:
            distance = self.check_ultrasonic(robot_x, robot_y, robot_th)
            self.generate_obstacle(robot_x, robot_y, robot_th, distance, 0, 0, 0)
        else:
            # us_left
            # self.generate_obstacle(robot_x, robot_y, robot_th, us_left, 0, 0, 0)

            # us_front
            if (usFront_update.value == 1):
                self.generate_obstacle(robot_x, robot_y, robot_th, us_front, 0, 0, 0)
                usFront_update.value = 0
                
            # us_front
            if (usLeft_update.value == 1):
                self.generate_obstacle(robot_x, robot_y, robot_th, us_left, -0.055, 0, 0)
                usLeft_update.value = 0
                
            # us_front
            if (usRight_update.value == 1):
                self.generate_obstacle(robot_x, robot_y, robot_th, us_right, 0.055, 0, 0)
                usRight_update.value = 0

            # # us_right
            # self.generate_obstacle(robot_x, robot_y, robot_th, us_right, 0, 0, 0)
        # Generate obstacles

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
            #min_dist = np.min(np.sqrt((curr_x-self.true_obstacles[:,0])**2+(curr_y-self.true_obstacles[:,1])**2))
            if (not self.is_collision_free(curr_x, curr_y)
                or np.abs(curr_x) > self.width / 2
                or np.abs(curr_y) > self.height / 2):
                # print(min_dist)
                # print(curr_x)
                # print(curr_y)
                # print(distance)
                hit = True

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

    def generate_obstacle(self, robot_x, robot_y, robot_th, distance, us_x=0, us_y=0, us_th=0):
        # def calc(rx, ry, theta, d, wx, wy,th):
        #     mat1 = np.array([[math.cos(theta), -1*math.sin(theta)],[math.sin(theta), math.cos(theta)]])
        #     mat2 = np.array([[math.cos(th), -1*math.sin(th)],[math.sin(th), math.cos(th)]])
        #     d_m = np.array([[d],
        #     [0]])
        #     r_m = np.array([[rx],
        #     [ry]])
        #     w_m = np.array([[wx],
        #     [wy]])
        #     return mat1@(mat2@d_m + r_m)+w_m

        # obs = calc(robot_x, robot_y, robot_th, distance, us_x, us_y, us_th)
        # print(obs)

        # Append to Obstacle_dots
        obs_x = robot_x + distance * np.cos(robot_th) + us_x * np.cos(robot_th - math.pi/ 2)
        obs_y = robot_y + distance * np.sin(robot_th) + us_x * np.sin(robot_th - math.pi/2)

        point = np.array([[obs_x, obs_y]])

        if (self.initialize):
            self.obstacle_dots = np.array([[2,2]])
            # for i in range(-6, 7, 1):
            #     i = i / 10
            #     # Arena
            #     self.obstacle_dots= np.concatenate((self.obstacle_dots, np.array([[-0.6, i]])), axis=0)
            #     self.obstacle_dots= np.concatenate((self.obstacle_dots, np.array([[0.6, i]])), axis=0)
            #     self.obstacle_dots= np.concatenate((self.obstacle_dots, np.array([[i, -0.6]])), axis=0)
            #     self.obstacle_dots= np.concatenate((self.obstacle_dots, np.array([[i, 0.6]])), axis=0)
            #     # print("test")
            
            # for i in range(0, 7, 1):
            #     i = i / 10
            #     # self.obstacle_dots= np.concatenate((self.obstacle_dots, np.array([[-0.05, i]])), axis=0)
            #     # self.obstacle_dots= np.concatenate((self.obstacle_dots, np.array([[0, i]])), axis=0)
            #     # self.obstacle_dots= np.concatenate((self.obstacle_dots, np.array([[0.05, i]])), axis=0)
            
            for i in range(-55, 56, 1):
                x = i / 100 * self.width
                for j in range(-55, 56, 1):
                    y = j / 100 * self.height

                    if (not self.is_collision_free(x, y)
                        or np.abs(x) > self.width / 2
                        or np.abs(y) > self.height / 2):
                        # print(min_dist)
                        # print(curr_x)
                        # print(curr_y)
                        # print(distance)
                        self.obstacle_dots= np.concatenate((self.obstacle_dots, np.array([[x, y]])), axis=0)
            
            self.initialize = False
            #print(f"Obstacles: {self.obstacle_dots}")
        
        # print(self.obstacle_dots)
        # print(point)
        # If obstacle is within 0.01 of another point, don't add it to avoid clutter.
        for dot in self.obstacle_dots:
            # print(dot)
            # print(point)
            dx = dot[0] - point[0][0]
            dy = dot[1] - point[0][1]
            h = math.sqrt(dx ** 2 + dy ** 2)

            if h < self.obstacle_closest_threshold:
               return
            

        self.obstacle_dots= np.concatenate((self.obstacle_dots, point), axis=0)


    def get_obstacle_list(self):
        obstacle_list = []

        if (not self.initialize):
            for dot in self.obstacle_dots:
                obstacle_list.append(Circle(dot[0], dot[1], self.obstacle_size))

        return obstacle_list

    def get_obstacle_log(self):
        obstacle_list = []

        if (not self.initialize):
            for dot in self.obstacle_dots:
                obstacle_list.append([dot[0],dot[1]])

        return obstacle_list

class GoalSetter():
    def __init__(self):
        self.goals = []
        self.wait_times = []
        self.current_id = -1

        # Thresholds for reaching goal
        self.threshold = 1e-2
        self.alpha = 3 # x y factor
        self.beta = 0.1 # th factor

        # Goal time
        # self.threshold_time = 5
        # self.last = time.time()
        # self.accumulative_time = 0
        

    def get_current_goal(self):
        if (self.current_id < len(self.goals)):
            return self.goals[self.current_id]
        else:
            return []
       
    def get_current_wait(self):
        if (self.current_id < len(self.wait_times)):
            return self.wait_times[self.current_id]
        else:
            return 5
        
    def add_emergency_goal(self, goal_x, goal_y, goal_th):
        self.goals.insert(self.current_id, [goal_x, goal_y, goal_th])

    def add_new_goal(self, goal_x, goal_y, goal_th, wait_time = 5):
        self.goals.append([goal_x, goal_y, goal_th])
        self.wait_times.append(wait_time)
        
    def check_reach_goal(self, robot_x, robot_y, robot_th):
        goal_x, goal_y, goal_th = self.get_current_goal()

        e_th = goal_th-robot_th
        e_th = np.arctan2(np.sin(e_th),np.cos(e_th))
        
        cost = self.alpha*((goal_x-robot_x)**2 + (goal_y-robot_y)**2) + self.beta*(e_th**2)

        if cost < self.threshold:
            return True
            # dt = time.time() - self.last
            # self.accumulative_time += dt
            # self.last = time.time()

            # if (self.accumulative_time > self.threshold_time):
            #     self.accumulative_time = 0
            #     return True
            # else:
            #     return False
        else:
            return False
            # self.accumulative_time = 0
            # self.last = time.time()

    # Increment Goal. If there's another goal, return True. If ran out of goals, return False.
    def increment_goal(self):
        self.current_id += 1

        if (self.current_id < len(self.goals)):
            return True
        else:
            return False

def navigation_loop(manager_mp: MPManager):

    # obstacles = [Circle(0.5,0.5,0.05),Circle(-0.5, -0.5, 0.05), Circle(-0.5, 0.5, 0.05), Circle(0.5, -0.5, 0.05)]
    robot = DiffDriveRobot(inertia=10, drag=2, wheel_radius=0.0258, wheel_sep=0.22,x=0.325,y=0.2,th=0)
    controller = RobotController(Kp=1.0,Ki=0.01,wheel_radius=0.0258, wheel_sep=0.22)
    tentaclePlanner = TentaclePlanner(dt=0.1,steps=10,alpha=1,beta=1e-9)
    map = Map(1.2, 1.2, obstacles)
    goal_setter = GoalSetter()
    
    # goal_setter.add_new_goal(0.3,0.0, math.pi)
    # goal_setter.add_new_goal(0, -0.3, math.pi/2, 1)
    # goal_setter.add_new_goal(0.325, -0.3, math.pi/2, 1)
    goal_setter.add_new_goal(0.325, 0.2, math.pi, 5)
    # goal_setter.add_new_goal(0.325, 0.2, math.pi, 10)
    #goal_setter.add_new_goal(0.3, 0.2, -math.pi/2, 1)
    #goal_setter.add_new_goal(0.3, -0.2, math.pi, 1)
    #goal_setter.add_new_goal(-0.3, -0.2, math.pi/2, 1)
    goal_setter.add_new_goal(-0.3, 0.2, math.pi/2, 5)

    while goal_setter.increment_goal():
            
            final_goal = np.array(goal_setter.get_current_goal())
            print(f"New Goal: {final_goal}")
            start = np.array([robot.x, robot.y, robot.th])
            expand_dis = 0.05
            path_resolution = 0.05
            algorithm = AStar(start = start, goal=final_goal, obstacle_list=map.get_obstacle_list(), width=map.width, height = map.height, node_distance=path_resolution)
            plan = algorithm.plan()
            print(plan)
            plan_index = 0

            while (not goal_setter.check_reach_goal(robot.x, robot.y, robot.th)):
                
                # Map Generation for obstacles
                map.update(robot.x, robot.y, robot.th, manager_mp.usLeft_update, manager_mp.usFront_update, manager_mp.usRight_update, manager_mp.usLeft_value, manager_mp.usFront_value, manager_mp.usRight_value)
                # map.update(robot.x+0.055, robot.y, robot.th, usLeft_update, 100, usLeft_value.value, 100)
                # map.update(robot.x-0.055, robot.y, robot.th, usRight_update, 100, usRight_value.value, 100)

                temp_goal = plan[plan_index]
                if map.is_collision_free(robot.x, robot.y):
                    # print(map.obstacle_dots)

                    # Example motion using controller
                    tentacle,cost = tentaclePlanner.plan(temp_goal[0],temp_goal[1],temp_goal[2],robot.x,robot.y,robot.th, map.obstacle_dots)
                    v,w=tentacle
                else:
                    v, w = -0.1, 0
                    cost = np.inf
                    # if math.pi / 4 < robot.th < 3 * math.pi / 4:
                    #     GoalSetter.add_emergency_goal(goal_setter, robot.x, robot.y - 0.3)
                    # elif 3 * math.pi / 4 < robot.th < 5 * math.pi / 4:
                    #     GoalSetter.add_emergency_goal(goal_setter, robot.x + 0.3, robot.y)
                    # elif 5 * math.pi / 4 < robot.th < 7 * math.pi / 4:
                    #     GoalSetter.add_emergency_goal(goal_setter, robot.x, robot.y + 0.3)
                    # else:
                    #     GoalSetter.add_emergency_goal(goal_setter, robot.x - 0.3, robot.y)

                # print(f"v: {v} w: {w}")

                duty_cycle_l,duty_cycle_r,wl_goal,wr_goal = controller.drive(v,w,robot.wl,robot.wr)
                manager_mp.wl_goal_value = wl_goal
                manager_mp.wr_goal_value = wr_goal

                # Simulate robot motion - send duty cycle command to controller
                if (not simulation):
                    wl = ((manager_mp.current_wl/ 60)* 2*math.pi)
                    wr = ((manager_mp.current_wr/ 60)* 2*math.pi)
                    #* 60 /( 2*math.pi)
                    x,y,th = robot.pose_update(duty_cycle_l,duty_cycle_r, wl, wr)
                else:
                    x,y,th = robot.pose_update(duty_cycle_l,duty_cycle_r)

                # Have we reached temp_goal?
                if (cost < 1e-2):
                    if (plan_index < len(plan) - 1):
                        plan_index += 1

                algorithm.obstacle_list = map.get_obstacle_list()

                if (not algorithm.is_collision_free_path(plan)):
                    start = np.array([robot.x, robot.y, robot.th])

                    # print("why are u runnin")
                    algorithm = AStar(start = start, goal=final_goal, obstacle_list=map.get_obstacle_list(), width=map.width, height = map.height, node_distance=path_resolution)
                    new_plan = algorithm.plan()

                    if (len(new_plan) > 0):
                        plan = new_plan
                        plan_index = 0

                # Log data
                # poses = manager_mp.poses.copy()
                # poses.append([x,y,th])
                # manager_mp.poses = poses
                # print(manager_mp.poses)
                manager_mp.poses.append([x,y,th])
                # print(manager_mp.poses)
                manager_mp.obstacle_data[:] = []
                manager_mp.obstacle_data.extend(map.get_obstacle_log())
                # print(manager_mp.obstacle_data)
                manager_mp.plan_mp[:] = []
                manager_mp.plan_mp.extend(plan)
                manager_mp.robot_data[:] = []
                manager_mp.robot_data.extend([robot.x, robot.y, robot.th])
                manager_mp.goal_data[:] = []
                manager_mp.goal_data.extend(goal_setter.get_current_goal())
                
                print("loop 1")

            # Hardcode wait for Milestone 1
            print(f"Reached Goal {goal_setter.current_id}, I am going to wait for {goal_setter.get_current_wait()} seconds.")
            start_wait = time.time()
            total_wait = goal_setter.get_current_wait()
            dt = 0

            while (dt <= total_wait):
                dt = time.time() - start_wait
                v, w = 0, 0

                map.update(robot.x, robot.y, robot.th, manager_mp.usLeft_update, manager_mp.usFront_update, manager_mp.usRight_update, manager_mp.usLeft_value, manager_mp.usFront_value, manager_mp.usRight_value)

                duty_cycle_l,duty_cycle_r,wl_goal,wr_goal = controller.drive(v,w,robot.wl,robot.wr)
                manager_mp.wl_goal_value = wl_goal
                manager_mp.wr_goal_value = wr_goal

                # Simulate robot motion - send duty cycle command to controller
                if (not simulation):
                    wl = (manager_mp.current_wl / 60) * 2*math.pi
                    wr = (manager_mp.current_wr / 60) * 2 * math.pi
                    x,y,th = robot.pose_update(duty_cycle_l,duty_cycle_r, wl, wr)
                else:
                    x,y,th = robot.pose_update(duty_cycle_l,duty_cycle_r)

                # Log data
                manager_mp.poses.append([x,y,th])
                manager_mp.robot_data[:] = []
                manager_mp.robot_data.extend([robot.x, robot.y, robot.th])
                
                time.sleep(0.1)

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
        plt.plot(robot_data_local[0], robot_data_local[1],'k',marker='+')
        plt.quiver(robot_data_local[0],robot_data_local[1],0.1*np.cos(robot_data_local[2]),0.1*np.sin(robot_data_local[2]))
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

        plt.pause(0.05)
        plt.show(block=False)

        display.clear_output(wait=True)
        display.display(plt.gcf())
        
        print("loop 3")