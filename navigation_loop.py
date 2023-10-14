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
from utils.const import *
from utils.cubic_spline_planner import *

from multiprocessing import Value
from multiprocessing.managers import ListProxy

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
        # print(f"dt: {dt}")

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
    
    def __init__(self,dt=0.1,steps=5,alpha=1,beta=0.1):
        
        self.dt = dt
        self.steps = steps
        # Tentacles are possible trajectories to follow
        self.tentacles = []
        for v in range(0,11, 1):
            for w in range(-20,21, 1):
                self.tentacles.append((v/200, w/20))

        self.reverse_tentacles = []
        for v in range(0,11, 1):
            for w in range(-20,21, 1):
                self.reverse_tentacles.append((-v/200, w/20))
            # print(self.tentacles)
  
        self.alpha = alpha
        self.beta = beta
    
    # Play a trajectory and evaluate where you'd end up
    def roll_out(self,v,w,goal_x,goal_y,goal_th,x,y,th,obstacles,collision):
        
        for j in range(self.steps):
        
            x = x + self.dt*v*np.cos(th)
            y = y + self.dt*v*np.sin(th)
            th = (th + w*self.dt)
            
            if collision:
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
    def plan(self,goal_x,goal_y,goal_th,x,y,th,obstacles, reverse=False, collision= True):
        
        costs =[]

        if (not reverse):
            for v,w in self.tentacles:
                costs.append(self.roll_out(v,w,goal_x,goal_y,goal_th,x,y,th,obstacles,collision))
            
            best_idx = np.argmin(costs)
            best_costs = np.min(costs)

            # print(self.tentacles[best_idx])

            return self.tentacles[best_idx], best_costs
        else:
            for v,w in self.reverse_tentacles:
                costs.append(self.roll_out(v,w,goal_x,goal_y,goal_th,x,y,th,obstacles,collision))
            
            best_idx = np.argmin(costs)
            best_costs = np.min(costs)

            # print(self.tentacles[best_idx])

            return self.reverse_tentacles[best_idx], best_costs

class Map:
    def __init__(self, width = 2, height = 2, true_obstacles = []):
        # Constants
        self.width = width
        self.height = height
        self.obstacle_dots = np.array([[100, 100]])
        self.true_obstacles = true_obstacles
        self.obstacle_size = OBSTACLE_SIZE
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
            
            for i in range(-55, 56, 4):
                x = i / 100 * self.width
                for j in range(-55, 56,4):
                    y = j / 100 * self.height

                    if (not self.is_collision_free(x, y)
                        or np.abs(x) > self.width / 2
                        or np.abs(y) > self.height / 2):
                        self.obstacle_dots= np.concatenate((self.obstacle_dots, np.array([[x, y]])), axis=0)
            
            self.initialize = False

        # If obstacle is within 0.01 of another point, don't add it to avoid clutter
        for dot in self.obstacle_dots:
            dx = dot[0] - point[0][0]
            dy = dot[1] - point[0][1]
            h = math.sqrt(dx ** 2 + dy ** 2)

            if h < self.obstacle_closest_threshold:
               return
            

        self.obstacle_dots= np.concatenate((self.obstacle_dots, point), axis=0)


    def get_obstacle_list(self, size=OBSTACLE_SIZE):
        obstacle_list = []

        if (not self.initialize):
            for dot in self.obstacle_dots:
                obstacle_list.append(Circle(dot[0], dot[1], size))

        return obstacle_list

    def get_obstacle_log(self):
        obstacle_list = []

        if (not self.initialize):
            for dot in self.obstacle_dots:
                obstacle_list.append([dot[0],dot[1]])

        return obstacle_list

class Goal():
    def __init__(self, x=0, y=0, theta=0, threshold=1e-2):
        self.x = x
        self.y = y
        self.theta = theta

        self.threshold = threshold
        self.alpha = 3 # x y factor
        self.beta = 0.5 # th factor
    
    def check_reach_goal(self, robot_x, robot_y, robot_th):
        e_th = self.theta-robot_th
        e_th = np.arctan2(np.sin(e_th),np.cos(e_th))
        
        cost = self.alpha*((self.x-robot_x)**2 + (self.y-robot_y)**2) + self.beta*(e_th**2)

        if cost < self.threshold:
            return True
        else:
            return False
    
    def to_nparray(self):
        return np.array([self.x, self.y, self.theta])
    
    def to_list(self):
        return [self.x, self.y, self.theta]

class RobotSystem():
    def __init__(self, manager_mp: MPManager, pose = [0, 0, math.pi/2]):
        self.robot = DiffDriveRobot(inertia=10, drag=2, wheel_radius=0.0258, wheel_sep=0.22,x=pose[0],y=pose[1],th=pose[2])
        self.controller = RobotController(Kp=1.0,Ki=0.01,wheel_radius=0.0258, wheel_sep=0.22)
        self.tentaclePlanner = TentaclePlanner(dt=0.1,steps=10,alpha=1,beta=1e-9)
        self.map = Map(1.2, 1.2, obstacles)

        self.manager_mp = manager_mp

    def drive_to_goal(self, goal: Goal = Goal(), reverse = False, collision = True) -> bool:
                
        final_goal = goal.to_nparray()
        print(f"New Goal: {final_goal}")
        start = np.array([self.robot.x, self.robot.y, self.robot.th])
        path_resolution = 0.05
        if collision:
            algorithm = AStar(start = start, goal=final_goal, obstacle_list=self.map.get_obstacle_list(OBSTACLE_SIZE + 0.05), width=self.map.width, height = self.map.height, node_distance=path_resolution)
        else:
            algorithm = AStar(start = start, goal=final_goal, obstacle_list=[], width=self.map.width, height = self.map.height, node_distance=path_resolution)
        plan = algorithm.plan()

        print("The Plan:")
        print(plan)
        plan_index = 0
        path_attempts = 0

        last_duty_cycle_l = 0
        last_duty_cycle_r = 0

        while (not goal.check_reach_goal(self.robot.x, self.robot.y, self.robot.th)):
            
            # Map Generation for obstacles
            self.map.update(self.robot.x, self.robot.y, self.robot.th, self.manager_mp.usLeft_update, self.manager_mp.usFront_update, self.manager_mp.usRight_update, self.manager_mp.usLeft_value, self.manager_mp.usFront_value, self.manager_mp.usRight_value)
            
            temp_goal = plan[plan_index]
            tentacle,cost = self.tentaclePlanner.plan(temp_goal[0],temp_goal[1],temp_goal[2],self.robot.x,self.robot.y,self.robot.th, self.map.obstacle_dots, reverse, collision)
            v,w=tentacle

            duty_cycle_l,duty_cycle_r,wl_goal,wr_goal = self.controller.drive(v,w,self.robot.wl,self.robot.wr)
            self.manager_mp.wl_goal_value = wl_goal
            self.manager_mp.wr_goal_value = wr_goal
            # print(f"v: {v}, w: {w}, wl_goal: {wl_goal}, wr_goal: {wr_goal}")
            # print(f"Temp Goal, x: {temp_goal[0]}, y: {temp_goal[1]}, z: {temp_goal[2]}")

            # Simulate robot motion - send duty cycle command to controller
            if (not simulation):
                wl = ((self.manager_mp.current_wl/ 60)* 2*math.pi)
                wr = ((self.manager_mp.current_wr/ 60)* 2*math.pi)
                x,y,th = self.robot.pose_update(last_duty_cycle_l,last_duty_cycle_r, wl, wr)
            else:
                x,y,th = self.robot.pose_update(last_duty_cycle_l,last_duty_cycle_r)

            last_duty_cycle_l = duty_cycle_l
            last_duty_cycle_r = duty_cycle_r

            # Have we reached temp_goal?
            # print(f"Tentacle cost: {cost}")
            if (cost < 1e-2):
                # print(f"plan_index: {plan_index}")
                if (plan_index < len(plan) - 1):
                    plan_index += 1
                    # print("New Temp Goal.")

            algorithm.obstacle_list = self.map.get_obstacle_list()

            if (collision and not algorithm.is_collision_free_path(plan, plan_index, 5)):
                if (path_attempts < MAX_PATH_ATTEMPTS):
                    print("Collision Detected. New Plan.")
                    start = np.array([self.robot.x, self.robot.y, self.robot.th])

                    # print("why are u runnin")
                    if collision:
                        algorithm = AStar(start = start, goal=final_goal, obstacle_list=self.map.get_obstacle_list(OBSTACLE_SIZE + 0.05), width=self.map.width, height = self.map.height, node_distance=path_resolution)
                    else:
                        algorithm = AStar(start = start, goal=final_goal, obstacle_list=[], width=self.map.width, height = self.map.height, node_distance=path_resolution)
        
                    new_plan = algorithm.plan()

                    if (len(new_plan) > 0):
                        plan = new_plan
                        plan_index = 0
                        path_attempts += 1
                else:
                    print("Collision Detected, but Max Attempts reached. KEEP GOING.")
                    collision = False

            # Log data
            self.manager_mp.poses.append([x,y,th])
            self.manager_mp.obstacle_data[:] = []
            self.manager_mp.obstacle_data.extend(self.map.get_obstacle_log())
            self.manager_mp.plan_mp[:] = []
            self.manager_mp.plan_mp.extend(plan)
            self.manager_mp.robot_data[:] = []
            self.manager_mp.robot_data.extend([self.robot.x, self.robot.y, self.robot.th])
            self.manager_mp.goal_data[:] = []
            self.manager_mp.goal_data.extend(goal.to_list())
            
            # print("loop 1")

        # Done.

        self.robot.wl = 0
        self.robot.wr = 0
        return True
    
    def unload_package(self):
        time.sleep(3)
        return

    def load_package(self):
        time.sleep(3)
        return
    
    def wait_for_state(self, required_state = []):
        while True:
            for state in required_state:
                if (state == self.manager_mp.other_robot_state):
                    return

            print(f"Waiting for other robot... (State: {self.manager_mp.other_robot_state}, Required: {required_state})")
            time.sleep(0.5)

    def set_state(self, state):
        self.manager_mp.robot_state = state
        print(f"Set State to {state}")

    def localization(self):
        # Step 1: Align to Wall
        # Step 2: Localization using TOF sensors
        pass

def simulate_other_robot_loop(manager_mp : MPManager):
    manager_mp.other_robot_state = 0
    manager_mp.other_robot_pose = [-0.2, -0.45]

    # Loading...
    time.sleep(5)

    # Go to corner.
    while (True):
        # State 1.
        for x in [-0.2,-0.25,-0.3,-0.35,-0.4,-0.45]:
            manager_mp.other_robot_pose = [x, -0.45]
            time.sleep(0.25)

        print("Other State: 1")
        manager_mp.other_robot_state = 1
        
        while (manager_mp.robot_state == 2 or manager_mp.robot_state == 0):
            time.sleep(1)
        
        # State 2
        print("Other State: 2")
        manager_mp.other_robot_state = 2

        for y in range(-9, 10, 1):
            manager_mp.other_robot_pose = [-0.45, y/20]
            time.sleep(0.25)
        for x in range(-9, 10, 1):
            manager_mp.other_robot_pose = [x/20, 0.45]
            time.sleep(0.25)

        while (manager_mp.robot_state == 3):
            time.sleep(0.25)
        for y in range(-9, 10, 1):
            manager_mp.other_robot_pose = [0.45, -y/20]
            time.sleep(0.25)

        # State 3
        print("Other State: 3")
        manager_mp.other_robot_state = 3

        while (manager_mp.robot_state == 1 or manager_mp.robot_state == 4):
            time.sleep(0.25)
        
        # State 4
        for x in range(-9, 1, 1):
            manager_mp.other_robot_pose = [-x/20, -0.45]
            time.sleep(0.25)
            
        print("Other State: 4")
        manager_mp.other_robot_state = 4
        time.sleep(5)
        for x in range(0, 4, 1):
            manager_mp.other_robot_pose = [-x/20, -0.45]
            time.sleep(0.25)

def navigation_loop(manager_mp: MPManager):
    init_goal = Goal(0.2, -0.4, math.pi/2)
    start_goal = Goal(0,0,math.pi/2)
    parcel_A_goal_init = Goal(-0.4,0.3, math.pi/2)
    parcel_A_goal = Goal(-0.4,0.45, math.pi/2)
    parcel_B_goal_init = Goal(0,0.3, math.pi/2)
    parcel_B_goal = Goal(0,0.45, math.pi/2)
    parcel_C_goal_init = Goal(0.4, 0.3, math.pi/2)
    parcel_C_goal = Goal(0.4, 0.45, math.pi/2)
    loading_zone = Goal(0, -0.45, math.pi/2)

    system = RobotSystem(manager_mp, [0.2,-0.45,math.pi/2])

    # State 0: Initialize
    # - Load Package
    # - Move to Centre point.
    # - Wait for Other Team to get set to State 1.
    # - To State 2 if travelling first in field.
    # system.drive_to_goal(loading_zone)
    system.drive_to_goal(init_goal) # just so we have map
    system.load_package()
    system.drive_to_goal(start_goal)
    
    init_skip = True

    while (True):
        # State 1: Loaded and Ready.
        # - Wait for Other Team to return (State 3)
        # - To State 2
        if (not init_skip):
            system.drive_to_goal(start_goal, False, False)
            system.set_state(1)
            system.wait_for_state([3, 4, 5])
        else:
            init_skip = False

        # State 2: In the Field.
        # - Travel to Delivery Package point
        # - Unload
        # - Wait for Other Team to reach Loaded and Ready (State 1)
        # - Return to State 3 zone
        system.set_state(2)
        system.drive_to_goal(parcel_A_goal_init, False, False)
        system.drive_to_goal(parcel_A_goal, False, False)
        system.unload_package()
        system.wait_for_state([1,5])
        
        # State 3: Returned
        # - Wait for Other Team to be in the Field (State 2)
        # - Go to State 4
        system.drive_to_goal(start_goal, True, False)
        system.set_state(3)
        system.wait_for_state([2,5])
        system.localization()

        # State 4: Loading
        # - Get package loaded
        # - Travel to Centre Point
        # - Go to State 1
        system.drive_to_goal(loading_zone, True, False)
        system.set_state(4)
        system.load_package()