# RRT Algorithm

from Obstacle import *

import numpy as np
import random
import matplotlib.pyplot as plt
from matplotlib.patches import Circle as C 

import math

# This is an adapted version of the RRT implementation done by Atsushi Sakai (@Atsushi_twi)
class RRTC:
    """
    Class for RRT planning
    """
    class Node:
        """
        RRT Node
        """
        def __init__(self, x, y, th = 0):
            self.x = x
            self.y = y
            self.th = th
            self.path_x = []
            self.path_y = []
            self.parent = None

    def __init__(self, start=np.zeros(3),
                 goal=np.array([120,90,0]),
                 obstacle_list=None,
                 width  = 160,
                 height = 100,
                 expand_dis = 3.0, 
                 path_resolution = 0.5, 
                 max_points = 200):
        """
        Setting Parameter
        start:Start Position [x,y]
        goal:Goal Position [x,y]
        obstacle_list: list of obstacle objects
        width, height: search area
        expand_dis: min distance between random node and closest node in rrt to it
        path_resolion: step size to considered when looking for node to expand
        """
        self.start = self.Node(start[0], start[1], start[2])
        self.end = self.Node(goal[0], goal[1], goal[2])
        self.width = width
        self.height = height
        self.expand_dis = expand_dis
        self.path_resolution = path_resolution
        self.max_nodes = max_points
        self.obstacle_list = obstacle_list
        self.start_node_list = [] # Tree from start
        self.end_node_list = [] # Tree from end
        
    def planning(self):
        """
        rrt path planning
        """
        self.start_node_list = [self.start]
        self.end_node_list = [self.end]

        print("Start planning: ")
        while len(self.start_node_list) + len(self.end_node_list) <= self.max_nodes:
            
        #TODO: Complete the planning method ----------------------------------------------------------------
            # 1. Sample and add a node in the start tree
            rand_node = self.get_random_node()
            expansion_ind = self.get_nearest_node_index(self.start_node_list, rand_node)
            expansion_node = self.start_node_list[expansion_ind]
            
            new_node = self.steer(expansion_node, rand_node, self.expand_dis)
            
            if self.is_collision_free(new_node):
                self.start_node_list.append(new_node)
                # print("Append new start node")

                # 2. Check whether trees can be connected
                nearest_node_index = self.get_nearest_node_index(self.end_node_list, new_node)
                nearest_node = self.end_node_list[nearest_node_index]
                distance = ((nearest_node.x - new_node.x) ** 2 + (nearest_node.y - new_node.y) ** 2) ** (1/2)
                can_connect = distance < self.expand_dis
                
                # 3. Add the node that connects the trees and generate the path
                    # Note: It is important that you return path found as:
                    # return self.generate_final_course(len(self.start_node_list) - 1, len(self.end_node_list) - 1)
                if can_connect:
                    print("Path found!")
                    return self.generate_final_course(len(self.start_node_list) - 1, nearest_node_index)
                
            # 4. Sample and add a node in the end tree
            nearest_node_index = self.get_nearest_node_index(self.end_node_list, new_node)
            nearest_node = self.end_node_list[nearest_node_index]
            end_new_node = self.steer(nearest_node, new_node, self.expand_dis)
            if self.is_collision_free(end_new_node):
                # print("Append new end node")
                self.end_node_list.append(end_new_node)

                # 2. Check whether trees can be connected...
                # I am doing this step because I think it is a bit better this way, potentially.
                nearest_node_index = self.get_nearest_node_index(self.start_node_list, end_new_node)
                nearest_node = self.start_node_list[nearest_node_index]
                distance = ((nearest_node.x - end_new_node.x) ** 2 + (nearest_node.y - end_new_node.y) ** 2) ** (1/2)
                can_connect = distance < self.expand_dis
                
                # 3. Add the node that connects the trees and generate the path
                    # Note: It is important that you return path found as:
                    # return self.generate_final_course(len(self.start_node_list) - 1, len(self.end_node_list) - 1)
                if can_connect:
                    print("Path found!")
                    # I am using nearest_node_index because that would be the path we are connecting.
                    return self.generate_final_course(nearest_node_index, len(self.end_node_list) - 1)

            # My Comments:
            # I just realized I could had saved a lot of trouble if I ACTUALLY READ IT PROPERLY
            # (I missed out on the part in Step 3 where I need to add new_node to end_node_list as well,
            # which resulted in a WORLD of pain)
            # My implementation in the end somehow did the trick but as you observe,
            # it is pretty spaghetti, pretty jank
            # Oof.
        #ENDTODO ----------------------------------------------------------------------------------------------
            
        return []  # cannot find path
    
    # ------------------------------DO NOT change helper methods below ----------------------------
    def steer(self, from_node, to_node, extend_length=float("inf")):
        """
        Given two nodes from_node, to_node, this method returns a node new_node such that new_node 
        is “closer” to to_node than from_node is.
        """
        
        new_node = self.Node(from_node.x, from_node.y)
        d, theta = self.calc_distance_and_angle(new_node, to_node)
        cos_theta, sin_theta = np.cos(theta), np.sin(theta)

        new_node.path_x = [new_node.x]
        new_node.path_y = [new_node.y]

        if extend_length > d:
            extend_length = d

        # How many intermediate positions are considered between from_node and to_node
        n_expand = math.floor(extend_length / self.path_resolution)

        # Compute all intermediate positions
        for _ in range(n_expand):
            new_node.x += self.path_resolution * cos_theta
            new_node.y += self.path_resolution * sin_theta
            new_node.path_x.append(new_node.x)
            new_node.path_y.append(new_node.y)

        d, _ = self.calc_distance_and_angle(new_node, to_node)
        if d <= self.path_resolution:
            new_node.path_x.append(to_node.x)
            new_node.path_y.append(to_node.y)

        new_node.parent = from_node

        return new_node

    def is_collision_free(self, new_node):
        """
        Determine if nearby_node (new_node) is in the collision-free space.
        """
        if new_node is None:
            return True
        
        points = np.vstack((new_node.path_x, new_node.path_y)).T
        for obs in self.obstacle_list:
            in_collision = obs.is_in_collision_with_points(points)
            if in_collision:
                return False
        
        return True  # safe
    
    def is_collision_free_path(self, path):
        """
        Determine if nearby_node (new_node) is in the collision-free space.
        """
        for point in path:
            points = np.vstack((point[0], point[1])).T
            for obs in self.obstacle_list:
                in_collision = obs.is_in_collision_with_points(points)
                if in_collision:
                    return False
            
        return True  # safe
        
    def generate_final_course(self, start_mid_point, end_mid_point):
        """
        Reconstruct path from start to end node
        """
        # First half
        node = self.start_node_list[start_mid_point]
        path = []
        while node.parent is not None:
            path.append([node.x, node.y])
            node = node.parent
        path.append([node.x, node.y])
        
        # Other half
        node = self.end_node_list[end_mid_point]
        path = path[::-1]
        while node.parent is not None:
            path.append([node.x, node.y])
            node = node.parent
        path.append([node.x, node.y])

        # Calculate angles
        for i in range(0, len(path)-1):
            node = path[i]
            next_node = path[i+1]

            angle = math.atan2(next_node[1] - node[1], next_node[0]-node[0])
            node.append(angle)
        
        last_node = path[len(path)-1]
        last_node.append(self.end.th)

        return path

    def calc_dist_to_goal(self, x, y):
        dx = x - self.end.x
        dy = y - self.end.y
        return math.hypot(dx, dy)

    def get_random_node(self):
        x = self.width * (np.random.random_sample() - 0.5)
        y = self.height * (np.random.random_sample() -0.5)
        rnd = self.Node(x, y)
        return rnd

    @staticmethod
    def get_nearest_node_index(node_list, rnd_node):        
        # Compute Euclidean disteance between rnd_node and all nodes in tree
        # Return index of closest element
        dlist = [(node.x - rnd_node.x) ** 2 + (node.y - rnd_node.y)
                 ** 2 for node in node_list]
        minind = dlist.index(min(dlist))
        return minind

    @staticmethod
    def calc_distance_and_angle(from_node, to_node):
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        d = math.hypot(dx, dy)
        theta = math.atan2(dy, dx)
        return d, theta


# goal = np.array([14.0, 10.0, 0.0])
# start = np.array([1.0, 1.0, 0.0])

# all_obstacles = [Circle(11.5, 5, 2), Circle(4.5, 2.5, 2), Circle(4.8, 8, 2.5)]

# rrtc = RRTC(start = start, goal=goal, obstacle_list=all_obstacles, width=16, height = 10, expand_dis=3.0, path_resolution=1)

# plan = rrtc.planning()
# print(plan)

# x_data = np.array([])
# y_data = np.array([])
# th_data = np.array([])

# for i in range(len(plan)):
#     x_data = np.append(x_data, plan[i][0])
#     y_data = np.append(y_data, plan[i][1])
#     th_data = np.append(th_data,plan[i][2])

# fig, ax = plt.subplots()
# # plt.plot(x_data, y_data, "g-")

# plt.quiver(x_data,y_data,0.1*np.cos(th_data),0.1*np.sin(th_data))

# for obstacle in all_obstacles:
#     circle = C((obstacle.center[0], obstacle.center[1]), obstacle.radius, fill=False, edgecolor='blue')
#     ax.add_patch(circle)
# plt.axis([-5, 18, -5, 15])
# plt.show()