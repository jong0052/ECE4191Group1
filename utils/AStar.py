# AStar Algorithm

# Bulk of Code originates from PythonRobotics.

from utils.Obstacle import *

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle as C 

class AStar():
    def __init__(self, start=np.zeros(3), goal=np.array([120,90,0]), obstacle_list=[], width=160, height=100, node_distance = 0.5):
        self.start = start
        self.goal = goal
        self.obstacle_list = obstacle_list
        self.width = width
        self.height = height
        self.node_distance = node_distance

        self.open_list = []
        self.closed_list = []

    class Node():
        """A node class for A* Pathfinding"""
        def __init__(self, parent=None, position=None, equality_threshold=0.25):
            self.parent = parent
            self.position = position
            self.equality_threshold = equality_threshold

            self.g = 0
            self.h = 0
            self.f = 0

        def __eq__(self, other):
            return self.dist(other) < self.equality_threshold
        
        def dist(self, other):
            return math.sqrt((self.position[0] - other.position[0])**2 + (self.position[1]- other.position[1])**2)

    def plan(self):
        """Returns a list of tuples as a path from the given start to the given end in the given maze"""

        # Create start and end node
        start_node = self.Node(None, self.start, self.node_distance / 2)
        start_node.g = start_node.h = start_node.f = 0
        end_node = self.Node(None, self.goal, self.node_distance / 2)
        end_node.g = end_node.h = end_node.f = 0

        # Initialize both open and closed list
        self.open_list = []
        self.closed_list = []

        # Add the start node
        self.open_list.append(start_node)

        # Loop until you find the end
        while len(self.open_list) > 0:
            # Get the current node
            current_node = self.open_list[0]
            current_index = 0
            for index, item in enumerate(self.open_list):
                if item.f < current_node.f:
                    current_node = item
                    current_index = index
            # print(current_node.position)

            # Pop current off open list, add to closed list
            self.open_list.pop(current_index)
            self.closed_list.append(current_node)

            # Found the goal
            if current_node.dist(end_node) < self.node_distance * 2:
                path = [[end_node.position[0], end_node.position[1], end_node.position[2]]]
                current = current_node
                while current is not None:
                    path.append([current.position[0], current.position[1]])
                    current = current.parent

                # Reverse the path
                path = path[::-1]

                # Calculate angles
                for i in range(0, len(path)-1):
                    node = path[i]
                    next_node = path[i+1]

                    angle = math.atan2(next_node[1] - node[1], next_node[0]-node[0])
                    node.append(angle)

                return path

            # Generate children
            children = []
            for new_position in [(0, -self.node_distance), (0, self.node_distance), (-self.node_distance, 0), (self.node_distance, 0), (-self.node_distance, -self.node_distance), (-self.node_distance, self.node_distance), (self.node_distance, -self.node_distance), (self.node_distance, self.node_distance)]: # Adjacent squares

                # Get node position
                node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])

                # Make sure within range
                if node_position[0] > self.width/2 or node_position[0] < -self.width/2 or node_position[1] > self.height/2 or node_position[1] < -self.height/2:
                    continue


                # Make sure walkable terrain
                if not self.is_collision_free(node_position):
                    # print(f"Collision! At: {node_position}")
                    continue

                # Create new node
                new_node = self.Node(current_node, node_position, self.node_distance / 2)

                # Append
                children.append(new_node)

            # Loop through children
            for child in children:
                self.add_node(child, current_node, end_node)
        
        # umm, epic fail?
        print("AStar couldn't find a path. Generate path directly to goal instead.")
        path = [[start_node.position[0], start_node.position[1], 0],
                [end_node.position[0], end_node.position[1], 0]]

        return path

    def add_node(self, child,current_node, end_node):
        # Child is on the closed list
        for closed_child in self.closed_list:
            if child == closed_child:
                # print(f"Found Dupe in Closed List")
                return

        # Create the f, g, and h values
        child.g = ((child.position[0] - current_node.position[0]) ** 2) + ((child.position[1] - current_node.position[1]) ** 2)
        child.h = ((child.position[0] - end_node.position[0]) ** 2) + ((child.position[1] - end_node.position[1]) ** 2)
        child.f = child.g + child.h

        # Child is already in the open list
        found_dupe = False
        for open_node in self.open_list:
            if child == open_node:
                # print("Found Dupe in Open List:")
                found_dupe = True
                if child.g >= open_node.g:
                    return
                else:
                    open_node.parent = child.parent
                    open_node.g = child.g
                    open_node.h = child.h
                    open_node.f = child.f
                    return

        # Add the child to the open list if it is unique
        if not found_dupe:
            # print(f"{current_node.position} Node added {child.position}")
            self.open_list.append(child)

    def is_collision_free(self, node_position):
        """
        Determine if nearby_node (new_node) is in the collision-free space.
        """
        points = np.vstack((node_position[0], node_position[1])).T
        for obs in self.obstacle_list:
            in_collision = obs.is_in_collision_with_points(points)
            if in_collision:
                return False
        
        return True  # safe
    
    def is_collision_free_path(self, path, path_index = 0, search_length = 999):
        """
        Determine if path is in the collision-free space.
        """
        id = 0
        for i in range(path_index, min(len(path), path_index + search_length)):
            point = path[i]
            points = np.vstack((point[0], point[1])).T
            for obs in self.obstacle_list:
                in_collision = obs.is_in_collision_with_points(points)
                if in_collision:
                    return False
            
        return True  # safe


def main():

    goal = np.array([14.0, 10.0, 0.0])
    start = np.array([1.0, 1.0, 0.0])

    all_obstacles = [Circle(11.5, 5, 2), Circle(4.5, 2.5, 2), Circle(4.8, 8, 2.5)]

    aStar = AStar(start, goal, all_obstacles, 32, 22, 1)
    plan = aStar.plan()


    x_data = np.array([])
    y_data = np.array([])
    th_data = np.array([])

    for i in range(len(plan)):
        x_data = np.append(x_data, plan[i][0])
        y_data = np.append(y_data, plan[i][1])
        th_data = np.append(th_data,plan[i][2])
    
    fig, ax = plt.subplots()
    plt.plot(x_data, y_data, "g-")

    print(plan)

    plt.quiver(x_data,y_data,0.1*np.cos(th_data),0.1*np.sin(th_data))

    for obstacle in all_obstacles:
        circle = C((obstacle.center[0], obstacle.center[1]), obstacle.radius, fill=False, edgecolor='blue')
        ax.add_patch(circle)
    plt.axis([-5, 18, -5, 15])
    plt.show()


if __name__ == '__main__':
    main()