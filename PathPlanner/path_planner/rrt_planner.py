import numpy as np
import matplotlib.pyplot as plt
from path_planner.utils import ObstaclesGrid

class Node:
    def __init__(self, x, y, parent=None):
        """
        Represents a node in the RRT tree.
        
        Args:
            x (float): X-coordinate of the node.
            y (float): Y-coordinate of the node.
            parent (Node, optional): Parent node in the tree.
        """
        self.x = x
        self.y = y
        self.parent = parent  

class RRTPlanner:
    def __init__(self, start, goal, map_size, obstacles, max_iter=500, step_size=5):
        """
        Initializes the RRT planner.

        Args:
            start (tuple): (x, y) coordinates of the start position.
            goal (tuple): (x, y) coordinates of the goal position.
            map_size (tuple): (width, height) of the environment.
            obstacles (ObstaclesGrid): Object that stores obstacle information.
            max_iter (int): Maximum number of iterations for RRT.
            step_size (float): Step size for expanding the tree.
        """
        self.start = Node(start[0], start[1])
        self.goal = Node(goal[0], goal[1])
        self.map_size = map_size
        self.obstacles = obstacles
        self.max_iter = max_iter
        self.step_size = step_size
        self.tree = [self.start] 

    def plan(self):
        """
        Implements the RRT algorithm to find a path from start to goal.

        Returns:
            list: A list of (x, y) tuples representing the path from start to goal.
        """
        for i in range(self.max_iter):
            rand_node = self.sample_random_point()  
            nearest_node = self.find_nearest_node(rand_node)  
            new_node = self.steer(nearest_node, rand_node) 

            if new_node and not self.is_colliding(new_node, nearest_node): 
                self.tree.append(new_node)

                if self.reached_goal(new_node):  
                    return self.construct_path(new_node) 
        
        print("Path not found.")
        return None

    def sample_random_point(self):
        """
        Samples a random point in the map.
        
        Returns:
            Node: A randomly sampled node.
        """

        pass

    def find_nearest_node(self, rand_node):
        """
        Finds the nearest node in the tree to a given random node.

        Args:
            rand_node (Node): The randomly sampled node.

        Returns:
            Node: The nearest node in the tree.
        """

        pass

    def steer(self, nearest_node, rand_node):
        """
        Generates a new node by moving from the nearest node toward the random node.

        Args:
            nearest_node (Node): The nearest node in the tree.
            rand_node (Node): The randomly sampled node.

        Returns:
            Node: A new node in the direction of rand_node.
        """

        pass

    def is_colliding(self, new_node, nearest_node):
        """
        Checks if the path between nearest_node and new_node collides with an obstacle.

        Args:
            new_node (Node): The new node to check.
            nearest_node (Node): The nearest node in the tree.

        Returns:
            bool: True if there is a collision, False otherwise.
        """

        pass

    def reached_goal(self, new_node):
        """
        Checks if the goal has been reached.

        Args:
            new_node (Node): The most recently added node.

        Returns:
            bool: True if goal is reached, False otherwise.
        """

        pass

    def construct_path(self, end_node):
        """
        Constructs the final path by backtracking from the goal node to the start node.

        Args:
            end_node (Node): The node at the goal position.

        Returns:
            list: A list of (x, y) tuples representing the path from start to goal.
        """

        pass
