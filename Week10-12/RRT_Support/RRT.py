# This is an adapted version of the RRT implementation done by Atsushi Sakai (@Atsushi_twi)
import numpy as np
import random
import os
import math
import cv2

from RRT_Support.Obstacle import Circle

class RRTC:
    """
    Class for RRT planning
    """

    class Node:
        """
        RRT Node
        """

        def __init__(self, x, y):
            self.x = x
            self.y = y
            self.path_x = []
            self.path_y = []
            self.parent = None

    def __init__(self, start=np.zeros(2),
                 goal=np.array([120, 90]),
                 obstacle_list=None,
                 width=3,
                 height=3,
                 expand_dis=0.1,
                 path_resolution=0.01,
                 max_points=300):
        """
        Setting Parameter
        start:Start Position [x,y]
        goal:Goal Position [x,y]
        obstacle_list: list of obstacle objects
        width, height: search area
        expand_dis: min distance between random node and closest node in rrt to it
        path_resolion: step size to considered when looking for node to expand
        """
        self.start = self.Node(start[0], start[1])
        self.end = self.Node(goal[0], goal[1])
        self.width = width
        self.height = height
        self.expand_dis = expand_dis
        self.path_resolution = path_resolution
        self.max_nodes = max_points
        self.obstacle_list = obstacle_list
        self.start_node_list = []  # Tree from start
        self.end_node_list = []  # Tree from end

    def planning_old(self):
        """
        rrt path planning
        """
        self.start_node_list = [self.start]
        self.end_node_list = [self.end]
        while len(self.start_node_list) + len(self.end_node_list) <= self.max_nodes:

            # 1. Sample and add a node in the start tree
            random_node = self.get_random_node()

            nearest_start_node = self.start_node_list[self.get_nearest_node_index(self.start_node_list, random_node)]
            new_node = self.steer(nearest_start_node, random_node, self.expand_dis)

            if self.is_collision_free(new_node):
                self.start_node_list.append(new_node)

                # 2. Check whether trees can be connected
                nearest_end_node = self.end_node_list[self.get_nearest_node_index(self.end_node_list, new_node)]
                distance, _ = self.calc_distance_and_angle(nearest_end_node, new_node)
                if distance <= self.expand_dis:

                    # 3. Add the node that connects the trees and generate the path
                    # Note: It is important that you return path found as:
                    # return self.generate_final_course(len(self.start_node_list) - 1, len(self.end_node_list) - 1)

                    new_node_connect = self.steer(nearest_end_node, new_node, distance)

                    if self.is_collision_free(new_node_connect):
                        self.end_node_list.append(new_node_connect)
                        final = self.straighten(
                            self.generate_final_course(len(self.start_node_list) - 1, len(self.end_node_list) - 1))
                        if final[0] == [self.end.x, self.end.y]:
                            final = final[::-1]
                        print(final)
                        return final

                # 4. Sample and add a node in the end tree

                nearest_end_node = self.end_node_list[self.get_nearest_node_index(self.end_node_list, new_node)]
                new_node_connect = self.steer(nearest_end_node, new_node, self.expand_dis)
                if self.is_collision_free(new_node_connect):
                    self.end_node_list.append(new_node_connect)

            # 5. Swap start and end trees
            self.start_node_list, self.end_node_list = self.end_node_list, self.start_node_list

        # ENDTODO ----------------------------------------------------------------------------------------------

        return None  # cannot find path

    def planning(self):
        path_old = self.planning_old()
        path_new = np.array(path_old).T
        display_path(self.obstacle_list, path_old)
        return path_new[0].tolist(),path_new[1].tolist()


    def straighten(self, path):
        final_path = []
        start_idx = 0
        search_idx = 2
        start_n = self.Node(path[0][0], path[0][1])
        final_path.append(path[0])
        while search_idx < len(path):
            new_node = self.steer(start_n, self.Node(path[search_idx][0], path[search_idx][1]))
            if not self.is_collision_free(new_node):
                final_path.append(path[search_idx - 1])
                start_n = self.Node(path[search_idx - 1][0], path[search_idx - 1][1])
                search_idx += 1
            else:
                search_idx += 1
        final_path.append(path[-1])

        return final_path

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

        return path

    def calc_dist_to_goal(self, x, y):
        dx = x - self.end.x
        dy = y - self.end.y
        return math.hypot(dx, dy)

    def get_random_node(self):
        x = (self.width * np.random.random_sample()) - 1.5
        y = (self.height * np.random.random_sample()) - 1.5
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

def get_obstacles(fruit_true_pos, aruco_true_pos, search_list_pose, idx):
    obstacles = []
    fruit_safety = 0.15
    aruco_safety = 0.20

    for fruit_pos in fruit_true_pos:
        print(f'fruit_pos={fruit_pos}')
        print(f'search_pose={search_list_pose[idx]}')
        # If the search fruit is the position or (last fruit is in the position (for second or more fruit))
        if search_list_pose[idx].tolist() == fruit_pos.tolist() or (idx > 0 and search_list_pose[idx-1].tolist() == fruit_pos.tolist()):
            continue
        else:
            obstacles.append(Circle(fruit_pos[0],fruit_pos[1],fruit_safety))      

    for arucos in aruco_true_pos:
        # obstacles.append(Rectangle((arucos[0], arucos[1]), aruco_safety, aruco_safety))
        obstacles.append(Circle(arucos[0], arucos[1], aruco_safety))
    return obstacles

def display_path(obs, path):
    """
    obs is a list of the obstacles
    path is the path in the form of [[x,y]]
    """
    map = np.zeros((300, 300, 3), np.uint8)

    # Make all the obstacles white circles
    for ob in obs:
        print(ob.center*100)
        print(ob.radius*100)
        ob_x = int((ob.center[0] + 1.5)*100)
        ob_y = int((ob.center[1] + 1.5)*100)
        rad = int(ob.radius*100)
        map = cv2.circle(map, (ob_x, ob_y), rad, (255,255,255), -1)
    if path is not None:
        for i in range(len(path)-1):
            s1_x = int((path[i][0] + 1.5)*100)
            s1_y = int((path[i][1] + 1.5)*100)
            s2_x = int((path[i+1][0] + 1.5)*100)
            s2_y = int((path[i+1][1] + 1.5)*100)
            print(s1_x, s1_y, s2_x, s2_y)
            map = cv2.line(map,(s1_x,s1_y),(s2_x,s2_y),(0,255,0),2)

    map = cv2.flip(map, 1)
    cv2.imwrite('RRT_path.png',map)
