import numpy as np
import math
import matplotlib.pyplot as plt

# class PathPlanner():
#     def __init__(self):
#         self.rx = [] # x coords of path to fruit
#         self.ry = [] # y coords of path to fruit
#         pass

#     def plan_path(self):
#         pass

class AStar:
    def __init__(self):
        self.resolution = 0.1
        self.rr = 0.17
        self.min_x, self.min_y = 0, 0
        self.max_x, self.max_y = 0, 0
        self.obstacle_map = None
        self.x_width, self.y_width = 0, 0
        self.motion = self.get_motion_model()
        
    class Node:
        def __init__(self, x, y, cost, parent_index):
            self.x = x
            self.y = y
            self.cost = cost
            self.parent_index = parent_index

        def __str__(self):
            return f"{str(self.x)}, {str(self.y)}, {str(self.cost)}, {str(self.parent_index)}"

    def planning(self, sx, sy, gx, gy):
        """
        A star path search
        input:
            s_x: start x position [m]
            s_y: start y position [m]
            gx: goal x position [m]
            gy: goal y position [m]
        output:
            rx: x position list of the final path
            ry: y position list of the final path
        """

        start_node = self.Node(self.calc_xy_index(sx, self.min_x),
                               self.calc_xy_index(sy, self.min_y), 0.0, -1)
        goal_node = self.Node(self.calc_xy_index(gx, self.min_x),
                              self.calc_xy_index(gy, self.min_y), 0.0, -1)

        open_set, closed_set = dict(), dict()
        open_set[self.calc_grid_index(start_node)] = start_node

        while 1:
            if len(open_set) == 0:
                print("Open set is empty..")
                break

            c_id = min(
                open_set,
                key=lambda o: open_set[o].cost + self.calc_heuristic(goal_node,
                                                                     open_set[
                                                                         o]))
            current = open_set[c_id]

            # show graph
            # if show_animation:  # pragma: no cover
            #     plt.plot(self.calc_grid_position(current.x, self.min_x),
            #              self.calc_grid_position(current.y, self.min_y), "xc")
            #     # for stopping simulation with the esc key.
            #     plt.gcf().canvas.mpl_connect('key_release_event',
            #                                  lambda event: [exit(
            #                                      0) if event.key == 'escape' else None])
            #     # if len(closed_set.keys()) % 10 == 0:
            #     #     plt.pause(0.001)

            if current.x == goal_node.x and current.y == goal_node.y:
                # print("Find goal")
                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                break

            # Remove the item from the open set
            del open_set[c_id]

            # Add it to the closed set
            closed_set[c_id] = current

            # expand_grid search grid based on motion model
            for i, _ in enumerate(self.motion):
                node = self.Node(current.x + self.motion[i][0],
                                 current.y + self.motion[i][1],
                                 current.cost + self.motion[i][2], c_id)
                n_id = self.calc_grid_index(node)

                # If the node is not safe, do nothing
                if not self.verify_node(node):
                    continue

                if n_id in closed_set:
                    continue

                if n_id not in open_set:
                    open_set[n_id] = node  # discovered a new node
                else:
                    if open_set[n_id].cost > node.cost:
                        # This path is the best until now. record it
                        open_set[n_id] = node

        self.rx, self.ry = self.calc_final_path(goal_node, closed_set)

        return self.rx, self.ry

    def calc_final_path(self, goal_node, closed_set):
        # generate final course
        self.rx, self.ry = [self.calc_grid_position(goal_node.x, self.min_x)], [
            self.calc_grid_position(goal_node.y, self.min_y)]
        parent_index = goal_node.parent_index
        while parent_index != -1:
            n = closed_set[parent_index]
            self.rx.append(self.calc_grid_position(n.x, self.min_x))
            self.ry.append(self.calc_grid_position(n.y, self.min_y))
            parent_index = n.parent_index

        return self.rx, self.ry

    @staticmethod
    def calc_heuristic(n1, n2):
        w = 1.0  # weight of heuristic
        d = w * math.hypot(n1.x - n2.x, n1.y - n2.y)
        return d

    def calc_grid_position(self, index, min_position):
        """
        calc grid position
        :param index:
        :param min_position:
        :return:
        """
        pos = index * self.resolution + min_position
        return pos

    def calc_xy_index(self, position, min_pos):
        return round((position - min_pos) / self.resolution)

    def calc_grid_index(self, node):
        return (node.y - self.min_y) * self.x_width + (node.x - self.min_x)

    def verify_node(self, node):
        px = self.calc_grid_position(node.x, self.min_x)
        py = self.calc_grid_position(node.y, self.min_y)

        if px < self.min_x:
            return False
        elif py < self.min_y:
            return False
        elif px >= self.max_x:
            return False
        elif py >= self.max_y:
            return False

        # collision check
        if self.obstacle_map[node.x][node.y]:
            return False

        return True

    def calc_obstacle_map(self, ox, oy):

        self.min_x = round(min(ox))
        self.min_y = round(min(oy))
        self.max_x = round(max(ox))
        self.max_y = round(max(oy))
        #print("min_x:", self.min_x)
        #print("min_y:", self.min_y)
        #print("max_x:", self.max_x)
        #print("max_y:", self.max_y)

        self.x_width = round((self.max_x - self.min_x) / self.resolution)
        self.y_width = round((self.max_y - self.min_y) / self.resolution)
        #print("x_width:", self.x_width)
        #print("y_width:", self.y_width)

        # obstacle map generation
        self.obstacle_map = [[False for _ in range(self.y_width)]
                             for _ in range(self.x_width)]
        for ix in range(self.x_width):
            x = self.calc_grid_position(ix, self.min_x)
            for iy in range(self.y_width):
                y = self.calc_grid_position(iy, self.min_y)
                for iox, ioy in zip(ox, oy):
                    d = math.hypot(iox - x, ioy - y)
                    if d <= self.rr:
                        self.obstacle_map[ix][iy] = True
                        break

    @staticmethod
    def get_motion_model():
        # dx, dy, cost
        motion = [[1, 0, 1],
                  [0, 1, 1],
                  [-1, 0, 1],
                  [0, -1, 1],
                  [-1, -1, math.sqrt(2)],
                  [-1, 1, math.sqrt(2)],
                  [1, -1, math.sqrt(2)],
                  [1, 1, math.sqrt(2)]]

        return motion


    def plan_path(self, r_state, goals, goal_num, markers, unknown_obs, show_animation=False):
        sx = r_state[0]
        sy = r_state[1]
        gx = goals[goal_num][0]  # [m]
        gy = goals[goal_num][1]  # [m]
        
        aruco_size = 0.3/2 # [m]

        # define map boundary
        ox, oy = [], []
        for i in np.linspace(-1.5, 1.5, 60):
            ox.append(i)
            oy.append(1.5)
        for i in np.linspace(-1.5, 1.5, 60):
            ox.append(i)
            oy.append(-1.5)
        for i in np.linspace(-1.5, 1.5, 60):
            ox.append(1.5)
            oy.append(i)
        for i in np.linspace(-1.5, 1.5, 60):
            ox.append(-1.5)
            oy.append(i)
        
        # reading map and set obstacles (aruco marker)
        for i in markers:
            x_min = i[0] - aruco_size
            x_max = i[0] + aruco_size
            y_min = i[1] - aruco_size
            y_max = i[1] + aruco_size
            for j in np.linspace(x_min,x_max, 15):
                for k in np.linspace(y_min,y_max, 15):
                    ox.append(j)
                    oy.append(k)
            for j in np.linspace(x_min,x_max, 15):
                ox.append(j)
                oy.append(y_min)
            for j in np.linspace(y_min,y_max, 15):
                ox.append(x_max)
                oy.append(j)
            for j in np.linspace(y_min,y_max, 15):
                ox.append(x_min)
                oy.append(j)

        # checking unknown fruit obstacles and set it on map
        print('unknown obstacles:', unknown_obs)
        # print(len(self.unknown_obstacles.keys()))
        for i in unknown_obs.keys():
            x_min = unknown_obs[i][0][0] - aruco_size
            x_max = unknown_obs[i][0][0] + aruco_size
            y_min = unknown_obs[i][1][0] - aruco_size
            y_max = unknown_obs[i][1][0] + aruco_size
            for j in np.linspace(x_min,x_max, 15):
                for k in np.linspace(y_min,y_max, 15):
                    ox.append(j)
                    oy.append(k)
            for j in np.linspace(x_min,x_max, 15):
                ox.append(j)
                oy.append(y_min)
            for j in np.linspace(y_min,y_max, 15):
                ox.append(x_max)
                oy.append(j)
            for j in np.linspace(y_min,y_max, 15):
                ox.append(x_min)
                oy.append(j)

        self.calc_obstacle_map(ox, oy)

        # checking arrived goals and set it on map
        print('arrived_goals:', goals[:goal_num])
        for goal in goals[:goal_num]:
            x_min = goal[0] - aruco_size
            x_max = goal[0] + aruco_size
            y_min = goal[1] - aruco_size
            y_max = goal[1] + aruco_size
            for j in np.linspace(x_min,x_max, 15):
                for k in np.linspace(y_min,y_max, 15):
                    ox.append(j)
                    oy.append(k)
            for j in np.linspace(x_min,x_max, 15):
                ox.append(j)
                oy.append(y_min)
            for j in np.linspace(y_min,y_max, 15):
                ox.append(x_max)
                oy.append(j)
            for j in np.linspace(y_min,y_max, 15):
                ox.append(x_min)
                oy.append(j)

        # if show_animation:  # pragma: no cover
        #     plt.plot(ox, oy, ".k")
        #     plt.plot(sx, sy, "og")
        #     plt.plot(gx, gy, "xb")
        #     plt.xticks([-1.6, -1.2, -0.8, -0.4, 0, 0.4, 0.8, 1.2, 1.6])
        #     plt.yticks([-1.6, -1.2, -0.8, -0.4, 0, 0.4, 0.8, 1.2, 1.6])
        #     plt.grid(True)
        #     plt.axis("equal")

        rx, ry = self.planning(sx, sy, gx, gy)
        rx.reverse()
        ry.reverse()

        rx = np.array(rx).reshape(-1,1)
        ry = np.array(ry).reshape(-1,1)
        plt.savefig('path.png')
        plt.clf()

        self.path = np.hstack((rx,ry))
        self.start_planning = False
        self.run_path = True
        
        print("------------------Next Goal Path------------------")
        print(self.path)
        print(' ')

        