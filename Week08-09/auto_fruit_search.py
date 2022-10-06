# M4 - Autonomous fruit searching

# basic python packages
import sys, os
import cv2
import numpy as np
import json
import argparse
import time
from RRT_Support.RRT import *
from RRT_Support.Obstacle import *

# import SLAM components
sys.path.insert(0, "{}/slam".format(os.getcwd()))
from slam.ekf import EKF
from slam.robot import Robot
import slam.aruco_detector as aruco

# import utility functions
sys.path.insert(0, "util")
from util.pibot import PenguinPi
import util.measure as measure


def read_true_map(fname):
    """Read the ground truth map and output the pose of the ArUco markers and 3 types of target fruit to search

    @param fname: filename of the map
    @return:
        1) list of target fruits, e.g. ['apple', 'pear', 'lemon']
        2) locations of the target fruits, [[x1, y1], ..... [xn, yn]]
        3) locations of ArUco markers in order, i.e. pos[9, :] = position of the aruco10_0 marker
    """
    with open(fname, 'r') as fd:
        gt_dict = json.load(fd)
        fruit_list = []
        fruit_true_pos = []
        aruco_true_pos = np.empty([10, 2])

        # remove unique id of targets of the same type
        for key in gt_dict:
            x = np.round(gt_dict[key]['x'], 1)
            y = np.round(gt_dict[key]['y'], 1)

            if key.startswith('aruco'):
                if key.startswith('aruco10'):
                    aruco_true_pos[9][0] = x
                    aruco_true_pos[9][1] = y
                else:
                    marker_id = int(key[5])
                    aruco_true_pos[marker_id][0] = x
                    aruco_true_pos[marker_id][1] = y
            else:
                fruit_list.append(key[:-2])
                if len(fruit_true_pos) == 0:
                    fruit_true_pos = np.array([[x, y]])
                else:
                    fruit_true_pos = np.append(fruit_true_pos, [[x, y]], axis=0)

        return fruit_list, fruit_true_pos, aruco_true_pos


def read_search_list():
    """Read the search order of the target fruits

    @return: search order of the target fruits
    """
    search_list = []
    with open('search_list.txt', 'r') as fd:
        fruits = fd.readlines()

        for fruit in fruits:
            search_list.append(fruit.strip())

    return search_list


def print_target_fruits_pos(search_list, fruit_list, fruit_true_pos):
    """Print out the target fruits' pos in the search order

    @param search_list: search order of the fruits
    @param fruit_list: list of target fruits
    @param fruit_true_pos: positions of the target fruits
    """

    print("Search order:")
    n_fruit = 1
    for fruit in search_list:
        for i in range(3):
            if fruit == fruit_list[i]:
                print('{}) {} at [{}, {}]'.format(n_fruit,
                                                  fruit,
                                                  np.round(fruit_true_pos[i][0], 1),
                                                  np.round(fruit_true_pos[i][1], 1)))
        n_fruit += 1


# Waypoint navigation
# the robot automatically drives to a given [x,y] coordinate
# additional improvements:
# you may use different motion model parameters for robot driving on its own or driving while pushing a fruit
# try changing to a fully automatic delivery approach: develop a path-finding algorithm that produces the waypoints
def drive_to_point(waypoint, robot_pose, control_clock, sim=False):
    # imports camera / wheel calibration parameters 
    if sim:
        fileS = "calibration/param/scale_sim.txt"
        scale = np.loadtxt(fileS, delimiter=',')
        fileB = "calibration/param/baseline_sim.txt"
        baseline = np.loadtxt(fileB, delimiter=',')
        # scale /= 2
        print(scale)
        print(baseline)
    else:
        fileS = "calibration/param/scale.txt"
        scale = np.loadtxt(fileS, delimiter=',')
        fileB = "calibration/param/baseline.txt"
        baseline = np.loadtxt(fileB, delimiter=',')

    ####################################################
    # TODO: replace with your codes to make the robot drive to the waypoint
    # One simple strategy is to first turn on the spot facing the waypoint,
    # then drive straight to the way point

    tick = 20  # tick
    turning_tick = 5

    # Robot_pose is a 3*1 matrix
    x_diff = waypoint[0] - robot_pose[0]
    y_diff = waypoint[1] - robot_pose[1]
    x_diff, y_diff = x_diff, y_diff

    distance_to_waypoint = np.hypot(x_diff, y_diff)
    angle_to_waypoint = np.arctan2(y_diff, x_diff)
    turning_angle = angle_to_waypoint - robot_pose[-1]
    angular_velocity = 1 if turning_angle > 0 else -1

    # print(f"x_diff: {x_diff}", f"y_diff: {y_diff}")
    print(f"distance_to_waypoint: {distance_to_waypoint}")
    # print(f"angle_to_waypoint: {angle_to_waypoint}")
    print(f"turning_angle: {turning_angle}")

    # turn towards the waypoint
    turn_time = (abs(turning_angle) * baseline) / (2 * scale * turning_tick)  # replace with your calculation
    print("scale: ", scale)
    print("baseline: ", baseline)
    print("Turning for {:.2f} seconds".format(turn_time))
    command = [0, angular_velocity]

    l_vel, r_vel = ppi.set_velocity(command, turning_tick=turning_tick, time=turn_time)

    # Uses the right and left velocities to estimate the position of the robot with ekf.predict()

    robot_pose, control_clock = get_robot_pose(l_vel, r_vel,
                                               control_clock)  # get_robot_pose now takes l_vel, r_vel, control_clock for the predict step

    # after turning, drive straight to the waypoint
    drive_time = (1.0 / (scale * tick)) * distance_to_waypoint  # replace with your calculation
    print("Driving for {:.2f} seconds".format(drive_time))
    command = [1, 0]
    l_vel, r_vel = ppi.set_velocity(command, tick=tick, time=drive_time)

    ####################################################

    print("Arrived at [{}, {}]".format(waypoint[0], waypoint[1]))
    robot_pose, control_clock = get_robot_pose(l_vel, r_vel,
                                               control_clock)  # After finished driving, estimate pose again
    return robot_pose, control_clock


def get_robot_pose(l_vel, r_vel, control_clock):
    ####################################################
    # TODO: replace with your codes to estimate the pose of the robot
    # We STRONGLY RECOMMEND you to use your SLAM code from M2 here
    dt = time.time() - control_clock  # Calculate dt as time between most recent predict and this predict
    raw_drive_meas = measure.Drive(l_vel, r_vel, dt)  # Obtain driving measurements
    control_clock = time.time()  # Update control_clock

    ekf.predict(raw_drive_meas)

    lms, aruco_img = aruco_det.detect_marker_positions(ppi.get_image())
    ekf.update(lms)

    # update the robot pose [x,y,theta]

    robot_pose = ekf.robot.state  # replace with your calculation
    ####################################################

    return robot_pose, control_clock


# This is a way to initialise EKF
def init_ekf(datadir, ip, sim=False):
    if not sim:
        fileK = "{}intrinsic.txt".format(datadir)
        camera_matrix = np.loadtxt(fileK, delimiter=',')
        fileD = "{}distCoeffs.txt".format(datadir)
        dist_coeffs = np.loadtxt(fileD, delimiter=',')
        fileS = "{}scale.txt".format(datadir)
        scale = np.loadtxt(fileS, delimiter=',')
        if ip == 'localhost':
            scale /= 2
        fileB = "{}baseline.txt".format(datadir)
    else:
        fileK = "{}intrinsic_sim.txt".format(datadir)
        camera_matrix = np.loadtxt(fileK, delimiter=',')
        fileD = "{}distCoeffs.txt".format(datadir)
        dist_coeffs = np.loadtxt(fileD, delimiter=',')
        fileS = "{}scale_sim.txt".format(datadir)
        scale = np.loadtxt(fileS, delimiter=',')
        # if ip == 'localhost':
        #     scale /= 2
        fileB = "{}baseline_sim.txt".format(datadir)
    baseline = np.loadtxt(fileB, delimiter=',')

    print(scale)
    print(baseline)

    robot = Robot(baseline, scale, camera_matrix, dist_coeffs)
    return EKF(robot)


def get_occupancy_map(fruit_true_pos, aruco_true_pos):
    # Have a 400x400 image representing the map, black being free, white being obstacle
    fruit_safety_radius = 30  # How far away in cm
    aruco_safety_radius = 30
    map = np.zeros((400, 400, 3), np.uint8)

    for fruit_pos in fruit_true_pos:
        x = int((fruit_pos[0] + 2) * 100)  # Convert from central coords to left corner coords
        y = int((fruit_pos[1] + 2) * 100)
        c1 = (int(x - fruit_safety_radius / 2), int(y - fruit_safety_radius / 2))
        c2 = (int(x + fruit_safety_radius / 2), int(y + fruit_safety_radius / 2))
        map = cv2.rectangle(map, c1, c2, (255,255,255),-1)

    for aruco_pos in aruco_true_pos:
        x = int((aruco_pos[0] + 2) * 100)  # Convert from central coords to left corner coords
        y = int((aruco_pos[1] + 2) * 100)
        c1 = (int(x - aruco_safety_radius / 2), int(y - aruco_safety_radius / 2))
        c2 = (int(x + aruco_safety_radius / 2), int(y + aruco_safety_radius / 2))
        map = cv2.rectangle(map, c1, c2, (255,255,255),-1)

    return map

def add_to_occupancy_map(new_point, map):
    """
    Adds a point to the occupancy map
    :param new_point: A point in the form of [x,y]
    :param map: The occupancy map
    :return: The updated occupancy map
    """
    default_safety_radius = 30
    x = int((new_point[0] + 2) * 100)  # Convert from central coords to left corner coords
    y = int((new_point[1] + 2) * 100)
    c1 = (int(x - default_safety_radius / 2), int(y - default_safety_radius / 2))
    c2 = (int(x + default_safety_radius / 2), int(y + default_safety_radius / 2))
    map = cv2.rectangle(map, c1, c2, (255, 255, 255), -1)
    return map

# main loop
if __name__ == "__main__":
    parser = argparse.ArgumentParser("Fruit searching")
    parser.add_argument("--map", type=str, default='M4_true_map_5fruit.txt')
    parser.add_argument("--ip", metavar='', type=str, default='localhost')
    parser.add_argument("--port", metavar='', type=int, default=40000)
    # Adds the calibration directory arg (normally default)
    parser.add_argument("--calib_dir", type=str, default="calibration/param/")
    parser.add_argument("--using_sim", action='store_true')
    args, _ = parser.parse_known_args()

    print(args.using_sim)

    ppi = PenguinPi(args.ip,args.port)

    # read in the true map
    fruits_list, fruits_true_pos, aruco_true_pos = read_true_map(args.map)
    search_list = read_search_list()
    print_target_fruits_pos(search_list, fruits_list, fruits_true_pos)

    # Gets the occupancy map
    map = get_occupancy_map(fruits_true_pos,aruco_true_pos)

    # TODO: Summon EKF into existence 
    ekf = init_ekf(args.calib_dir, args.ip, sim=args.using_sim)
    aruco_det = aruco.aruco_detector(
        ekf.robot, marker_length = 0.07) # size of the ARUCO markers

    # Gets the true measurments from the true map and convert to markers
    # NOTE: This will not work for future milestones
    measurements = []
    for i, position in enumerate(aruco_true_pos):
        measurements.append(measure.Marker(position, tag=i))
    # Pass all these markers into the EKF
    ekf.add_landmarks(measurements) 

    # Moved the original get_robot_pose() out of the while loop and hard coded it
    waypoint = [0.0,0.0]
    robot_pose = [0.0,0.0,0.0]

    # The following code is only a skeleton code the semi-auto fruit searching task
    while True:
        # enter the waypoints
        # instead of manually enter waypoints, you can get coordinates by clicking on a map, see camera_calibration.py
        x,y = 0.0,0.0
        x = input("X coordinate of the waypoint: ")
        try:
            x = float(x)
        except ValueError:
            print("Please enter a number.")
            continue
        y = input("Y coordinate of the waypoint: ")
        try:
            y = float(y)
        except ValueError:
            print("Please enter a number.")
            continue

        # New variable used for the ekf.predict() function
        control_clock = time.time()

        # robot drives to the waypoint
        waypoint = [x,y]
        robot_pose, control_clock = drive_to_point(waypoint,robot_pose,control_clock,args.using_sim) # Now returns robot_pose and control_clock. Uses control_clock for predict function
        
        print("Finished driving to waypoint: {}; New robot pose: {}".format(waypoint,robot_pose))

        # exit
        ppi.set_velocity([0, 0])
        uInput = input("Add a new waypoint? [Y/N]")
        if uInput == 'N':
            break
