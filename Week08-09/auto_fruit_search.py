# M4 - Autonomous fruit searching

# basic python packages
from email.mime import image
from lib2to3.pgen2 import driver
import sys, os
import cv2
import numpy as np
import json
import argparse
import time
import matplotlib
import matplotlib.pyplot as plt
import pygame

# import SLAM components
sys.path.insert(0, "{}/slam".format(os.getcwd()))
from slam.ekf import EKF
from slam.robot import Robot
import slam.aruco_detector as aruco

# import utility functions
sys.path.insert(0, "util")
from util.pibot import PenguinPi
import util.measure as measure
import util.DatasetHandler as dh
from util.operate import Operate


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
                    aruco_true_pos[marker_id-1][0] = x
                    aruco_true_pos[marker_id-1][1] = y
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

def calculate_turning_angle(robot_pose, waypoint):
        x_diff = waypoint[0] - robot_pose[0]
        y_diff = waypoint[1] - robot_pose[1]
        x_diff, y_diff = x_diff, y_diff

        
        angle_to_waypoint =  np.arctan2(y_diff, x_diff) # - robot_pose[2]
        turning_angle = angle_to_waypoint - robot_pose[2]
        print(f"[calculate_turning_angle][turning_angle] {turning_angle}")
        return turning_angle
    
def calculate_turn_time(turning_angle, baseline, scale, turning_tick=5):
    return (abs(turning_angle) * baseline) / (2 * scale * turning_tick)

def calculate_drive_time(scale, distance_to_waypoint, tick=20):
    return (1.0 / (scale * tick)) * distance_to_waypoint # replace with your calculation

def calculate_distance_to_waypoint(robot_pose, waypoint):
        x_diff = waypoint[0] - robot_pose[0]
        y_diff = waypoint[1] - robot_pose[1]
        x_diff, y_diff = x_diff, y_diff
        distance_to_waypoint = np.hypot(x_diff, y_diff)
        print(f"[calculate_distance_to_waypoint][distance_to_waypoint] {distance_to_waypoint}")
        return distance_to_waypoint

def partly_turn(operate, turn_time, resolution=2):
    done = False
    then = time.time()
    while not done:
        drive_robot(operate)
        if time.time() - then > turn_time/resolution*2:
            done = True
            lv, rv = operate.pibot.set_velocity(operate.command['motion'])
            drive_meas = measure.Drive(lv, rv, dt=turn_time/resolution)
            operate.update_slam(drive_meas)
    return operate.ekf.robot.state.flatten()

def partly_drive(operate, drive_time, resolution=10):
    done = False
    then = time.time()
    while not done:
        drive_robot(operate)
        if time.time() - then > drive_time/resolution*2:
            done = True
            lv, rv = operate.pibot.set_velocity(operate.command['motion'])
            drive_meas = measure.Drive(lv, rv, dt=drive_time/resolution)
            operate.update_slam(drive_meas)
    return operate.ekf.robot.state.flatten()
def drive_to_point(waypoint, robot_pose, operate:Operate, sim=False):
    # imports camera / wheel calibration parameters 
    if sim:
        fileS = "calibration/param/scale_sim.txt"
        scale = np.loadtxt(fileS, delimiter=',')
        fileB = "calibration/param/baseline_sim.txt"
        baseline = np.loadtxt(fileB, delimiter=',')
    else:
        fileS = "calibration/param/scale.txt"
        scale = np.loadtxt(fileS, delimiter=',')
        fileB = "calibration/param/baseline.txt"
        baseline = np.loadtxt(fileB, delimiter=',')
    
    ####################################################
    # TODO: replace with your codes to make the robot drive to the waypoint
    # One simple strategy is to first turn on the spot facing the waypoint,
    # then drive straight to the way point

    tick = 20 # tick
    turning_tick = 5

    # Robot_pose is a 3*1 matrix
    x_diff = waypoint[0] - robot_pose[0]
    y_diff = waypoint[1] - robot_pose[1]
    x_diff, y_diff = x_diff, y_diff

    turning_angle  = calculate_turning_angle(robot_pose, waypoint)
    angular_velocity = 2 if turning_angle > 0 else -2

    # turn towards the waypoint
    turn_time = calculate_turn_time(turning_angle, baseline, scale)
    print("Turning for {:.2f} seconds".format(turn_time))
    operate.command['motion'] = [0, angular_velocity] if turn_time!=0 else [0, 0]
    done = False
    dt = calculate_turn_time(np.pi/180*5, baseline, scale)
    while not done:
        robot_pose = get_robot_pose(operate, dt).flatten()
        turning_angle = calculate_turning_angle(robot_pose, waypoint)
        if abs(turning_angle) < np.pi/180*10:
            done = True
    operate.command['motion'] = [0,0]

    print(f"Rotated pose: {robot_pose}") 
    # after turning, drive straight to the waypoint
    distance_to_waypoint = calculate_distance_to_waypoint(robot_pose, waypoint)
    drive_time = calculate_drive_time(scale, distance_to_waypoint)
    print("Driving for {:.2f} seconds".format(drive_time))
    operate.command['motion'] = [1, 0] if drive_time != 0 else [0, 0]
    done = False
    then = time.time()
    while not done:
        robot_pose = get_robot_pose(operate).flatten()
        if time.time() - then > drive_time*2:
            done = True
    operate.command['motion'] = [0,0]

    # done = False
    # dt = calculate_drive_time(scale, 0.05)
    # while not done:
    #     robot_pose = get_robot_pose(operate, dt).flatten()
    #     turning_angle = calculate_distance_to_waypoint(robot_pose, waypoint)
    #     if distance_to_waypoint < 0.1:
    #         done = True
    operate.command['motion'] = [0,0]

    ####################################################

    print("Arrived at [{}, {}]".format(waypoint[0], waypoint[1])) 
    print(f"Translated pose: {robot_pose}") 
    return robot_pose


def get_robot_pose(operate:Operate, dt=None):
    ####################################################
    # TODO: replace with your codes to estimate the pose of the robot
    # We STRONGLY RECOMMEND you to use your SLAM code from M2 here
    previous_img = operate.img
    operate.take_pic()
    drive_meas = operate.control(dt)
    operate.update_slam(drive_meas)
    operate.record_data()
    operate.save_image()
    operate.detect_target()
    # visualise
    operate.draw(canvas)
    pygame.display.update()

    # update the robot pose [x,y,theta]

    robot_pose = operate.ekf.robot.state.flatten() # replace with your calculation
    ####################################################

    return robot_pose

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
        if ip == 'localhost':
            scale /= 2
        fileB = "{}baseline_sim.txt".format(datadir)  
    baseline = np.loadtxt(fileB, delimiter=',')
    robot = Robot(baseline, scale, camera_matrix, dist_coeffs)
    return EKF(robot)

# main loop
if __name__ == "__main__":
    parser = argparse.ArgumentParser("Fruit searching")
    parser.add_argument("--map", type=str, default='M4_true_map.txt')
    parser.add_argument("--ip", metavar='', type=str, default='localhost')
    parser.add_argument("--port", metavar='', type=int, default=40000)
    # Adds the calibration directory arg (normally default)
    parser.add_argument("--calib_dir", type=str, default="calibration/param/")
    parser.add_argument("--using_sim", action='store_true')
    # from operate
    parser.add_argument("--save_data", action='store_true')
    parser.add_argument("--play_data", action='store_true')
    parser.add_argument("--ckpt", default='network/scripts/model/model.best.pth')
    args, _ = parser.parse_known_args()

    print(f"Using simulator = {args.using_sim}")

    ppi = PenguinPi(args.ip,args.port)

    # read in the true map
    fruits_list, fruits_true_pos, aruco_true_pos = read_true_map(args.map)
    search_list = read_search_list()
    print_target_fruits_pos(search_list, fruits_list, fruits_true_pos)

    # TODO: Summon EKF into existence 
    ekf = init_ekf(args.calib_dir, args.ip, sim=args.using_sim)
    aruco_det = aruco.aruco_detector(
        ekf.robot, marker_length = 0.07) # size of the ARUCO markers

    # Gets the true measurments from the true map and convert to markers
    # NOTE: This will not work for future milestones
    measurements = []
    for i, position in enumerate(aruco_true_pos):
        measurements.append(measure.Marker(position, tag=i+1, covariance=0*np.eye(2)))# covariance=1e-2*np.eye(2)))
        print(f"{i}, {position}")
    # Pass all these markers into the EKF
    print(f"[__main__][measurement.tags]\n{list(map(lambda x: x.tag, measurements))}\n")
    print(f"[__main__][measurement.positions]\n{list(map(lambda x: x.position.tolist(), measurements))}\n")
    print(f"[__main__][measurement.covariances]\n{list(map(lambda x: x.covariance, measurements))}\n")
    ekf.add_landmarks(measurements) 

    # Moved the original get_robot_pose() out of the while loop and hard coded it
    waypoint = [0.0,0.0]
    robot_pose = [0.0,0.0,0.0]

    pygame.font.init() 
    TITLE_FONT = pygame.font.Font('pics/8-BitMadness.ttf', 35)
    TEXT_FONT = pygame.font.Font('pics/8-BitMadness.ttf', 40)
    
    width, height = 700, 660
    canvas = pygame.display.set_mode((width, height))
    pygame.display.set_caption('ECE4078 2021 Lab')
    pygame.display.set_icon(pygame.image.load('pics/8bit/pibot5.png'))
    canvas.fill((0, 0, 0))
    splash = pygame.image.load('pics/loading.png')
    pibot_animate = [pygame.image.load('pics/8bit/pibot1.png'),
                     pygame.image.load('pics/8bit/pibot2.png'),
                     pygame.image.load('pics/8bit/pibot3.png'),
                     pygame.image.load('pics/8bit/pibot4.png'),
                     pygame.image.load('pics/8bit/pibot5.png')]
    pygame.display.update()

    start = True # False

    counter = 40
    while not start:
        for event in pygame.event.get():
            if event.type == pygame.KEYDOWN:
                start = True
        canvas.blit(splash, (0, 0))
        x_ = min(counter, 600)
        if x_ < 600:
            canvas.blit(pibot_animate[counter%10//2], (x_, 565))
            pygame.display.update()
            counter += 2

    operate = Operate(args)
    operate.ekf = ekf

    operate.run_slam()
    get_robot_pose(operate)
    
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
        
        robot_pose = drive_to_point(waypoint, robot_pose, operate, sim=args.using_sim) # Now returns robot_pose and control_clock. Uses control_clock for predict function
        then = time.time()
        while time.time() - then < 3:
            pass # get_robot_pose(operate)
        print("Finished driving to waypoint: {}; New robot pose: {}".format(waypoint,robot_pose))

        # exit
        ppi.set_velocity([0, 0])
        # visualise
        operate.draw(canvas)
        pygame.display.update()
        uInput = input("Add a new waypoint? [Y/N]")
        if uInput == 'N' or uInput == 'n':
            break

