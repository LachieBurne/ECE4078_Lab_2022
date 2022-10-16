import numpy as np
import cv2
import os, sys
import time
import argparse
import json

# import utility functions
sys.path.insert(0, "{}/utility".format(os.getcwd()))
from util.pibot import PenguinPi  # access the robot
import util.DatasetHandler as dh  # save/load functions
import util.measure as measure  # measurements
import pygame  # python package for GUI
import shutil  # python package for file operations

# import SLAM components you developed in M2
sys.path.insert(0, "{}/slam".format(os.getcwd()))
from slam.ekf import EKF
from slam.robot import Robot
import slam.aruco_detector as aruco

# import CV components
sys.path.insert(0, "{}/network/".format(os.getcwd()))
sys.path.insert(0, "{}/network/scripts".format(os.getcwd()))
from network.scripts.detector import Detector

# from AStar import *
from util.Helper import *
from TargetPoseEst import *
from PathPlanner import *


class Operate:
    def __init__(self, args):
        self.folder = 'pibot_dataset/'
        if not os.path.exists(self.folder):
            os.makedirs(self.folder)
        else:
            shutil.rmtree(self.folder)
            os.makedirs(self.folder)

        # initialise data parameters
        if args.play_data:
            self.pibot = dh.DatasetPlayer("record")
        else:
            self.pibot = PenguinPi(args.ip, args.port)

        # initialise SLAM parameters
        self.ekf = self.init_ekf(args.calib_dir, args.ip)
        self.aruco_det = aruco.aruco_detector(
            self.ekf.robot, marker_length=0.07)  # size of the ARUCO markers

        if args.save_data:
            self.data = dh.DatasetWriter('record')
        else:
            self.data = None
        self.output = dh.OutputWriter('lab_output')
        self.command = {'motion': [0, 0],
                        'inference': False,
                        'output': False,
                        'save_inference': False,
                        'save_image': False}

        self.quit = False
        self.pred_fname = ''
        self.request_recover_robot = False
        self.file_output = None
        self.ekf_on = True
        self.double_reset_comfirm = 0
        self.image_id = 0
        self.notification = 'Press ENTER to start SLAM'
        # a 5min timer
        self.count_down = 300
        self.start_time = time.time()
        self.control_clock = time.time()
        # initialise images
        self.img = np.zeros([240, 320, 3], dtype=np.uint8)
        self.aruco_img = np.zeros([240, 320, 3], dtype=np.uint8)
        self.detector_output = np.zeros([240, 320], dtype=np.uint8)
        if args.ckpt == "":
            self.detector = None
            self.network_vis = cv2.imread('pics/8bit/detector_splash.png')
        else:
            self.detector = Detector(args.ckpt, use_gpu=False)
            self.network_vis = np.ones((240, 320, 3)) * 100
        self.bg = pygame.image.load('pics/gui_mask.jpg')

        self.markers = np.zeros((10,2))
        self.unknown_obs = {}
        self.tags = []
        self.goals = []
        self.start = False
        
        self.no_planning = True

    # wheel control
    def control(self):
        if args.play_data:
            lv, rv = self.pibot.set_velocity()
        else:
            lv, rv = self.pibot.set_velocity(
                self.command['motion'])
        if not self.data is None:
            self.data.write_keyboard(lv, rv)
        dt = time.time() - self.control_clock
        drive_meas = measure.Drive(lv, rv, dt)
        self.control_clock = time.time()
        return drive_meas

    # camera control
    def take_pic(self):
        self.img = self.pibot.get_image()
        if not self.data is None:
            self.data.write_image(self.img)

    # SLAM with ARUCO markers       
    def update_slam(self, drive_meas):
        lms, self.aruco_img = self.aruco_det.detect_marker_positions(self.img)
        if self.request_recover_robot:
            is_success = self.ekf.recover_from_pause(lms)
            if is_success:
                self.notification = 'Robot pose is successfuly recovered'
                self.ekf_on = True
            else:
                self.notification = 'Recover failed, need >2 landmarks!'
                self.ekf_on = False
            self.request_recover_robot = False
        elif self.ekf_on:  # and not self.debug_flag:
            self.ekf.predict(drive_meas)
            self.ekf.add_landmarks(lms)
            self.ekf.update(lms)

    # using computer vision to detect targets
    def detect_target(self):
        if self.command['inference'] and self.detector is not None:
            self.detector_output, self.network_vis = self.detector.detect_single_image(self.img)
            # print(f"detector_output:{self.detector_output}\n,network_vis:{self.network_vis}")
            self.command['inference'] = False
            self.file_output = (self.detector_output, self.ekf)
            self.notification = f'{len(np.unique(self.detector_output)) - 1} target type(s) detected'

    # save raw images taken by the camera
    def save_image(self):
        f_ = os.path.join(self.folder, f'img_{self.image_id}.png')
        if self.command['save_image']:
            image = self.pibot.get_image()
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            cv2.imwrite(f_, image)
            self.image_id += 1
            self.command['save_image'] = False
            self.notification = f'{f_} is saved'

    # wheel and camera calibration for SLAM
    def init_ekf(self, datadir, ip):
        fileK = "{}intrinsic.txt".format(datadir)
        camera_matrix = np.loadtxt(fileK, delimiter=',')
        fileD = "{}distCoeffs.txt".format(datadir)
        dist_coeffs = np.loadtxt(fileD, delimiter=',')
        if ip == 'localhost':
            fileS = "{}scale_sim.txt".format(datadir)
            scale = np.loadtxt(fileS, delimiter=',')
            fileB = "{}baseline_sim.txt".format(datadir)
            baseline = np.loadtxt(fileB, delimiter=',')
            scale /= 2
        else:
            fileS = "{}scale.txt".format(datadir)
            scale = np.loadtxt(fileS, delimiter=',')
            fileB = "{}baseline.txt".format(datadir)
            baseline = np.loadtxt(fileB, delimiter=',')
        robot = Robot(baseline, scale, camera_matrix, dist_coeffs)
        return EKF(robot)

    # save SLAM map
    def record_data(self):
        if self.command['output']:
            self.output.write_map(self.ekf)
            self.notification = 'Map is saved'
            self.command['output'] = False
        # save inference with the matching robot pose and detector labels
        if self.command['save_inference']:
            if self.file_output is not None:
                # image = cv2.cvtColor(self.file_output[0], cv2.COLOR_RGB2BGR)
                self.pred_fname = self.output.write_image(self.file_output[0],
                                                          self.file_output[1])
                self.notification = f'Prediction is saved to {operate.pred_fname}'
            else:
                self.notification = f'No prediction in buffer, save ignored'
            self.command['save_inference'] = False

    # paint the GUI            
    def draw(self, canvas):
        canvas.blit(self.bg, (0, 0))
        text_colour = (220, 220, 220)
        v_pad = 40
        h_pad = 20

        # paint SLAM outputs
        ekf_view = self.ekf.draw_slam_state(res=(320, 480 + v_pad),
                                            not_pause=self.ekf_on)
        canvas.blit(ekf_view, (2 * h_pad + 320, v_pad))
        robot_view = cv2.resize(self.aruco_img, (320, 240))
        self.draw_pygame_window(canvas, robot_view,
                                position=(h_pad, v_pad)
                                )

        # for target detector (M3)
        detector_view = cv2.resize(self.network_vis,
                                   (320, 240), cv2.INTER_NEAREST)
        self.draw_pygame_window(canvas, detector_view,
                                position=(h_pad, 240 + 2 * v_pad)
                                )

        # canvas.blit(self.gui_mask, (0, 0))
        self.put_caption(canvas, caption='SLAM', position=(2 * h_pad + 320, v_pad))
        self.put_caption(canvas, caption='Detector',
                         position=(h_pad, 240 + 2 * v_pad))
        self.put_caption(canvas, caption='PiBot Cam', position=(h_pad, v_pad))

        notifiation = TEXT_FONT.render(self.notification,
                                       False, text_colour)
        canvas.blit(notifiation, (h_pad + 10, 596))

        time_remain = self.count_down - time.time() + self.start_time
        if time_remain > 0:
            time_remain = f'Count Down: {time_remain:03.0f}s'
        elif int(time_remain) % 2 == 0:
            time_remain = "Time Is Up !!!"
        else:
            time_remain = ""
        count_down_surface = TEXT_FONT.render(time_remain, False, (50, 50, 50))
        canvas.blit(count_down_surface, (2 * h_pad + 320 + 5, 530))
        return canvas

    @staticmethod
    def draw_pygame_window(canvas, cv2_img, position):
        cv2_img = np.rot90(cv2_img)
        view = pygame.surfarray.make_surface(cv2_img)
        view = pygame.transform.flip(view, True, False)
        canvas.blit(view, position)

    @staticmethod
    def put_caption(canvas, caption, position, text_colour=(200, 200, 200)):
        caption_surface = TITLE_FONT.render(caption,
                                            False, text_colour)
        canvas.blit(caption_surface, (position[0], position[1] - 25))

    # keyboard teleoperation        
    def update_keyboard(self):
        for event in pygame.event.get():
            # drive forward
            keys = pygame.key.get_pressed()
            # stop
            if event.type == pygame.KEYDOWN and event.key == pygame.K_SPACE:
                self.command['motion'] = [0, 0]
            # save image
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_i:
                self.command['save_image'] = True
            # save SLAM map
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_s:
                self.command['output'] = True
            # run SLAM
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_RETURN:
                operate.no_planning = False
                # self.start = True
                # if self.arrived_goal == []:
                #     self.start_planning = True
                # else:
                #     self.command['inference'] = True # force it True so it constantly detects fruits
                #     self.ekf_on = True
                #     self.notification = 'Driving to goal {}'.format(self.goal_num + 1)
                #     print('MOVING TO GOAL {} --------------------------------------------------------'.format(self.goal_num + 1))
                #     self.nextgoal = False

            # object detector
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_p:
                self.command['inference'] = True
            # quit
            elif event.type == pygame.QUIT:
                self.quit = True
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                self.quit = True

            else:
                self.command['motion'] = [0, 0]
        if self.quit:
            pygame.quit()
            sys.exit()

    def read_true_map(self, args):
        """Read the ground truth map and output the pose of the ArUco markers and 3 types of target fruit to search
        @param fname: filename of the map
        @return:
            1) list of target fruits, e.g. ['apple', 'pear', 'lemon']
            2) locations of the target fruits, [[x1, y1], ..... [xn, yn]]
            3) locations of ArUco markers in order, i.e. pos[9, :] = position of the aruco10_0 marker
        """
        fname = args.map
        with open(fname, 'r') as fd:
            gt_dict = json.load(fd)
            fruit_list = []
            fruit_true_pos = []
            aruco_true_pos = np.empty([10, 2])
        search_list = self.read_search_list(args)
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
                    aruco_true_pos[marker_id - 1][0] = x
                    aruco_true_pos[marker_id - 1][1] = y
            else:
                fruit_list.append(key[:-2])
                if len(fruit_true_pos) == 0:
                    fruit_true_pos = np.array([[x, y]])
                else:
                    fruit_true_pos = np.append(fruit_true_pos, [[x, y]], axis=0)
        search_list_pose = []
        fruit_list_e = []
        for j in range(len(search_list)):
            target_fruit = search_list[j]
            for i in range(len(search_list)):
                fruit_name = fruit_list[i]
                # print(i, fruit_name)
                if fruit_name == target_fruit:
                    search_list_pose.append(fruit_true_pos[i])
                    fruit_list_e.append(fruit_name)
                    continue
        print("--------Search order: ", fruit_list_e, "--------")
        # time.sleep(3)

        return fruit_list_e, fruit_true_pos, aruco_true_pos, search_list_pose

    def read_search_list(self, args):
        """Read the search order of the target fruits
        @return: search order of the target fruits
        """
        search_list = []
        list = args.search_list
        with open(list, 'r') as fd:
            fruits = fd.readlines()

            for fruit in fruits:
                search_list.append(fruit.strip())

        return search_list


    def update_markers(self, temp_tags, markers, tags):
        num = len(temp_tags)
        marker_list = np.zeros((2, num))
        tag_list = []

        for i in range(num):
            tag = temp_tags[i]

            try:
                temp_marker_tag = np.where(np.array(tags) == tag)[0][0]
            except IndexError:
                temp_marker_tag = np.where(np.array(tags) == 10)[0][0]
            temp_tag = tags[temp_marker_tag - 1]
            temp_marker = markers[temp_marker_tag - 1].T

            marker_list[:, i] = temp_marker
            tag_list.append(int(temp_tag))

        return marker_list, tag_list

    ## Kelvin Added ##########################
    # Calculate the fruit positions from fruits (Copied from TargetPoseEst.py)
    def calc_fruit_pos(self, completed_img_dict):
        intrinsic_filename = 'intrinsic_sim.txt' if args.using_sim else 'intrinsic.txt'
        calibration_path = './calibration/param/'
        fileK = calibration_path + intrinsic_filename
        camera_matrix = np.loadtxt(fileK, delimiter=',')
        target_map = estimate_pose(camera_matrix, completed_img_dict, args.using_sim)

        return target_map
    ##########################################

    ## Kelvin Added ##########################
    # Using YOLO to detect fruits
    def detect_target(self):
        if self.run_path or self.call_calib:
            self.command['inference'] = True
        if self.command['inference'] and self.detector is not None:
            self.detector_output, self.network_vis, completed_img_dict = self.detector.detect_single_image(self.img, self.ekf.robot.state, args.using_sim)
            self.command['inference'] = False
            self.file_output = (self.detector_output, self.ekf)
            target_est = self.calc_fruit_pos(completed_img_dict)
            """ check target_est, if detected fruits as obstacles, stop and replan path after current step"""
            for fruit in target_est:
                if fruit not in operate.read_search_list():
                    # Once Detected Fruits that are not in the search list, force stop and replan path even if there is no path collision
                    if self.run_path and self.current_waypoint_idx != 1:
                        self.start_planning = True
                        self.run_path = False
                        self.current_waypoint_idx = 1
                        self.turning = True
                        print('Detected obstacle. Replanning path.\n')

                    self.unknown_obstacles[fruit] = np.array([target_est[fruit]['x'], target_est[fruit]['y']])

        #visualise
        pygame.display.update()
        self.draw(canvas)
    ##########################################

    def drive_to_point(self, waypoint):

        threshold_angle = 0.1

        x_r = self.ekf.robot.state[0]
        y_r = self.ekf.robot.state[1]
        x_w = float(waypoint[0])
        y_w = float(waypoint[1])
        x_diff = x_w - x_r
        y_diff = y_w - y_r
        theta_r = self.ekf.robot.state[2]

        ## limit theta_r between -pi and pi

        while (abs(theta_r) > np.pi):
            if theta_r > 0:
                theta_r -= 2 * np.pi
            else:
                theta_r += 2 * np.pi

        ## in robot frame, how much angle diff between robot and target
        theta_diff = np.arctan2(y_diff, x_diff)

        ## Angle needs to turn = Angle diff from robot - robot pose in world
        theta_turn = theta_diff - theta_r

        if int((time.time() - print_time)) % 2 == 0:
            print("Theta to turn: " + str(theta_turn)[1:6] + " and current robot pose: " + str(theta_r)[1:6])

        if abs(theta_turn) >= threshold_angle:
            if y_diff >= 0:
                if theta_turn > 0:
                    self.command['motion'] = [0, 1]
                else:
                    self.command['motion'] = [0, -1]
            else:
                if theta_turn <= 0:
                    self.command['motion'] = [0, -1]
                else:
                    self.command['motion'] = [0, 1]
        else:
            self.command['motion'] = [1, 0]

    def reset_robot_pose(self):
            angle = self.ekf.robot.state[2]
            while (abs(angle) > np.pi):
                if angle > 0:
                    angle -= 2 * np.pi
                else:
                    angle += 2 * np.pi

            if angle < 0:
                self.command['motion'] = [0, 1]
            else:
                self.command['motion'] = [0, -1]


# ## Kelvin Added ##########################
#     # Path navigation 
#     def navigate_path(self):
#         # Position Calibration Phase
#         # Turn 8 times to calibrate robot's position
#         if self.call_calib:
#             angle_diff = get_angle_robot_to_goal(self.ekf.robot.state.squeeze(), waypoint)
#             angular_velocity = 1 if angle_diff > 0 else -1
#             self.command['motion'] = [0, angular_velocity]

#             lms, self.aruco_img = self.aruco_det.detect_marker_positions(self.img)

#             if len(lms) >= 1:
#                 self.call_calib = False
#                 self.start_planning = True
#                 self.notification = 'Calibration Over'
#                 self.calib_turn_number = 0
#                 self.command['inference'] = True
#                 # self.current_waypoint_idx = 1
#         # Path Following Phase (from auto_fruit_search.py)
#         elif self.run_path:
#             #try
#             lms, self.aruco_img = self.aruco_det.detect_marker_positions(self.img)

#             if len(lms) == 0:
#                 self.lost_count +=1
#                 if self.lost_count > 100:
#                     self.call_calib=True
#                     self.lost_count=0

#             waypoint = self.path[self.current_waypoint_idx]  
#             angle_thresh = 0.05       
#             dist_thresh = 0.05  
#             if len(self.path) > 3:
#                 print('Driving to :', waypoint)
#                 x_r, y_r, theta_r = self.ekf.robot.state
#                 angle_diff = get_angle_robot_to_goal(self.ekf.robot.state.squeeze(), waypoint)
#                 dist_diff = get_distance_robot_to_goal(self.ekf.robot.state.squeeze(), waypoint)
#                 if abs(angle_diff) >= angle_thresh:
#                     angular_velocity = 1 if angle_diff > 0 else -1
#                     self.command['motion'] = [0, angular_velocity]
#                 elif dist_diff >= dist_thresh:
#                     self.command['motion'] = [1, 0]
#                 else:
#                     self.current_waypoint_idx += 1                
                    
#                 if self.current_waypoint_idx == len(self.path)-2:
#                     self.at_goal = True
#                     self.run_path = False
#                     self.call_calib = True
#                     self.command['inference'] = False 
#             else:
#                 self.at_goal = True
#                 # lv, rv, dt = 0, 0, 0
                
#             if self.at_goal:
#                 self.at_goal = False
#                 self.goal_num += 1
#                 self.run_path = False
#                 self.arrived_goal.append(self.path[-1])
#                 self.notification = 'Arrived at Goal {}'.format(self.goal_num)
#                 operate.save_image()
#                 time.sleep(5)
#                 self.nextgoal = True
#                 self.call_calib = True



if __name__ == "__main__":

    global print_time
    print_time = time.time()

    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", metavar='', type=str, default='localhost')
    parser.add_argument("--port", metavar='', type=int, default=40000)
    parser.add_argument("--calib_dir", type=str, default="calibration/param/")
    parser.add_argument("--save_data", action='store_true')
    parser.add_argument("--play_data", action='store_true')
    parser.add_argument("--ckpt", default='')
    parser.add_argument("--map", default='M4_true_map_3fruits.txt')
    parser.add_argument("--search_list", default='search_list.txt')
    args, _ = parser.parse_known_args()

    pygame.font.init()
    TITLE_FONT = pygame.font.Font('pics/8-BitMadness.ttf', 35)
    TEXT_FONT = pygame.font.Font('pics/8-BitMadness.ttf', 40)

    width, height = 700, 660
    canvas = pygame.display.set_mode((width, height))
    pygame.display.set_caption('ECE4078 2022 Lab')
    pygame.display.set_icon(pygame.image.load('pics/8bit/pibot5.png'))
    canvas.fill((0, 0, 0))
    splash = pygame.image.load('pics/loading.png')
    pibot_animate = [pygame.image.load('pics/8bit/pibot1.png'),
                     pygame.image.load('pics/8bit/pibot2.png'),
                     pygame.image.load('pics/8bit/pibot3.png'),
                     pygame.image.load('pics/8bit/pibot4.png'),
                     pygame.image.load('pics/8bit/pibot5.png')]
    pygame.display.update()

    start = False

    counter = 40
    while not start:
        for event in pygame.event.get():
            if event.type == pygame.KEYDOWN:
                start = True
        canvas.blit(splash, (0, 0))
        x_ = min(counter, 600)
        if x_ < 600:
            canvas.blit(pibot_animate[counter % 10 // 2], (x_, 565))
            pygame.display.update()
            start = True
            counter += 2

    operate = Operate(args)
    fruit_found = False

    n_observed_markers = len(operate.ekf.taglist)
    if n_observed_markers == 0:
        if not operate.ekf_on:
            operate.notification = 'SLAM is running'
            operate.ekf_on = True
        else:
            operate.notification = '> 2 landmarks is required for pausing'
    elif n_observed_markers < 3:
        operate.notification = '> 2 landmarks is required for pausing'
    else:
        if not operate.ekf_on:
            operate.request_recover_robot = True
        operate.ekf_on = not operate.ekf_on
        if operate.ekf_on:
            operate.notification = 'SLAM is running'
        else:
            operate.notification = 'SLAM is paused'

    fruits_list, fruit_true_pos, aruco_true_pos, search_list_pose = operate.read_true_map(args)

    operate.goals = search_list_pose
    operate.tags = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]

    a_star = AStar()

    for i, fruit in enumerate(fruits_list):
        print(f"Fruit: {fruit}")
        print(f"Coords: {operate.goals[i]}")

    while operate.no_planning:
        operate.update_keyboard()
        operate.take_pic()
        drive_meas = operate.control()
        operate.markers, tag_list = operate.update_markers(operate.ekf.taglist, aruco_true_pos, operate.tags)
        operate.ekf.taglist = tag_list
        operate.ekf.markers = operate.markers
        operate.update_slam(drive_meas)
        robot_pose = operate.ekf.robot.state[:2]
        operate.record_data()
        operate.save_image()
        # operate.detect_target()
        # visualise
        operate.draw(canvas)
        pygame.display.update()

    for i in range(len(operate.goals)):
        reset_robot_pose = True
        state = operate.ekf.robot.state[:2].squeeze()
        a_star.plan_path(r_state=state, goals=operate.goals, goal_num=i, markers=operate.markers, unknown_obs=operate.unknown_obs)
        rx, ry = a_star.rx, a_star.ry

        robot_pose = [state[0],state[1]]

        for j in range(len(rx)):
            # input_waypoint_x = input("Input x coord:")
            # input_waypoint_y = input("Input y coord:")
            # waypoint = [input_waypoint_x, input_waypoint_y]
            waypoint = [rx[j], ry[j]]

            dist = get_distance_robot_to_goal(robot_pose,np.array([waypoint[0],waypoint[1]]))
            while dist > 0.05:
                fruit_distance = get_distance_robot_to_goal(robot_pose,search_list_pose[i])
                dist = get_distance_robot_to_goal(robot_pose, np.array([waypoint[0], waypoint[1]]))
                if fruit_distance < 0.25:
                    fruit_found = True
                    break
                if reset_robot_pose:
                    if not operate.ekf.robot.state[2] <= 0.05:
                        operate.reset_robot_pose()
                        time.sleep(1)
                    else:
                        reset_robot_pose = False
                else:
                    if int((time.time() - print_time)) % 2 == 0:
                        print_time = time.time()
                    print(f"next waypoint:{float(waypoint[0]):2f}, {float(waypoint[1]):2f}")
                    print(f"Distance to waypoint: {dist}")
                    print(f"Predicted robot position: {operate.ekf.robot.state[:2]}")
                    operate.drive_to_point(waypoint)

                operate.update_keyboard()
                operate.take_pic()

                # operate.detect_target()

                drive_meas = operate.control()
                operate.markers, tag_list = operate.update_markers(operate.ekf.taglist, aruco_true_pos, operate.tags)
                operate.ekf.taglist = tag_list
                operate.ekf.markers = operate.markers
                operate.update_slam(drive_meas)
                robot_pose = operate.ekf.robot.state[:2]
                operate.record_data()
                operate.save_image()
                # operate.detect_target()
                # visualise
                operate.draw(canvas)
                pygame.display.update()

            operate.command['motion'] = [0,0]
            operate.control()
            # at_waypoint = input("At fruit? (Y/N)")
            # if at_waypoint == "Y":
            #     break
            if fruit_found:
                fruit_found = False
                print("Fruit found!")
                time.sleep(5)
                break