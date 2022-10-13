# teleoperate the robot and perform SLAM
# will be extended in following milestones for system integration

# basic python packages
import numpy as np
import cv2 
import os, sys
import time
## Kelvin Added ##########################
from auto_fruit_search import *
from TargetPoseEst import *
from Astar import *
##########################################

# import utility functions
sys.path.insert(0, "{}/utility".format(os.getcwd()))
from util.pibot import PenguinPi # access the robot
import util.DatasetHandler as dh # save/load functions
import util.measure as measure # measurements
from util.Helper import *
import pygame # python package for GUI
import shutil # python package for file operations

# import SLAM components
sys.path.insert(0, "{}/slam".format(os.getcwd()))
from slam.ekf import EKF
from slam.robot import Robot
import slam.aruco_detector as aruco

## Kelvin Added ##########################
# import YOLO Dependencies if needed (not sure what to add yet)
sys.path.insert(0,"{}/network/".format(os.getcwd()))
sys.path.insert(0,"{}/network/scripts".format(os.getcwd()))
from network.scripts.detector import Detector
##########################################

class Operate:
    def __init__(self, args, goals, aruco_pos):
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
            self.ekf.robot, marker_length = 0.07) # size of the ARUCO markers

        if args.save_data:
            self.data = dh.DatasetWriter('record')
        else:
            self.data = None
        self.output = dh.OutputWriter('lab_output')
        self.command = {'motion':[0, 0], 
                        'inference': False,
                        'output': False,
                        'save_inference': False,
                        'save_image': False}
        self.quit = False
        self.pred_fname = ''
        self.request_recover_robot = False
        self.file_output = None
        self.ekf_on = False
        self.double_reset_comfirm = 0
        self.image_id = 0
        self.notification = 'Press ENTER to start SLAM'

        ## Kelvin Added ##########################
        # Initialize global variables to flag algorithm phases
        self.start_planning = False         # Flag for Path Planning to Start
        self.goals = goals                  # Waypoints/Goals for Fruit Search
        self.unknown_obstacles = {}         # List of Unknown Obstacles (Fruits)
        self.goal_num = 0                   # Number of fruits to be searched
        self.aruco_pos = aruco_pos          # Position of Aruco Markers
        self.path = None                    # Path for Fruit Search
        self.run_path = False               # Flag for running Planned Path
        self.current_waypoint_idx = 1       # Index of current waypoint
        self.turning = True                 # Flag for Robot Turning
        self.call_calib = False             # Flag for Calibration (Wheel and Baseline)
        self.check = False              
        self.calib_turn_number = 0
        self.threshold = 0.02
        self.arrived_goal = []              # List of Arrived Waypoints/Goals
        self.at_goal = False                # Flag for being at a Waypoint/Goal
        self.nextgoal = False               # Flag for travelling to next Waypoint/Goal

        self.lost_count = 0

        # Weight File for YOLO
        if args.using_sim:
            self.ckpt = 'final_weights/best_sim.pt'
        else:                 
            self.ckpt = 'final_weights/best_real.pt'  
        ##########################################

        # a 5min timer
        self.count_down = 300
        self.start_time = time.time()
        self.control_clock = time.time()

        # initialise images
        self.img = np.zeros([240,320,3], dtype=np.uint8)
        self.aruco_img = np.zeros([240,320,3], dtype=np.uint8)
        ## Kelvin Added ##########################
        # initialize YOLO
        if self.ckpt == "":
            self.detector = None
            self.network_vis = cv2.imread('pics/8bit/detector_splash.png')
        else:
            self.detector = Detector(self.ckpt, use_gpu=False)
            self.network_vis = np.ones((240, 320,3))* 100 # Initialize Gray Background
        ##########################################
        self.bg = pygame.image.load('pics/gui_mask.jpg')

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
        ## Kelvin Added ##########################
        # Control measurements from Autonomous Driving
        if self.run_path or self.call_calib:
            lv, rv, dt = self.navigate_path()
            time.sleep(0.5)
        ##########################################
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
        previous_img = self.img
        while np.array_equal(previous_img, self.img):
            self.take_pic()
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
        elif self.ekf_on: # and not self.debug_flag:
            self.ekf.predict(drive_meas)
            self.ekf.add_landmarks(lms)
            self.ekf.update(lms)
            if self.run_path:
                print('current pose: ', self.ekf.robot.state.squeeze(), '\n')

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
                if fruit not in read_search_list():
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

    # save images taken by the camera
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
        if args.using_sim:
            fileK = "{}intrinsic_sim.txt".format(datadir)
            fileS = "{}scale_sim.txt".format(datadir)
            fileB = "{}baseline_sim.txt".format(datadir)
        else:
            fileK = "{}intrinsic.txt".format(datadir)
            fileS = "{}scale.txt".format(datadir)
            fileB = "{}baseline.txt".format(datadir)
        fileD = "{}distCoeffs.txt".format(datadir)

        camera_matrix = np.loadtxt(fileK, delimiter=',')
        scale = np.loadtxt(fileS, delimiter=',')
        baseline = np.loadtxt(fileB, delimiter=',')
        dist_coeffs = np.loadtxt(fileD, delimiter=',')

        robot = Robot(baseline, scale, camera_matrix, dist_coeffs)
        return EKF(robot)

    # save SLAM map
    def record_data(self):
        if self.command['output']:
            self.output.write_map(self.ekf)
            self.notification = 'Map is saved'
            self.command['output'] = False

        ## Kelvin Added ##########################
        ## Save Fruit Location Predictions
        if self.command['save_inference']:
            if self.file_output is not None:
                self.pred_fname = self.output.write_image(self.file_output[0],
                                                        self.file_output[1])
                self.notification = f'Prediction is saved to {operate.pred_fname}'
            else:
                self.notification = f'No predictions detected, nothing saved'
            self.command['save_inference'] = False
        ##########################################

    # paint the GUI            
    def draw(self, canvas):
        canvas.blit(self.bg, (0, 0))
        text_colour = (220, 220, 220)
        v_pad = 40
        h_pad = 20

        # paint SLAM outputs
        ekf_view = self.ekf.draw_slam_state(res=(320, 480+v_pad),
            not_pause = self.ekf_on)
        canvas.blit(ekf_view, (2*h_pad+320, v_pad))
        robot_view = cv2.resize(self.aruco_img, (320, 240))
        self.draw_pygame_window(canvas, robot_view, 
                                position=(h_pad, v_pad)
                                )

        ## Kelvin Added ##########################
        # For Detector to work in GUI
        detector_view = cv2.resize(self.network_vis,
                                   (320, 240), cv2.INTER_NEAREST)
        self.draw_pygame_window(canvas, detector_view, 
                                position=(h_pad, 240+2*v_pad)
                                )
        ##########################################

        # canvas.blit(self.gui_mask, (0, 0))
        self.put_caption(canvas, caption='SLAM', position=(2*h_pad+320, v_pad)) # M2
        self.put_caption(canvas, caption='Detector (M3)',
                         position=(h_pad, 240+2*v_pad)) # M3
        self.put_caption(canvas, caption='PiBot Cam', position=(h_pad, v_pad))

        notifiation = TEXT_FONT.render(self.notification,
                                          False, text_colour)
        canvas.blit(notifiation, (h_pad+10, 596))

        time_remain = self.count_down - time.time() + self.start_time
        if time_remain > 0:
            time_remain = f'Count Down: {time_remain:03.0f}s'
        elif int(time_remain)%2 == 0:
            time_remain = "Time Is Up !!!"
        else:
            time_remain = ""
        count_down_surface = TEXT_FONT.render(time_remain, False, (50, 50, 50))
        canvas.blit(count_down_surface, (2*h_pad+320+5, 530))
        return canvas
    
    ## Kelvin Added ##########################
    # Path navigation 
    def navigate_path(self):

        tick = 20
        turning_tick = 30
        scale = self.ekf.robot.wheels_scale
        baseline = self.ekf.robot.wheels_width
        
        # Position Calibration Phase
        # Turn 8 times to calibrate robot's position
        if self.call_calib:
            angle_diff = np.pi/4
            
            if self.calib_turn_number == 0:
                self.notification = "Calibration Started"

            dt = (abs(angle_diff)*baseline) / (2 * turning_tick * scale)

            lv, rv = self.pibot.set_velocity([0, -2], turning_tick=turning_tick, time=dt)

            previous_img = self.img
            while np.array_equal(previous_img, self.img):
                self.take_pic()

            lms, self.aruco_img = self.aruco_det.detect_marker_positions(self.img)
            print(len(lms))
            if len(lms) >= 2:
                self.ekf.recover_from_pause(lms)
                self.calib_turn_number = 7

            self.calib_turn_number += 1     
            if self.calib_turn_number == 8:
                self.call_calib = False
                self.start_planning = True
                self.notification = 'Calibration Over'
                self.calib_turn_number = 0
                self.command['inference'] = True
                # self.current_waypoint_idx = 1
        # Path Following Phase (from auto_fruit_search.py)
        elif self.run_path:
            #try
            lms, self.aruco_img = self.aruco_det.detect_marker_positions(self.img)
            print(len(lms))
            if len(lms) == 0:
                self.lost_count +=1
                if self.lost_count > 1:
                    self.call_calib=True
                    self.lost_count=0

            waypoint = self.path[self.current_waypoint_idx]  
            # angle_thresh = 0.05         
            if len(self.path) > 3:
                # print('Driving to :', waypoint)
                # x_r, y_r, theta_r = self.ekf.robot.state
                # angle_diff = get_angle_robot_to_goal(self.ekf.robot.state.squeeze(), waypoint)
                # if angle_diff >= angle_thresh:
                #     self.turning = 1

                if self.turning:
                    angle_diff = get_angle_robot_to_goal(self.ekf.robot.state.squeeze(), waypoint)
                    dt = (abs(angle_diff)*baseline) / (2*turning_tick*scale)
                    print("Turning for {:.5f} seconds".format(dt))

                    angular_velocity = 1 if angle_diff > 0 else -1
                    lv, rv = self.pibot.set_velocity([0, angular_velocity], turning_tick=turning_tick, time=dt)            
                    print("Robot state: ")
                    print(self.ekf.robot.state.squeeze())
                else:
                    dist_diff = get_distance_robot_to_goal(self.ekf.robot.state.squeeze(), waypoint)

                    dt = (1.0 / (scale * tick)) * dist_diff
                    print("Driving for {:.5f} seconds".format(dt))
                    lv, rv = self.pibot.set_velocity([1, 0], tick=tick, time=dt)
                    print("Robot state: ")
                    print(self.ekf.robot.state.squeeze())
                    print("Arrived at [{}, {}]\n".format(waypoint[0], waypoint[1]))
                    self.current_waypoint_idx += 1
                    

                if self.current_waypoint_idx == len(self.path)-2:
                    self.at_goal = True
                    self.run_path = False
                    self.call_calib = True
                    self.command['inference'] = False 
            else:
                self.at_goal = True
                lv, rv, dt = 0, 0, 0
                
            if self.at_goal:
                self.at_goal = False
                self.goal_num += 1
                self.run_path = False
                self.arrived_goal.append(self.path[-1])
                self.notification = 'Arrived at Goal {}'.format(self.goal_num)
                operate.save_image()
                time.sleep(5)
                self.nextgoal = True
                self.call_calib = True

            self.turning = not self.turning
            # except:
            #     self.unknown_obstacles = {}
            #     self.current_waypoint_idx = 1
            #     self.command['inference'] = True
            #     self.start_planning = True
            #     self.run_path = False
            #     self.notification = 'No path found, replanning...'

            #     return 0, 0, 0
        
        return lv, rv, dt
    ##########################################

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
        canvas.blit(caption_surface, (position[0], position[1]-25))

    # keyboard teleoperation        
    def update_keyboard(self):
        for event in pygame.event.get():
            ########### replace with your M1 codes ###########
            # drive forward
            if event.type == pygame.KEYDOWN and event.key == pygame.K_UP:
                self.command['motion'] = [2, 0]
            # drive backward
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_DOWN:
                self.command['motion'] = [-2, 0]
            # turn left
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_LEFT:
                self.command['motion'] = [0, 2]
            # drive right
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_RIGHT:
                self.command['motion'] = [0, -2]
            # stop on key up
            elif event.type == pygame.KEYUP and (event.key in [pygame.K_UP, pygame.K_DOWN, pygame.K_LEFT, pygame.K_RIGHT]):
                self.command['motion'] = [0, 0]
            ####################################################
            # stop
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_SPACE:
                self.command['motion'] = [0, 0]
            # save image
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_i:
                self.command['save_image'] = True
            # save SLAM map
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_s:
                self.command['output'] = True
            # reset SLAM map
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_r:
                if self.double_reset_comfirm == 0:
                    self.notification = 'Press again to confirm CLEAR MAP'
                    self.double_reset_comfirm +=1
                elif self.double_reset_comfirm == 1:
                    self.notification = 'SLAM Map is cleared'
                    self.double_reset_comfirm = 0
                    self.ekf.reset()
            # run SLAM
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_RETURN:
                n_observed_markers = len(self.ekf.taglist)
                if n_observed_markers == 0:
                    if not self.ekf_on:
                        self.notification = 'SLAM is running'
                        self.ekf_on = True
                        self.ekf.load_map()
                    else:
                        self.notification = '> 2 landmarks is required for pausing'
                elif n_observed_markers < 3:
                    self.notification = '> 2 landmarks is required for pausing'
                else:
                    if not self.ekf_on:
                        self.request_recover_robot = True
                    self.ekf_on = not self.ekf_on
                    if self.ekf_on:
                        self.notification = 'SLAM is running'
                    else:
                        self.notification = 'SLAM is paused'
            ## Kelvin Added ##########################
            # run Detector
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_p:
                self.command['inference'] = True
            # save Detector Prediction
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_n:
                self.command['save_inference'] = True
            ##########################################
            # quit
            elif event.type == pygame.QUIT:
                self.quit = True
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                self.quit = True
            ## Kelvin Added ##########################
            # start Path Planning
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_q or self.nextgoal == True:
                if self.arrived_goal == []:
                    self.start_planning = True
                else:
                    self.command['inference'] = True # force it True so it constantly detects fruits
                    self.ekf_on = True
                    self.notification = 'Driving to goal {}'.format(self.goal_num + 1)
                    print('MOVING TO GOAL {} --------------------------------------------------------'.format(self.goal_num + 1))
                    self.nextgoal = False
            ##########################################
        if self.quit:
            pygame.quit()
            sys.exit()

        
if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", metavar='', type=str, default='localhost')
    parser.add_argument("--port", metavar='', type=int, default=40000)
    parser.add_argument("--calib_dir", type=str, default="calibration/param/")
    parser.add_argument("--save_data", action='store_true')
    parser.add_argument("--play_data", action='store_true')
    ## Kelvin Added ##########################
    parser.add_argument("--using_sim", action='store_true')
    parser.add_argument("--map", type=str, default='M4_true_map_5fruits.txt')
    ##########################################
    args, _ = parser.parse_known_args()
    
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

    ## Kelvin Added ##########################
    fruit_name, fruit_pos, aruco_pos = read_true_map(args.map)
    search_list = read_search_list()
    print_target_fruits_pos(search_list, fruit_name, fruit_pos)
    goals = []
    for fruit_for_search in search_list:
        goals.append(fruit_pos[fruit_name.index(fruit_for_search)])
    ##########################################

    start = False

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

    operate = Operate(args, goals, aruco_pos)

    while start:
        operate.update_keyboard()
        operate.take_pic()
        ## Kelvin Added ##########################
        operate.detect_target()
        if operate.start_planning:
            path_planning(operate)
        ##########################################
        drive_meas = operate.control()
        operate.update_slam(drive_meas)
        operate.record_data()
        operate.save_image()

        # visualise
        operate.draw(canvas)
        pygame.display.update()




