import time
import util.measure as measure
import numpy as np
import pygame


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

def calculate_drive_time(distance_to_waypoint, scale, tick=20):
    return (1.0 / (scale * tick)) * distance_to_waypoint # replace with your calculation

def calculate_distance_to_waypoint(robot_pose, waypoint):
        x_diff = waypoint[0] - robot_pose[0]
        y_diff = waypoint[1] - robot_pose[1]
        x_diff, y_diff = x_diff, y_diff
        distance_to_waypoint = np.hypot(x_diff, y_diff)
        print(f"[calculate_distance_to_waypoint][distance_to_waypoint] {distance_to_waypoint}")
        return distance_to_waypoint

def partly_turn(operate, turn_time, ip, resolution=2):
    done = False
    then = time.time()
    while not done:
        robot_pose = get_robot_pose(operate)
        termination_condition = time.time() - then > turn_time/resolution*2 if ip == "localhost" else time.time() - then > turn_time/resolution
        if termination_condition:
            done = True
            lv, rv = operate.pibot.set_velocity(operate.command['motion'])
            drive_meas = measure.Drive(lv, rv, dt=turn_time/resolution)
            operate.update_slam(drive_meas)
    return robot_pose

def partly_drive(operate, drive_time, ip, resolution=2):
    done = False
    then = time.time()
    while not done:
        robot_pose = get_robot_pose(operate)
        termination_condition = time.time() - then > drive_time/resolution*2 if ip == "localhost" else time.time() - then > drive_time/resolution
        if termination_condition:
            done = True
            lv, rv = operate.pibot.set_velocity(operate.command['motion'])
            drive_meas = measure.Drive(lv, rv, dt=drive_time/resolution)
            operate.update_slam(drive_meas)
    return robot_pose

def drive_to_point(operate, scale, baseline, waypoint):
    print(f"Driving to waypoint...")
    done = False
    dt = calculate_drive_time(0.1, scale) # if SIM else calculate_drive_time(0.09, scale)
    operate.command['motion'] = [1,0]
    while not done:
        operate.command['motion'] = [1,0]
        robot_pose = get_robot_pose(operate, dt)
        operate.pibot.set_velocity([0,0])
        distance_to_waypoint = calculate_distance_to_waypoint(robot_pose, waypoint)
        turning_angle = calculate_turning_angle(robot_pose, waypoint)
        if distance_to_waypoint < 0.1:
            done = True
        elif abs(turning_angle) > np.pi/180*10:
            sign = 1 if turning_angle > 0 else -1
            operate.command['motion'] = [0, 2*sign] if turning_angle > 0 else [0, 0]
            robot_pose = turn_to_point(operate, waypoint, baseline, scale, sign, turning_angle)
    operate.command['motion'] = [0,0]
    print(f"Done...")
    return robot_pose

def turn_to_point(operate, waypoint, baseline, scale, sign, turning_angle):
        print(f"Turning to waypoint...")
        done = False
        robot_pose = operate.ekf.robot.state.flatten()
        dt = calculate_turn_time(np.pi/180*5, baseline, scale)
        while not done:
            operate.command['motion'] = [0, 2*sign] if abs(turning_angle) > 0 else [0, 0]
            robot_pose = get_robot_pose(operate, dt)
            operate.pibot.set_velocity([0,0])
            turning_angle = calculate_turning_angle(robot_pose, waypoint)/2
            sign = 1 if turning_angle > 0 else -1
            if abs(turning_angle) < np.pi/180*5:
                done = True
        operate.command['motion'] = [0,0]
        print(f"Done...")
        return robot_pose

def move_to_waypoint(waypoint, robot_pose, operate, ip):
    # imports camera / wheel calibration parameters 
    if ip=="localhost":
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

    then = time.time()
    condition = 1*2 if ip=="localhost" else 1
    while time.time() - then < condition:
        robot_pose = get_robot_pose(operate)

    turning_angle  = calculate_turning_angle(robot_pose, waypoint)
    sign = 1 if turning_angle > 0 else -1
    operate.command['motion'] = [0, 2*sign] if turning_angle > 0 else [0, 0]
    
    robot_pose = turn_to_point(operate, waypoint, baseline, scale, sign, turning_angle)
    robot_pose = drive_to_point(operate, scale, baseline, waypoint)

    ####################################################

    print("Arrived at [{}, {}]".format(waypoint[0], waypoint[1])) 
    print(f"Translated pose: {robot_pose}") 
    return robot_pose


def get_robot_pose(operate, dt=None):
    ####################################################
    for event in pygame.event.get():
        if event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
            operate.command['motion'] = [0, 0]
            operate.pibot.set_velocity(operate.command['motion'])
    
    for i in range(5):
        operate.take_pic()
    drive_meas = operate.control(dt)
    operate.update_slam(drive_meas)
    operate.markers, tag_list = operate.update_markers(operate.ekf.taglist, operate.aruco_true_pos, operate.tags)
    operate.ekf.taglist = tag_list
    operate.ekf.markers = operate.markers
    operate.record_data()
    operate.save_image()
    # operate.detect_target()
    # visualise
    operate.draw(operate.canvas)
    pygame.display.update()

    # update the robot pose [x,y,theta]

    robot_pose = operate.ekf.robot.state.flatten() # replace with your calculation
    ####################################################

    return robot_pose