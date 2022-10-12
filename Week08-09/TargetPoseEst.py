# estimate the pose of a target object detected
import numpy as np
import json
import os
from pathlib import Path
import ast
from os import listdir

import torch
from sklearn.cluster import KMeans

def get_YOLO_stuff(fruit_select, using_sim):

    fruit_xmin = fruit_select[0]
    fruit_ymin = fruit_select[1]
    fruit_xmax = fruit_select[2]
    fruit_ymax = fruit_select[3]
    fruit_conf = fruit_select[4]
    fruit_class = fruit_select[5]
    fruit_name = fruit_select[6]

    u_0 = 640/2 if using_sim else 320/2
    v_0 = 480/2 if using_sim else 240/2
   
    fruit_xcent = (fruit_xmin + fruit_xmax)/2 - u_0
    fruit_ycent = (fruit_ymin + fruit_ymax)/2 - v_0
    fruit_width = fruit_xmax - fruit_xmin
    fruit_height = fruit_ymax - fruit_ymin

    class_converter = {0:1,1:3,2:4,3:5,4:2}

    return (class_converter[fruit_class], [fruit_xcent,fruit_ycent, fruit_width, fruit_height])

def unpack_image(image, using_sim):
    """
    :param: image The file path to the image
    :return: A list of tuples in the form of (fruit_class, bounding_box)
    """

    #confidence threshold.
    if using_sim:
        model.conf = 0.2
    else:
        model.conf = 0.65

    result = model(image)
    image_data = result.pandas().xyxy[0]
    image_data_list = image_data.values.tolist()

    yolo_stuff = []

    for fruit_data in image_data_list:
        yolo_stuff.append(get_YOLO_stuff(fruit_data, using_sim))

    return yolo_stuff


# # use the machinevision toolbox to get the bounding box of the detected target(s) in an image
# def get_bounding_box(target_number, image_path):
#     image = PIL.Image.open(image_path).resize((640,480), PIL.Image.NEAREST)
#     target = Image(image)==target_number
#     blobs = target.blobs()
#     [[u1,u2],[v1,v2]] = blobs[0].bbox # bounding box
#     width = abs(u1-u2)
#     height = abs(v1-v2)
#     center = np.array(blobs[0].centroid).reshape(2,)
#     box = [center[0], center[1], int(width), int(height)] # box=[x,y,width,height]
#     # plt.imshow(fruit.image)
#     # plt.annotate(str(fruit_number), np.array(blobs[0].centroid).reshape(2,))
#     # plt.show()
#     # assert len(blobs) == 1, "An image should contain only one object of each target type"
#     return box


# read in the list of detection results with bounding boxes and their matching robot pose info
def get_image_info(img_file_path, image_poses, using_sim):
    """
    Output in the form of 
    """
    # there are at most five types of targets in each image
    target_lst_box = [[], [], [], [], []]
    target_lst_pose = [[], [], [], [], []]
    completed_img_dict = {}

    # add the bounding box info of each target in each image
    # target labels: 1 = apple, 2 = lemon, 3 = pear, 4 = orange, 5 = strawberry, 0 = not_a_target
    
    all_vals = unpack_image(img_file_path, using_sim)
    for (target_num, box) in all_vals:
        pose = image_poses[file_path] # [x, y, theta] 
        target_lst_box[target_num-1].append(box) # bounding box of target
        target_lst_pose[target_num-1].append(np.array(pose).reshape(3,)) # robot pose

    # if there are more than one objects of the same type, combine them
    for i in range(5):
        if len(target_lst_box[i])>0:
            box = np.stack(target_lst_box[i], axis=1)
            pose = np.stack(target_lst_pose[i], axis=1)
            completed_img_dict[i+1] = {'target': box, 'robot': pose}
        
    return completed_img_dict

# estimate the pose of a target based on size and location of its bounding box in the robot's camera view and the robot's pose
def estimate_pose(camera_matrix, completed_img_dict, using_sim):
    camera_matrix = camera_matrix
    focal_length = camera_matrix[0][0]
    # actual sizes of targets [For the simulation models]
    # You need to replace these values for the real world objects
    target_dimensions = []

    if using_sim:
        apple_dimensions = [0.075448, 0.074871, 0.071889]
        target_dimensions.append(apple_dimensions)
        lemon_dimensions = [0.060588, 0.059299, 0.053017]
        target_dimensions.append(lemon_dimensions)
        pear_dimensions = [0.0946, 0.0948, 0.135]
        target_dimensions.append(pear_dimensions)
        orange_dimensions = [0.0721, 0.0771, 0.0739]
        target_dimensions.append(orange_dimensions)
        strawberry_dimensions = [0.052, 0.0346, 0.0376]
        target_dimensions.append(strawberry_dimensions)


    else:  # width length height
        apple_dimensions = [0.078, 0.078, 0.070]
        target_dimensions.append(apple_dimensions)
        lemon_dimensions = [0.0781, 0.0535, 0.050]
        target_dimensions.append(lemon_dimensions)
        pear_dimensions = [0.073, 0.080, 0.091]
        target_dimensions.append(pear_dimensions)
        orange_dimensions = [0.075, 0.075, 0.075]
        target_dimensions.append(orange_dimensions)
        strawberry_dimensions = [0.0412, 0.0410, 0.0382]
        target_dimensions.append(strawberry_dimensions)

    target_list = ['apple', 'lemon', 'pear', 'orange', 'strawberry']

    target_pose_dict = {}
    # for each target in each detection output, estimate its pose
    for target_num in completed_img_dict.keys():
        box = completed_img_dict[target_num]['target'] # [[x],[y],[width],[height]]
        robot_pose = completed_img_dict[target_num]['robot'] # [[x], [y], [theta]]
        true_height = target_dimensions[target_num-1][2]
        
        ######### Replace with your codes #########
        box = [x[0] for x in box]
        robot_pose = [x[0] for x in robot_pose]
        x_box, y_box, width_box, height_box = box
        x_robot, y_robot, theta_robot = robot_pose

        depth = (focal_length/height_box) * true_height
        z_fruit = depth
        x_fruit = (x_box/focal_length) * z_fruit
        alpha = np.arctan(x_fruit/z_fruit)
        dist_fruit = np.hypot(x_fruit,z_fruit)

        phi = theta_robot - alpha
        x_loc = x_robot + dist_fruit * np.cos(phi)
        y_loc = y_robot + dist_fruit * np.sin(phi)

        target_pose = {'y': y_loc, 'x': x_loc}
        
        # {apple:{y:YPOS,x:XPOS}}
        target_pose_dict[target_list[target_num-1]] = target_pose
        ###########################################
    
    return target_pose_dict

def merge_to_mean(position_est, remove_outlier = False):

    # Inputs:
    # position_est : An numpy array of coordinates {position_est[estimation #][0 = x, 1 = y]}
    # remove_outlier : Boolean (Remove outliers using Standard Distribution z-scores)
    # Outputs:
    # new_mean : An numpy array of coordinates {new_mean[0 = x, 1 = y]}

    # Check if the position_est has no elements
    if len(position_est) == 0:
        return None

    # Set up working parameters
    position_est_result = []
    z_threshold = 3

    # Compute mean and standard deviations
    means = np.mean(position_est, axis = 0)
    stds = np.std(position_est, axis = 0)
    mean_x = means[0]
    std_x = stds[0]
    mean_y = means[1]
    std_y = stds[1]
    
    # Remove outliers
    if remove_outlier:
        for i in range(len(position_est)):
            coordinates = position_est[i]
            z_score_x = (coordinates[0] - mean_x)/std_x
            z_score_y = (coordinates[1] - mean_y)/std_y
            if np.abs(z_score_x) > z_threshold or np.abs(z_score_y) > z_threshold:
                position_est_result.append(coordinates)
    else:
        position_est_result = position_est

    # Compute Mean
    new_mean = np.mean(position_est_result, axis = 0)

    return new_mean


# merge the estimations of the targets so that there are at most 3 estimations of each target type
def merge_estimations(target_pose_dict):
    target_map = target_pose_dict
    apple_est, lemon_est, pear_est, orange_est, strawberry_est = [], [], [], [], []
    target_est = {}
    
    # combine the estimations from multiple detector outputs
    for f in target_map:
        for key in target_map[f]:
            if key.startswith('apple'):
                apple_est.append(np.array(list(target_map[f][key].values()), dtype=float))
            elif key.startswith('lemon'):
                lemon_est.append(np.array(list(target_map[f][key].values()), dtype=float))
            elif key.startswith('pear'):
                pear_est.append(np.array(list(target_map[f][key].values()), dtype=float))
            elif key.startswith('orange'):
                orange_est.append(np.array(list(target_map[f][key].values()), dtype=float))
            elif key.startswith('strawberry'):
                strawberry_est.append(np.array(list(target_map[f][key].values()), dtype=float))

    ######### Replace with your codes #########
    # TODO: the operation below takes the first three estimations of each target type, replace it with a better merge solution
    remove_outlier = False
    use_Kmeans = False
    if len(apple_est) > 1:
        apple_est = merge_to_mean(apple_est, remove_outlier = False)
    if len(lemon_est) > 1:
        lemon_est = merge_to_mean(lemon_est, remove_outlier = False)
    if len(pear_est) > 1:
        pear_est = merge_to_mean(pear_est, remove_outlier = False)
    if len(orange_est) > 1:
        orange_est = merge_to_mean(orange_est, remove_outlier = False)
    if len(strawberry_est) > 1:
        strawberry_est = merge_to_mean(strawberry_est, remove_outlier = False)

    for i in range(2):
        try:
            target_est['apple_'+str(i)] = {'y':apple_est[i][0], 'x':apple_est[i][1]}
        except:
            pass
        try:
            target_est['lemon_'+str(i)] = {'y':lemon_est[i][0], 'x':lemon_est[i][1]}
        except:
            pass
        try:
            target_est['pear_'+str(i)] = {'y':pear_est[i][0], 'x':pear_est[i][1]}
        except:
            pass
        try:
            target_est['orange_'+str(i)] = {'y':orange_est[i][0], 'x':orange_est[i][1]}
        except:
            pass
        try:
            target_est['strawberry_'+str(i)] = {'y':strawberry_est[i][0], 'x':strawberry_est[i][1]}
        except:
            pass
    ###########################################
        
    return target_est


if __name__ == "__main__":

    weights_filename = 'best_sim.pt' if USING_SIM else 'best_real.pt'
    weight_path = f'final_weights/{weights_filename}'

    #Make sure the weight file corresponds to the test image.

    model = torch.hub.load('ultralytics/yolov5', 'custom', path=weight_path, force_reload=True)
    #Downloads from github once. We can make it do this locally if necessary.
    

    # camera_matrix = np.ones((3,3))/2
    intrinsic_filename = 'intrinsic_sim.txt' if USING_SIM else 'intrinsic.txt'
    calibration_path = 'calibration/param/'
    fileK = calibration_path + intrinsic_filename   

    camera_matrix = np.loadtxt(fileK, delimiter=',')
    base_dir = Path('./')
    
    
    # a dictionary of all the saved detector outputs
    image_poses = {}
    with open(base_dir/'lab_output/images.txt') as fp:
        for line in fp.readlines():
            pose_dict = ast.literal_eval(line)
            image_poses[pose_dict['imgfname']] = pose_dict['pose']
    
    # estimate pose of targets in each detector output
    # {image_0:{apple:{y:YPOS,x:XPOS}}}
    target_map = {}        
    for file_path in image_poses.keys():
        completed_img_dict = get_image_info(file_path, image_poses, USING_SIM)
        target_map[file_path] = estimate_pose(camera_matrix, completed_img_dict, USING_SIM)

    # merge the estimations of the targets so that there are at most 3 estimations of each target type
    target_est = merge_estimations(target_map)
                     
    # save target pose estimations
    with open(base_dir/'lab_output/targets.txt', 'w') as fo:
        json.dump(target_est, fo)
    
    print('Estimations saved :)')