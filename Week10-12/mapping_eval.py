import json
import numpy as np
from copy import deepcopy


def parse_true_map(fname):
    with open(fname, 'r') as fd:
        gt_dict = json.load(fd)
        aruco_dict = {}
        apples, lemons, pears, oranges, strawberries = [], [], [], [], []

        for key in gt_dict:
            if key.startswith('aruco'):
                aruco_num = int(key.strip('aruco')[:-2])
                aruco_dict[aruco_num] = np.reshape([gt_dict[key]['x'], gt_dict[key]['y']], (2, 1))
            elif key.startswith('apple'):
                apples.append(np.array([gt_dict[key]['x'], gt_dict[key]['y']], dtype=float))
            elif key.startswith('lemon'):
                lemons.append(np.array([gt_dict[key]['x'], gt_dict[key]['y']], dtype=float))
            elif key.startswith('pear'):
                pears.append(np.array([gt_dict[key]['x'], gt_dict[key]['y']], dtype=float))
            elif key.startswith('orange'):
                oranges.append(np.array([gt_dict[key]['x'], gt_dict[key]['y']], dtype=float))
            elif key.startswith('strawberry'):
                strawberries.append(np.array([gt_dict[key]['x'], gt_dict[key]['y']], dtype=float))

    fruits = {
        'apples': apples,
        'lemons': lemons,
        'pears': pears,
        'oranges': oranges,
        'strawberries': strawberries
    }
    return aruco_dict, fruits


def parse_fruit_map(fname):
    with open(fname, 'r') as fd:
        gt_dict = json.load(fd)
        apples, lemons, pears, oranges, strawberries = [], [], [], [], []

        for key in gt_dict:
            if key.startswith('apple'):
                apples.append(np.array([gt_dict[key]['x'], gt_dict[key]['y']], dtype=float))
            elif key.startswith('lemon'):
                lemons.append(np.array([gt_dict[key]['x'], gt_dict[key]['y']], dtype=float))
            elif key.startswith('pear'):
                pears.append(np.array([gt_dict[key]['x'], gt_dict[key]['y']], dtype=float))
            elif key.startswith('orange'):
                oranges.append(np.array([gt_dict[key]['x'], gt_dict[key]['y']], dtype=float))
            elif key.startswith('strawberry'):
                strawberries.append(np.array([gt_dict[key]['x'], gt_dict[key]['y']], dtype=float))

    fruits = {
        'apples': apples,
        'lemons': lemons,
        'pears': pears,
        'oranges': oranges,
        'strawberries': strawberries
    }
    return fruits


def parse_slam_map(fname: str) -> dict:
    with open(fname, 'r') as fd:
        usr_dict = json.load(fd)
        aruco_dict = {}
        for (i, tag) in enumerate(usr_dict['taglist']):
            aruco_dict[tag] = np.reshape([usr_dict['map'][0][i], usr_dict['map'][1][i]], (2, 1))
    return aruco_dict


def match_aruco_points(aruco0: dict, aruco1: dict):
    points0 = []
    points1 = []
    keys = []
    for key in aruco0:
        if not key in aruco1:
            continue

        points0.append(aruco0[key])
        points1.append(aruco1[key])
        keys.append(key)
    return keys, np.hstack(points0), np.hstack(points1)


def solve_umeyama2d(points1, points2):
    # Solve the optimal transform such that
    # R(theta) * p1_i + t = p2_i

    assert (points1.shape[0] == 2)
    assert (points1.shape[0] == points2.shape[0])
    assert (points1.shape[1] == points2.shape[1])

    # Compute relevant variables
    num_points = points1.shape[1]
    mu1 = 1 / num_points * np.reshape(np.sum(points1, axis=1), (2, -1))
    mu2 = 1 / num_points * np.reshape(np.sum(points2, axis=1), (2, -1))
    sig1sq = 1 / num_points * np.sum((points1 - mu1) ** 2.0)
    sig2sq = 1 / num_points * np.sum((points2 - mu2) ** 2.0)
    Sig12 = 1 / num_points * (points2 - mu2) @ (points1 - mu1).T

    # Use the SVD for the rotation
    U, d, Vh = np.linalg.svd(Sig12)
    S = np.eye(2)
    if np.linalg.det(Sig12) < 0:
        S[-1, -1] = -1

    # Return the result as an angle and a 2x1 vector
    R = U @ S @ Vh
    theta = np.arctan2(R[1, 0], R[0, 0])
    x = mu2 - R @ mu1

    return theta, x


def apply_transform(theta, x, points):
    # Apply an SE(2) transform to a set of 2D points
    assert (points.shape[0] == 2)

    c, s = np.cos(theta), np.sin(theta)
    R = np.array(((c, -s), (s, c)))

    points_transformed = R @ points + x
    return points_transformed


def compute_slam_rmse(points1, points2):
    # Compute the RMSE between two matched sets of 2D points.
    assert (points1.shape[0] == 2)
    assert (points1.shape[0] == points2.shape[0])
    assert (points1.shape[1] == points2.shape[1])
    num_points = points1.shape[1]
    residual = (points1 - points2).ravel()
    MSE = 1.0 / num_points * np.sum(residual ** 2)

    return np.sqrt(MSE)


def compute_fruit_est_error(gt_list, est_list):
    """Compute the fruit target pose estimation error based on Euclidean distance

    If there are more estimations than the number of targets (e.g. only 1 target apple, but detected 2),
        then take the average error of the 2 detections

    if there are fewer estimations than the number of targets (e.g. 2 target apples, but only detected 1),
        then return [MAX_ERROR, error with the closest target]

    @param gt_list: target ground truth list
    @param est_list: estimation list
    @return: error of all the fruits
    """

    MAX_ERROR = 1

    fruit_errors = {}

    for fruit_type in gt_list:
        n_gt = len(gt_list[fruit_type])  # number of targets in this fruit type

        type_errors = []
        for i, gt in enumerate(gt_list[fruit_type]):
            dist = []
            for est in est_list[fruit_type]:
                dist.append(np.linalg.norm(gt - est))  # compute Euclidean distance

            n_est = len(est_list[fruit_type])

            # if this fruit type has been detected
            if len(dist) > 0:
                if n_est > n_gt:    # if more estimation than target, take the mean error
                    fruit_errors[fruit_type + '_{}'.format(i)] = np.round(np.mean(dist), 3)
                elif n_est < n_gt:  # see below
                    type_errors.append(np.min(dist))
                else:   # for normal cases, n_est == n_gt, take the min error
                    fruit_errors[fruit_type + '_{}'.format(i)] = np.round(np.min(dist), 3)
            else:   # if there is no estimation for this fruit type
                for j in range(n_gt):
                    fruit_errors[fruit_type + '_{}'.format(j)] = MAX_ERROR

        if len(type_errors) > 0:    # for the n_est < n_gt scenario
            type_errors = np.sort(type_errors)
            for i in range(len(type_errors) - 1):
                fruit_errors[fruit_type + '_{}'.format(i+1)] = np.round(type_errors[i], 3)
            fruit_errors[fruit_type + '_{}'.format(0)] = MAX_ERROR

    return fruit_errors


def align_fruit_poses(theta, x, fruits_est):
    fruits = deepcopy(fruits_est)

    for fruit in fruits:
        poses = []
        for pos in fruits[fruit]:
            pos = np.reshape(pos, (2, 1))
            pos = apply_transform(theta, x, pos)
            pos = np.reshape(pos, (1, 2))[0]

            poses.append(pos)

        fruits[fruit] = poses

    return fruits


if __name__ == '__main__':
    import argparse
    
    parser = argparse.ArgumentParser('Matching the estimated map and the true map')
    parser.add_argument('--true-map', type=str, default='true_map.txt')
    parser.add_argument('--slam-est', type=str, default='slam.txt')
    parser.add_argument('--target-est', type=str, default='targets.txt')
    parser.add_argument('--slam-only', action='store_true')
    parser.add_argument('--target-only', action='store_true')
    args, _ = parser.parse_known_args()

    slam_only = args.slam_only      # only evaluate slam.txt
    target_only = args.target_only  # only evaluate targets.txt

    if slam_only and target_only:
        print('You cannot choose --slam-only and --target-only at the same time!')
        exit()

    aruco_gt, fruits_gt = parse_true_map(args.true_map)

    slam_rmse = 99

    if slam_only:
        # only evaluate SLAM
        aruco_est = parse_slam_map(args.slam_est)
        taglist, slam_est_vec, slam_gt_vec = match_aruco_points(aruco_est, aruco_gt)
        theta, x = solve_umeyama2d(slam_est_vec, slam_gt_vec)
        slam_est_vec_aligned = apply_transform(theta, x, slam_est_vec)

        slam_rmse = compute_slam_rmse(slam_est_vec_aligned, slam_gt_vec)

        print(f'The SLAM RMSE = {np.round(slam_rmse, 3)}')

    elif target_only:
        fruits_est = parse_fruit_map(args.target_est)
        fruit_est_errors = compute_fruit_est_error(fruits_gt, fruits_est)
        print('Fruit pose estimation errors (without alignment):')
        print(json.dumps(fruit_est_errors, indent=4))
    else:
        # evaluate SLAM
        aruco_est = parse_slam_map(args.slam_est)
        taglist, slam_est_vec, slam_gt_vec = match_aruco_points(aruco_est, aruco_gt)
        theta, x = solve_umeyama2d(slam_est_vec, slam_gt_vec)
        slam_est_vec_aligned = apply_transform(theta, x, slam_est_vec)

        slam_rmse_raw = compute_slam_rmse(slam_est_vec, slam_gt_vec)
        slam_rmse_aligned = compute_slam_rmse(slam_est_vec_aligned, slam_gt_vec)

        print(f'The SLAM RMSE before alignment = {np.round(slam_rmse_raw, 3)}')
        print(f'The SLAM RMSE after alignment = {np.round(slam_rmse_aligned, 3)}')

        print('----------------------------------------------')
        # evaluate fruit pose estimation errors
        fruits_est = parse_fruit_map(args.target_est)

        # align the fruit poses using the transform computed from SLAM
        fruits_est_aligned = align_fruit_poses(theta, x, fruits_est)

        fruit_est_errors_raw = compute_fruit_est_error(fruits_gt, fruits_est)
        fruit_est_errors_aligned = compute_fruit_est_error(fruits_gt, fruits_est_aligned)

        print('Fruit pose estimation errors before alignment:')
        print(json.dumps(fruit_est_errors_raw, indent=4))
        print('Fruit pose estimation errors after alignment:')
        print(json.dumps(fruit_est_errors_aligned, indent=4))

        avg_err = []
        for fruit_err in fruit_est_errors_aligned:
            avg_err.append(fruit_est_errors_aligned[fruit_err])

        print(f'Average fruit pose estimation error after alignment: {np.mean(avg_err)}')





