# measure performance of target detection and pose estimation
import ast
import numpy as np
import json

# read in the object poses
def parse_map(fname: str) -> dict:
    with open(fname,'r') as f:
        gt_dict = ast.literal_eval(f.readline())        
        apple_gt, lemon_gt, pear_gt, orange_gt, strawberry_gt = [], [], [], [], []

        # remove unique id of targets of the same type 
        for key in gt_dict:
            if key.startswith('apple'):
                apple_gt.append(np.array(list(gt_dict[key].values()), dtype=float))
            elif key.startswith('lemon'):
                lemon_gt.append(np.array(list(gt_dict[key].values()), dtype=float))
            elif key.startswith('pear'):
                pear_gt.append(np.array(list(gt_dict[key].values()), dtype=float))
            elif key.startswith('orange'):
                orange_gt.append(np.array(list(gt_dict[key].values()), dtype=float))
            elif key.startswith('strawberry'):
                strawberry_gt.append(np.array(list(gt_dict[key].values()), dtype=float))
    # if more than 3 estimations are given for a target type, only the first 3 estimations will be used
    if len(apple_gt) > 2:
        apple_gt = apple_gt[0:2]
    if len(lemon_gt) > 2:
        lemon_gt = lemon_gt[0:2]
    if len(pear_gt) > 2:
        pear_gt = pear_gt[0:2]
    if len(orange_gt) > 2:
        orange_gt = orange_gt[0:2]
    if len(strawberry_gt) > 2:
        strawberry_gt = strawberry_gt[0:2]

    return apple_gt, lemon_gt, pear_gt, orange_gt, strawberry_gt


# compute the Euclidean distance between each target and its closest estimation and returns the average over all targets
def compute_dist(gt_list, est_list):
    gt_list = gt_list
    est_list = est_list
    dist_av = 0
    dist_list = []
    dist = []
    for gt in gt_list:
        # find the closest estimation for each target
        for est in est_list:
            dist.append(np.linalg.norm(gt-est)) # compute Euclidean distance
        dist.sort()
        dist_list.append(dist[0]) # distance between the target and its closest estimation
        dist = []
    dist_av = sum(dist_list)/len(dist_list) # average distance
    return dist_av

# main program
if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser("Matching the estimated map and the true map")
    parser.add_argument("--truth", type=str, default='M3_marking_map.txt')
    parser.add_argument("--est", type=str, default='lab_output/targets.txt')
    args, _ = parser.parse_known_args()

    # read in ground truth and estimations
    apple_gt, lemon_gt, pear_gt, orange_gt, strawberry_gt = parse_map(args.truth)
    apple_est, lemon_est, pear_est, orange_est, strawberry_est = parse_map(args.est)
    
    # compute average distance between a target and its closest estimation
    apple_dist = compute_dist(apple_gt,apple_est)
    lemon_dist = compute_dist(lemon_gt,lemon_est)
    pear_dist = compute_dist(pear_gt, pear_est)
    orange_dist = compute_dist(orange_gt, orange_est)
    strawberry_dist = compute_dist(strawberry_gt, strawberry_est)
    
    av_dist = (apple_dist+lemon_dist+pear_dist+orange_dist+strawberry_dist)/5
    
    print("Average distances between the targets and the closest estimations:")
    print("apple = {}, lemon = {}, pear = {}, orange = {}, strawberry = {}".format(apple_dist,lemon_dist,orange_dist,pear_dist,strawberry_dist))
    print("estimation error: ", av_dist)

