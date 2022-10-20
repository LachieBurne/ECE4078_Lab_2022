import json
import ast
import numpy as np


def parse_user_map(fname: str) -> dict:
    with open(fname, 'r') as f:
        usr_dict = ast.literal_eval(f.read())
        aruco_dict = {}
        for (i, tag) in enumerate(usr_dict["taglist"]):
            aruco_dict[tag] = np.reshape([usr_dict["map"][0][i], usr_dict["map"][1][i]], (2, 1))
    return aruco_dict


def convert_slam(fname):
    final_dict = {}
    slam_dict = parse_user_map(fname)
    for aruco_num in slam_dict.keys():
        x = slam_dict[aruco_num][0][0]
        y = slam_dict[aruco_num][1][0]
        position = {"x": x, "y": y}
        final_dict[f'aruco{aruco_num}_0'] = position
    return final_dict


def combine_everything(slam_fname, fruit_fname):
    f = open(fruit_fname, 'r')
    fruits = json.load(f)
    slam = convert_slam(slam_fname)
    slam.update(fruits)
    with open('../Week01-02/util/output_map.txt', 'w') as f:
        json.dump(slam, f)


if __name__ == '__main__':
    combine_everything('slam6_best.txt', 'targets.txt')
