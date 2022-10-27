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
        if aruco_num in range(1,11):
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
    with open('output_map.txt', 'w') as f:
        json.dump(slam, f)


if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--slam_map", type=str, default="lab_output/targets.txt")
    parser.add_argument("--fruit_map", type=str, default="lab_output/slam.txt")
    args, _ = parser.parse_known_args()

    combine_everything(args.slam_map, args.fruit_map)
