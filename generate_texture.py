import cv2
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image
import json
import click

scale = 1000
width_m = 4
height_m = 4

height = height_m * scale
width = width_m * scale # 1 pixel = 1mm

color = (0,0,0)
radius = 500
thickness = 2

def to_img_coor(img_width, img_height, scale, x, y):
    return (int(x * scale + img_width/2),
            int(img_height/2 - y*scale))

def main_loop(args):
    with open(args.map) as f:
        lm_data = json.load(f)
    with open(args.search_list) as f:
        lines = f.readlines()
    search_fruits = [line.rstrip() for line in lines]
    processed_img = np.ones((height, width, 3), np.uint8) * 145
    for fruit in search_fruits:
        current_fruit = lm_data[fruit + "_0"]
        center_coordinates = to_img_coor(width, height, scale,
                                        current_fruit['x'], current_fruit['y'])
        processed_img = cv2.circle(processed_img, center_coordinates, radius, color, thickness)
    
    im = Image.fromarray(processed_img)
    im.save("texture_ECE4078.jpeg")

if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser()
    parser.add_argument('--map', type=str, default='true_map.txt')
    parser.add_argument('--search_list', type=str, default='slam.txt')
    args, _ = parser.parse_known_args()
    main_loop(args)
