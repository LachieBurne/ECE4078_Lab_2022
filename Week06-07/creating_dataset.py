from typing import overload
import cv2
import numpy as np
import os
import random
import string
from collections import namedtuple

class Dataset():
    def __init__(self, fruit_path, bg_path, out_path):
        self.fruit_path = fruit_path
        self.fruit_names = os.listdir(fruit_path)
        self.bg_path = bg_path
        self.bg_names = os.listdir(bg_path)
        self.out_path = out_path
        self.threshold = 0.7

        self.transforms = ['resize', 'aspect', 'blur']

        self.cls_ = {"apple": 0, "pear": 1, "orange": 2, "strawberry": 3, "lemon": 4}

    def import_fruit(self):
        self.fruit = []
        for fname in self.fruit_names:
            self.fruit.append(cv2.imread(os.path.join(self.fruit_path, fname), cv2.IMREAD_UNCHANGED))

    def import_background(self):
        self.backgrounds = []
        for fname in self.bg_names:
            self.backgrounds.append(cv2.imread(os.path.join(self.bg_path, fname), cv2.IMREAD_UNCHANGED))

    def transform_fruit(self):
        temp_list = []
        temp_names = []
        count = 0
        for _ in np.arange(15):
            for transform in self.transforms:
                for i, fruit_img in enumerate(self.fruit):
                    if transform == "resize":
                        resize_factor = random.uniform(0.25, 0.9)
                        temp_list.append(cv2.resize(fruit_img, None, fx=resize_factor, fy=resize_factor,
                                                    interpolation=cv2.INTER_CUBIC))
                        temp_names.append(f"{self.fruit_names[i][:-4].rstrip(string.digits)}{count}.png")
                        count += 1

                    if transform == "aspect":
                        resize_factor1 = random.uniform(0.8, 1.2)
                        resize_factor2 = random.uniform(0.8, 1.2)
                        temp_list.append(cv2.resize(fruit_img, None, fx=resize_factor1, fy=resize_factor2,
                                                    interpolation=cv2.INTER_CUBIC))
                        temp_names.append(f"{self.fruit_names[i][:-4].rstrip(string.digits)}{count}.png")
                        count += 1

                    if transform == "blur":
                        kernel_size = (random.randint(1, 3) * 2) + 1
                        temp_list.append(cv2.GaussianBlur(fruit_img, (kernel_size, kernel_size), 0))
                        temp_names.append(f"{self.fruit_names[i][:-4].rstrip(string.digits)}{count}.png")
                        count += 1

                    # if transform == "brightness":
                    #     value = random.randint(-30, 30)
                    #     temp_list.append(self.change_brightness(fruit_img, value))

        self.fruit = self.fruit + temp_list
        self.fruit_names = self.fruit_names + temp_names

    def save_fruit_dataset(self):
        for i in range(len(self.fruit)):
            # Now outputs not directly to the fruit path
            cv2.imwrite(os.path.join(self.out_path, "fruit_dataset", f"{self.fruit_names[i][:-4].rstrip(string.digits)}{i}.png"), self.fruit[i])

    def paste_backgrounds(self):
        c = list(zip(self.fruit, self.fruit_names))
        random.shuffle(c)
        random.shuffle(c)
        random.shuffle(c)
        self.fruit, self.fruit_names = zip(*c)

        fruit_idx = 0
        bg_idx = 0
        photo_idx = 0
        while fruit_idx < len(self.fruit):
            num_fruit = random.randint(1, 3)
            paste_fruit = self.fruit[fruit_idx:fruit_idx + num_fruit]

            cls = []
            x_center = []
            y_center = []
            width = []
            height = []

            background = self.backgrounds[bg_idx].copy()
            for i, fruit in enumerate(paste_fruit):
                y_offset = random.randint(0, background.shape[0]-fruit.shape[0])
                x_offset = random.randint(0, background.shape[1]-fruit.shape[1])
                
                cls.append(self.cls_[self.fruit_names[fruit_idx+i][:-4].rstrip(string.digits)])
                x_center.append((x_offset + fruit.shape[1] / 2) / background.shape[1])
                y_center.append((y_offset + fruit.shape[0] / 2) / background.shape[0])
                width.append(fruit.shape[1] / background.shape[1])
                height.append(fruit.shape[0] / background.shape[0])

                background = self.overlay_img(background, fruit, y_offset, x_offset)

            if num_fruit == 1:
                with open(os.path.join(self.out_path, "dataset", f"{photo_idx}.txt"), 'w') as f:
                    for i in range(len(cls)):
                        f.write(
                            f"{cls[i]} {x_center[i]} {y_center[i]} {width[i]} {height[i]}\n")  # Need to add the labels here in rows

                cv2.imwrite(os.path.join(self.out_path, "photo", f"{photo_idx}.png"), background)

                fruit_idx += num_fruit  # Will throw index error at end of loop
                if bg_idx == len(self.backgrounds) - 1:
                    bg_idx = 0
                else:
                    bg_idx += 1
                photo_idx += 1

            if num_fruit == 2:
                prev_x, prev_y, prev_width, prev_height = x_center[0], y_center[0], width[0], height[0]
                curr_x, curr_y, curr_width, curr_height = x_center[1], y_center[1], width[1], height[1]

                area = self.area_overlap(prev_x, prev_width, prev_y, prev_height, curr_x, curr_width, curr_y, curr_height)
                if area < self.threshold:
                    with open(os.path.join(self.out_path, "dataset", f"{photo_idx}.txt"), 'w') as f:
                        for i in range(len(cls)):
                            f.write(
                                f"{cls[i]} {x_center[i]} {y_center[i]} {width[i]} {height[i]}\n")  # Need to add the labels here in rows

                    cv2.imwrite(os.path.join(self.out_path, "photo", f"{photo_idx}.png"), background)

                    fruit_idx += num_fruit  # Will throw index error at end of loop
                    if bg_idx == len(self.backgrounds) - 1:
                        bg_idx = 0
                    else:
                        bg_idx += 1
                    photo_idx += 1

            if num_fruit == 3:
                indices = [(0,1), (0,2), (1,2)]
                areas = []

                for pair in indices:
                    prev_x, prev_y, prev_width, prev_height = x_center[pair[0]], y_center[pair[0]], width[pair[0]], height[pair[0]]
                    curr_x, curr_y, curr_width, curr_height = x_center[pair[1]], y_center[pair[1]], width[pair[1]], height[pair[1]]

                    areas.append(self.area_overlap(prev_x, prev_width, prev_y, prev_height, curr_x, curr_width, curr_y, curr_height))

                if all(area < self.threshold for area in areas):
                    with open(os.path.join(self.out_path, "dataset", f"{photo_idx}.txt"), 'w') as f:
                        for i in range(len(cls)):
                            f.write(
                                f"{cls[i]} {x_center[i]} {y_center[i]} {width[i]} {height[i]}\n")  # Need to add the labels here in rows

                    cv2.imwrite(os.path.join(self.out_path, "photo", f"{photo_idx}.png"), background)

                    fruit_idx += num_fruit  # Will throw index error at end of loop
                    if bg_idx == len(self.backgrounds) - 1:
                        bg_idx = 0
                    else:
                        bg_idx += 1
                    photo_idx += 1
            
            # Add code here to save the image as photo_idx.png (variable called background)

            # Add exit condition (end of fruit images)

    def overlay_img(self, l_img, s_img, y_offset, x_offset):
        y1, y2 = y_offset, y_offset + s_img.shape[0]
        x1, x2 = x_offset, x_offset + s_img.shape[1]

        alpha_s = s_img[:, :, 3] / 255.0
        alpha_l = 1.0 - alpha_s

        for c in range(0, 3):
            x = alpha_l * l_img[y1:y2, x1:x2, c]
            l_img[y1:y2, x1:x2, c] = (alpha_s * s_img[:, :, c] +
                                      x)

        return l_img

    def area_overlap(self, prev_x, prev_width, prev_y, prev_height, x, width, y, height):
    
        rec1 = (prev_x - (prev_width/2), prev_y - (prev_height/2), prev_x + (prev_width/2), prev_y + (prev_height/2))
        rec2 = (x - (width/2), y - (height/2), x + (width/2), y + (height/2))

        Rectangle = namedtuple('Rectangle', 'xmin ymin xmax ymax')

        ra = Rectangle(*rec1)
        rb = Rectangle(*rec2)

        dx = min(ra.xmax, rb.xmax) - max(ra.xmin, rb.xmin)
        dy = min(ra.ymax, rb.ymax) - max(ra.ymin, rb.ymin)
        if (dx>=0) and (dy>=0):
            area = dx*dy
        else:
            area = 0

        rec1_area = prev_width*prev_height
        rec2_area = width*height
        shape_areas = [rec1_area, rec2_area]
        shape_areas.sort()
        area /= shape_areas[0]

        return area

    # def change_brightness(self, img, value=30):
    #     hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    #     h, s, v = cv2.split(hsv)

    #     upper_lim = 255 - value
    #     lower_lim = value

    #     value = np.uint8(value)

    #     if value > 0:
    #         v[v > upper_lim] = 255
    #         v[v <= upper_lim] += value
    #     else:
    #         v[v > lower_lim] += value
    #         v[v <= lower_lim] = 0

    #     final_hsv = cv2.merge((h, s, v))
    #     img = cv2.cvtColor(final_hsv, cv2.COLOR_HSV2BGR)
    #     return img


dataset = Dataset(r"Week06-07\data\fruits",
                  r"Week06-07\data\backgrounds",
                  r"Week06-07\data\final")
dataset.import_fruit()
dataset.import_background()
dataset.transform_fruit()
# dataset.save_fruit_dataset()
dataset.paste_backgrounds()
# print(len(dataset.fruit))
# idx = random.randint(51, len(dataset.fruit))
# print(dataset.fruit[idx])
