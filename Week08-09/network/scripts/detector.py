import os 
import time

import cmd_printer
import numpy as np
import torch
from args import args
from res18_skip import Resnet18Skip
from torchvision import transforms
import cv2

class Detector:
    def __init__(self, ckpt, use_gpu=False):
        self.args = args
        ## Kelvin Added ########################## 
        self.model_YOLO = torch.hub.load('ultralytics/yolov5', 'custom', path=ckpt, force_reload=True)
        self.model = Resnet18Skip(args)
        ##########################################
        if torch.cuda.torch.cuda.device_count() > 0 and use_gpu:
            self.use_gpu = True
            self.model = self.model.cuda()
        else:
            self.use_gpu = False
        self.load_weights(ckpt)
        self.model = self.model.eval()
        cmd_printer.divider(text="warning")
        print('This detector uses "RGB" input convention by default')
        print('If you are using Opencv, the image is likely to be in "BRG"!!!')
        cmd_printer.divider()
        self.colour_code = np.array([(220, 220, 220), (128, 0, 0), (128, 128, 0), (0, 128, 0), (192, 68, 0), (0, 0, 255)])
        # color of background, apple, lemon, pear, orange, strawberry

    def detect_single_image(self, np_img, image_pose):

        ## Kelvin Commented Out ########################## 
        # NOT USING RESNET
        # torch_img = self.np_img2torch(np_img)
        # tick = time.time()
        # with torch.no_grad():
        #     pred = self.model.forward(torch_img)
        #     if self.use_gpu:
        #         pred = torch.argmax(pred.squeeze(),
        #                             dim=0).detach().cpu().numpy()
        #     else:
        #         pred = torch.argmax(pred.squeeze(), dim=0).detach().numpy()
        # dt = time.time() - tick
        # # print(f'Inference Time {dt:.2f}s, approx {1/dt:.2f}fps', end="\r")
        # colour_map = self.visualise_output(pred)
        ##########################################

        ## Kelvin Added ########################## 
        # Predict using YOLO
        if args.using_sim:
            self.model_YOLO.conf = 0.2
        else:
            self.model_YOLO.conf = 0.65

        results = self.model_YOLO(np_img)
        YOLO_pred = results.pandas().xyxy[0]
        YOLO_pred = YOLO_pred.to_numpy()   

        class_converter = {0:1,1:3,2:4,3:5,4:2}

        # there are at most five types of targets in each image
        target_lst_box = [[], [], [], [], []]
        target_lst_pose = [[], [], [], [], []]
        completed_img_dict = {}
        
        if (YOLO_pred.shape[0] == 0): # Nothing is detected
            pred = np.zeros((96,128)) # get mask image (background)
            colour_map = self.visualise_output(pred)
        else:
            pred = np.zeros((480,640)) # get mask image (background)
            yolo_stuff = []
            for i in range(YOLO_pred.shape[0]):
                xmin = int(YOLO_pred[i,0])
                ymin = int(YOLO_pred[i,1])
                xmax = int(YOLO_pred[i,2])
                ymax = int(YOLO_pred[i,3])
                label = class_converter[YOLO_pred[i,5]]

                u_0 = 640/2 if args.using_sim else 320/2
                v_0 = 480/2 if args.using_sim else 240/2
                
                fruit_xcent = (xmin + xmax)/2 - u_0
                fruit_ycent = (ymin + ymax)/2 - v_0
                fruit_width = xmax - xmin
                fruit_height = ymax - ymin
                
                yolo_stuff[i] = (label, [fruit_xcent,fruit_ycent, fruit_width, fruit_height])
                pred[ymin:ymax, xmin:xmax] = np.ones((ymax-ymin, xmax-xmin)) * label

            for (target_num, box) in yolo_stuff:
                target_lst_box[target_num-1].append(box) # bounding box of target
                target_lst_pose[target_num-1].append(np.array(image_pose).reshape(3,)) # robot pose

            # if there are more than one objects of the same type, combine them
            for i in range(5):
                if len(target_lst_box[i])>0:
                    box = np.stack(target_lst_box[i], axis=1)
                    pose = np.stack(target_lst_pose[i], axis=1)
                    completed_img_dict[i+1] = {'target': box, 'robot': image_pose}    

            np.resize(pred, (96, 128))
            colour_map = self.visualise_output(pred)
        
        return pred, colour_map, completed_img_dict
        ##########################################

        

    def visualise_output(self, nn_output):
        r = np.zeros_like(nn_output).astype(np.uint8)
        g = np.zeros_like(nn_output).astype(np.uint8)
        b = np.zeros_like(nn_output).astype(np.uint8)
        for class_idx in range(0, self.args.n_classes + 1):
            idx = nn_output == class_idx
            r[idx] = self.colour_code[class_idx, 0]
            g[idx] = self.colour_code[class_idx, 1]
            b[idx] = self.colour_code[class_idx, 2]
        colour_map = np.stack([r, g, b], axis=2)
        colour_map = cv2.resize(colour_map, (320, 240), cv2.INTER_NEAREST)
        w, h = 10, 10
        pt = (10, 160)
        pad = 5
        labels = ['apple', 'lemon', 'pear', 'orange', 'strawberry']
        font = cv2.FONT_HERSHEY_SIMPLEX 
        for i in range(1, self.args.n_classes + 1):
            c = self.colour_code[i]
            colour_map = cv2.rectangle(colour_map, pt, (pt[0]+w, pt[1]+h),
                            (int(c[0]), int(c[1]), int(c[2])), thickness=-1)
            colour_map  = cv2.putText(colour_map, labels[i-1],
            (pt[0]+w+pad, pt[1]+h-1), font, 0.4, (0, 0, 0))
            pt = (pt[0], pt[1]+h+pad)
        return colour_map

    def load_weights(self, ckpt_path):
        ckpt_exists = os.path.exists(ckpt_path)
        if ckpt_exists:
            ckpt = torch.load(ckpt_path,
                              map_location=lambda storage, loc: storage)
            self.model.load_state_dict(ckpt['weights'])
        else:
            print(f'checkpoint not found, weights are randomly initialised')
            
    @staticmethod
    def np_img2torch(np_img, use_gpu=False, _size=(192, 256)):
        preprocess = transforms.Compose([transforms.ToPILImage(),
                                         transforms.Resize(size=_size),
                                        # transforms.ColorJitter(brightness=0.4, contrast=0.3,
                                        #                         saturation=0.3, hue=0.05),
                                         transforms.ToTensor(),
                                         transforms.Normalize(
                                             mean=[0.485, 0.456, 0.406],
                                             std=[0.229, 0.224, 0.225])])
        img = preprocess(np_img)
        img = img.unsqueeze(0)
        if use_gpu:
            img = img.cuda()
        return img
