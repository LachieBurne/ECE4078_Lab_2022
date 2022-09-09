# Milestone 3: Object Recognition and Localisation
- [Introduction](#introduction)
- [Useful resources](#an-index-of-provided-resources)
- [Data collection (Week 6)](#step-1-data-collection)
- [Training the NN (Week 6)](#step-2-training-your-neural-network)
- [Estimating the target poses (Week 7)](#step-3-estimating-target-poses)


## Introduction
For milestone 3 you are required to detect and localise a list of target objects by training a target detector. In Week 6, you need to collect the training data and train your target detector with the helper scripts. In Week 7, you need to complete the codes to estimate the pose of the detected targets. 

The following is only a guideline to assist you to train a basic target detector that can provide sufficient performance for you to complete this milestone. However, you may want to explore using other state of the art object detection models such as [YOLOv5](https://github.com/ultralytics/yolov5). This guideline provides helper scripts for you to automate the data generation for the simulation. However, you will need to collect your own data set for detecting the fruits with the physical robot. We encourage you to write your own scripts to automate the training data generation for the real life data set. We have provided some suggestions on how to do so in instructions below. You may combine your synthetic data with the real data together for training a single model, or you can train two separate models for the simulation and the physical robot.

**Note:** If you decide to use other models, please make sure you follow the same naming convention and weight file (.pth) for evaulation purpose. Otherwise, your may get zero score for your submission

**Tip:** Make sure you take enough photos of the fake fruits during your Week 6 lab session so that you can start training your model after the lab

## An index of provided resources
- [operate.py](operate.py) is the central control program that combines keyboard teleoperation (M1), SLAM (M2), and object recognitions (M3). It requires the [utility scripts](../Week01-02/util), the [GUI pics](../Week01-02/pics) from M1, and the [calibration parameters](../Week03-05/calibration/param/) and [SLAM scripts](../Week03-05/slam/) of M2. In addition, it calles for the [detector model](#Training-your-neural-network) that you will develop in M3.
- [ECE4078_brick.world](ECE4078_brick.world) and [ECE4078_brick.launch](ECE4078_brick.launch) are used for launching a new testing arena with wall and floor textures added.
- [models.zip](https://drive.google.com/file/d/1-RMIWwW1THG4xKS_XeNMGs168aqACDL0/view?usp=sharing) contains the new fruit models for the simulation
- [FruitMap.txt](FruitMap.txt) is an example map to spawn the fruit models for testing
- [data_collector.zip](https://drive.google.com/file/d/1zxjZpJVTPvO_RGJGkwHJBu0dJRlktOqe/view?usp=sharing) contains scripts required for generating the [synthetic dataset](#Data-collection) used for training your target detector.
- [network](network/) contains [scripts](network/scripts) required for [training a target detector model](#Training-your-neural-network) locally and for training on [Google Colab](network/ECE4078_2022_Lab_M3_Colab).
- [CV_eval.py](CV_eval.py) is used for [evaluating the pose estimations](#Estimating-target-poses) given by your detector compared to the groundtruth map. The pose estimation is done by [TargetPoseEst.py](TargetPoseEst.py) which you will complete.

## Step 1: Data collection
### a) Copying the additional files
1. Download [ECE4078_brick.world](ECE4078_brick.world) and [ECE4078_brick.launch](ECE4078_brick.launch) to your ```worlds``` and ```launch``` folder inside ```catkin_ws/src/penguinpi_gazebo```

2. Unzip the [models.zip](https://drive.google.com/file/d/1-RMIWwW1THG4xKS_XeNMGs168aqACDL0/view?usp=sharing) folder to ```catkin_ws/src/penguinpi_gazebo/models```. This models folder contains new fruit models for the simulation world: orange, pear and strawberry  

3. Copy [scene_manager.py](scene_manager.py) to the ```catkin_ws/src/penguinpi_gazebo``` folder. Line 24 to 26 have been updated

4. Unzip [data_collector.zip](https://drive.google.com/file/d/1zxjZpJVTPvO_RGJGkwHJBu0dJRlktOqe/view?usp=sharing) to ```catkin_ws/src/```

### b) Changing environment textures
First, navigate to your ```catkin_ws``` directory, then open a terminal and run either ```catkin_make``` or ```catkin build```. If you run into trouble with this step with ```catkin_make```, try sourcing the ROS setup.bash by running 

```
source /opt/ros/melodic/setup.bash
```
Or try ```catkin build``` with the [instructions](../Week01-02/InstallationGuide.md#troubleshooting-and-known-issues) in the "Troubleshooting and known issues" guide.

Once the workspace has been built,
```
source ~/catkin_ws/devel/setup.bash
``` 
The data_collector package will now be recognised by ROS. Now run

```
roslaunch penguinpi_gazebo ECE4078_brick.launch
``` 
You should see an areana with brick walls and wooden floors. You can then spawn objects inside this new environment by opening a new terminal and run 
```
source ~/catkin_ws/devel/setup.bash
rosrun penguinpi_gazebo scene_manager.py -l /path_to_file_dir/FruitMap.txt
```
Try changing the wall and floor materials during development to test the robustness of your detector. To do so, open [ECE4078_brick.world](ECE4078_brick.world) in a text editor, search and replace "Gazebo/Bricks" with other pre-defined materials listed [here](http://wiki.ros.org/simulator_gazebo/Tutorials/ListOfMaterials) to change the wall material (4 occurrences in total), and search and replace "Gazebo/Wood" with other pre-defined materials to change the floor material (1 occurrence in total).

![Brick Walls](Screenshots/BrickWallWorld.png?raw=true "Brick Walls")

### c) Generating synthetic data
As training a neural network requires a lot of data, but manual data collection is time consuming, a ROS-based [data collector](https://drive.google.com/file/d/1zxjZpJVTPvO_RGJGkwHJBu0dJRlktOqe/view?usp=sharing) is provide for you to generate synthetic data for training. 

Open a terminal and install required packages for **python 2** used by the ROS data collector:
```
sudo apt install python-pip
python -m pip install tqdm h5py
```
In catkin_ws/src/data_collector/data_collector.py, replace the camera matrix at Line 33 with your [camera parameters](../Week03-05/calibration/param/intrinsic.txt) computed from the camera calibration process in M2.

**Close Gazebo if it is running.** Then open the Gazebo photo studio (an empty world with lighting set-up) by running the following command in a terminal 
```
source ~/catkin_ws/devel/setup.bash
roslaunch data_collector photo_studio.launch gui:=true
```

Now open a new terminal and generate the synthetic dataset by typing the commands below (note that we are using "python" instead of "python3")
```
source ~/catkin_ws/devel/setup.bash
roscd data_collector
python data_collector.py --dataset_name Synth --sample_per_class 1000
```
This creates a folder in "catkin_ws/src/data_collector/dataset/" called "Synth", in which there is a folder for each target class containing 1000 synthetic training images generated with grey background in the images folder (if you check the Gazebo window while the data is being generated, you can see the models appearing at random locations for taking each of these images). The "images" folder saves the original image, while the "labels" folder saves the segmentation labels (silhouette of the target model) with small variation in the black level for training your neural network. The black level variation is amplified in "labels_readable" so that you can visualize what the labels look like if you want.

![pear model with grey background](Screenshots/pear_grey.jpg?raw=true "Pear model with grey background")

Now we need to replace the grey background with random background images to increase the detector's robustness. Open a terminal and run the following commands (note that we are using "python", not "python3")
```
cd ~/catkin_ws/src/data_collector/
python randomise_background.py --dataset_name Synth
``` 
You should now see the images in "catkin_ws/src/data_collector/dataset/Synth" with random background pictures added to them. The background pictures are stored in "catkin_ws/src/data_collector/textures/random" and you can remove part of it or add your own collection.

![Pear model with random background](Screenshots/pear_rand.jpg?raw=true "Pear model with random background")

In the same terminal, after the background shuffling is done, run the dataset split script (note that we are using "python", not "python3"):
```
python split_dataset.py --sim_dataset Synth --training_ratio 0.8
``` 
This will separate the synthetic dataset randomly into a training set containing 80% of all images and an evaluation set containing 20% of all images, specified by "train.hdf5" and "eval.hdf5" generated under "catkin_ws/src/data_collector/dataset/", which will be used for training your neural network.


### d) Generating real world data
During the Week 6 lab session, make sure you take plenty of photos of the fake fruits provided. To make post processing simpler, use simple backgrounds like the picture below (e.g. white background with A4 paper). 

![real pear picture with white background](Screenshots/real_pear.jpg?raw=true "Real pear picture with white background")

Rotate the fruits around and take images at different view angles. 

The principles of processing these images are similar to the synethic data generation described above. Start with cropping out the background with tools of your choice, such as this [online tool](https://www.remove.bg/upload) to remove the background of the images. Then, we suggest you to write a script with OpenCV to automate randomly placing the cropped out fruit images on different backgrounds, as well as generating the segmentation labels. Finally, you maybe use the ```split_dataset.py``` script to split your dataset into a training set and a evaluation set. 


## Step 2: Training your neural network 
Install [PyTorch](https://pytorch.org/) (select "Pip" as Package and "None" for CUDA) and other dependencies for **python 3** using the following commands:
```
python3 -m pip install pandas h5py tqdm
python3 -m pip install torch==1.6.0+cpu torchvision==0.7.0+cpu -f https://download.pytorch.org/whl/torch_stable.html pip install torch==1.6.0+cpu torchvision==0.7.0+cpu -f https://download.pytorch.org/whl/torch_stable.html
python3 -m pip install -U PyYAML
```

We use [ResNet-18](https://pytorch.org/hub/pytorch_vision_resnet/) as our model, which is a deep convolutional neural network pre-trained on the ImageNet dataset. The structure of the ResNet18 model is specified in [res18_skip.py](network/scripts/res18_skip.py).

![ResNet-18](Screenshots/ResNet.jpg?raw=true "ResNet-18")

To train the neural network with the data you generated, run the commands below in a terminal (note that we are using "python3" now)
```
cd ~/YourDir/Week05-06/network/script/
python3 main.py --dataset_dir ~/catkin_ws/src/data_collector/dataset --model_dir model
```
You can change the [default parameters](network/scripts/args.py) by adding flags, such as ```--lr 0.05```, when running [main.py](network/scripts/main.py). Parameters that you can change, what they represent and their default values are specified in [args.py](network/scripts/args.py).

You will see some information printed regarding how a model performs for training and evalution during each epoch. Once training is done, a best performing model will be saved as "~/YourDir/Week05-06/network/scripts/model/model.best.pth". This model should be able to detect the different targets (red apple, yellow lemon, pear, orange, strawberry) and segment them from the background. Delete old checkpoints (.pth) before training new models.

If you would like to train your neural network on Google Colab, upload the [ECE4078_2022_Lab_M3_Colab folder](network/ECE4078_2022_Lab_M3_Colab) with your dataset included (images, labels, and the hdf5 files) to your Google Drive. Open [main_colab.ipynb](network/ECE4078_2022_Lab_M3_Colab/main_colab.ipynb) in Colab and then run its cells (you can change the default parameters in [args.py](network/ECE4078_2022_Lab_M3_Colab/args.py)). If you would like to use GPU or TPU with Colab, in the top menu, go to "Runtime" -> "Change runtime type" -> "Hardware accelerator" -> select "GPU" or "TPU" in the dropdown menu. After the training is done, you can view the performance of the trained network on 4 test images using [detector_debugger.ipynb](network/ECE4078_2022_Lab_M3_Colab/detector_debugger.ipynb), and you can download the generated best model "model.best.pth" to your local directory for the detector to use.

![Segmenting target from background](Screenshots/Prediction.jpg?raw=true "Segmenting target from background")
Your network should segment the input image into one of six classes (background, apple, lemon, pear, orange and strawberry). Detecting humans will NOT be required this year.

Once you are happy with the model's performance, you can use it for your robot. Load the [Gazebo world](#Environment-variances) with environment textures and targets, and then in a terminal run 
```
cd ~/YourDir/Week05-06/
python3 operate.py
```
You may need to change the path to the best performing model by running [operate.py](operate.py) with the ```--ckpt``` flag. Once the GUI is launched, you can run the target detector by pressing "p", this shows the detector's output in the bottom left of the GUI. 

![GUI view of the detector](Screenshots/DetectorWorking.png?raw=true "GUI view of the detector")

## Step 3: Estimating target poses
To estimate pose of targets, you will need to run the SLAM component (press ENTER to run) while running the target detector, so that the robot's pose is known. Every time you want to perform object detection, press "p" to run the detector, then press "n" to save the robot's current pose estimated by SLAM (as "/YourDir/Week05-06/lab_output/image.txt") as well as the corresponding detector's segmentation labels (similar to the images in the "labels" folder when generating the synthesized data, these segmentation labels appear all black. You can press "i" to save the raw image in addition for visual debugging of the pose estimation). After you have collected the detector's outputs for all targets, you can press "s" to save SLAM map and then exit the GUI by pressing ESC.

**Complete [TargetPoseEst.py](TargetPoseEst.py)** to estimate the locations of the apples, lemons, orange, pear and strawberry models based on the detector's outputs and the robot's poses. Replace [Lines 81-86](TargetPoseEst.py#L81) with your own codes to compute the target pose using the robot's pose and the detector's output. There are 2 of each fruit models in the marking map, so you can only output at most 2 estimations per target type in the estimation output. Replace [Lines 106-128](TargetPoseEst.py#L106) with your own codes to merge the estimations with a better way than taking the first 2 estimations. The [TargetPoseEst.py](TargetPoseEst.py) generates an estimation result file as "/YourDir/Week05-06/lab_output/targets.txt", which is in the same format as the groundtruth maps.

You can use [CV_eval.py](CV_eval.py) to evaluate performance of your target pose estimation. Run 
```
python3 CV_eval.py TRUEMAP.txt lab_output/targets.txt
```
This computes the Euclidean distance between each target and its closest estimation (the estimation error) and returns the average estimation error over all targets. If more than 3 estimations are given for a target type, the first 3 estimations will be used in the evaluation.
