# How to set up your Simulator Environment or Robot
This readme provides a step-by-step guide on setting up your simulator environment and connecting to the physcial PenguinPi robot.
- [Installing Linux](#Installing-Linux)
    - [Option 1: using the pre-built VM](#Option-1:-Using-the-pre-built-virtual-machine-image)
    - [Option 2: using WSL2](#Option-2:-Installing-WSL2-on-Windows)
    - [Option 3: dual boot your computer](#Option-3:-Dual-boot-your-computer)
- [Install the environment from scratch (Only for Option 2 & 3)](#Install-the-simulator-environment-from-scratch-in-an-empty-Ubuntu-18)
- [Launch the Simulator Environment](#Launch-the-Simulator-Environment)
- [Running on the Physical Robot](#Running-on-the-Physical-Robot)
- [Troubleshooting and Known Issues](#Troubleshooting-and-Known-Issues)

# Installing Linux 

Before setting up ROS and the simulator, you need to install Linux on your machine. We provide three options here for you and Option 1 is the easiest solution but it may not give you the best performance in the simulator, computation-wise. The other two options would likely to give you some boost in computational performance, but may require a bit more work to get things set up. We strongly recommend you to stick with Option 1 if you are not comfortable with setting up your own ROS environment. 

Note that for people with Mac (especially with Apple M1/M2 chips), please work with the demonstrators to see what works best, as some of these instructions may not work for you.

## Option 1: Using the pre-built virtual machine image 

Install [Oracle VM Virtualbox](https://www.virtualbox.org/) (make sure your [BIOS setting allows virtualization](https://forums.virtualbox.org/viewtopic.php?f=1&t=62339))

Download [the ready-to-use VM image](https://drive.google.com/file/d/1_MRbKHOA1Lo_qFBaASKvvaQ36HuJRIPI/view?usp=sharing) and [import](https://docs.oracle.com/en/virtualization/virtualbox/6.0/user/ovf.html#ovf-import-appliance) it to your VirtualBox (username: ece4078  password: 2021Lab). 

You can change the amound of resources (e.g., RAM, processing cores) assigned to your VM in VirtualBox settings after importing the VM image depending on the hardware available to you.

## Option 2: Installing WSL2 on Windows
The new Windows Subsystem for Linux (WSL2) has nice supports for GUI app (eg. Gazebo), and it may run faster on your laptop/PC compared to running to a virtual machine. However, the nice GUI support for WSL2 only works well on Windows 11 (and certain Windows 10 build), so this option may not be viable for you if you don't have Windows 11. 

Please follow these [instructions](https://ubuntu.com/tutorials/install-ubuntu-on-wsl2-on-windows-11-with-gui-support#2-install-wsl) to set up your WSL2. Note that you should install **Ubuntu 18.04**, and you can skip the step to install octave.

To access the files in your Win11 from the Ubuntu subsystem, in your ubuntu terminal you can use ```cd /mnt/```. For example, if you have a folder named ```test_folder``` in the C:\ drive of Win11, you can navigate to that directory using ```cd /mnt/c/test_folder```. To transfer files between Win11 and Ubuntu, you can use the ```cp``` command (more info [here](https://manpages.ubuntu.com/manpages/trusty/man1/cp.1.html)). For example, if you have a file at C:/test.txt in Win11, you can copy it to your Ubuntu using ```cp /mnt/c/test.txt your_destination_dir```.

Otherwise, you may try [this](https://devblogs.microsoft.com/commandline/access-linux-filesystems-in-windows-and-wsl-2/) if you prefer using explorer to drag and drop. You may place your files/folders in ```/home/your_user_name/```.

## Option 3: Dual boot your computer 
CAUTION! Please only procced with this option if you know what you are doing, and do this at your own risk.

Here is a [tutorial](https://www.youtube.com/watch?v=CWQMYN12QD0&ab_channel=TechnoTim) you can follow to set up dual boot on your computer. 

## Option 4: Remote desktop
If you are struggling to implement any of these 3 options on your laptop (especially for Mac M1 users), you may want to try to use a remote desktop software (eg. AnyDesk) and install Ubuntu on your home desktop if you have one. 

Note that you will only be able to use this method with the simulator and you won't be able to connect to the physical robot with the remote desktop. However, for Mac M1 users, as long as you have installed the required python dependencies, you will be able to connect to physical robot with your Mac and run the python code locally (eg. operate.py). 

# Install the simulator environment from scratch in an empty Ubuntu 18

***Note: ROS-Melodic requires Ubuntu 18, and you only need to do this if you have chosen either Option 2 or 3***

install python packages
```
sudo apt update
sudo apt install python3-pip curl python-flask python-gevent
python3 -m pip install --upgrade pip 
python3 -m pip install flask gevent pyyaml numpy requests opencv-python pynput pygame
```

install Gazebo 11 (after the installation is done you should be able to open Gazebo by typing ```gazebo``` in your terminal)
```
curl -sSL http://get.gazebosim.org | sh
```

install ROS Melodic
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install ros-melodic-desktop
```

install ROS Melodic packages for Gazebo 11
```
sudo apt install ros-melodic-gazebo11-plugins
sudo apt install ros-melodic-gazebo11-ros-pkgs 
sudo apt install ros-melodic-gazebo11-ros-control
sudo apt install python-catkin-tools python3-dev python3-catkin-pkg-modules python3-rospkg-modules python3-numpy python3-empy
```

setup local catkin workspace
```
source /opt/ros/melodic/setup.bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
```

download the PenguinPi modules
```
cd ~/catkin_ws/src
sudo apt install git
sudo apt update
git clone https://bitbucket.org/cirrusrobotics/penguinpi_description.git
git clone https://bitbucket.org/cirrusrobotics/penguinpi_gazebo.git
git clone -b melodic https://github.com/ros-perception/vision_opencv.git
cd ~/catkin_ws
catkin_make
```

Replace the src folder in your local catkin_ws directory with [the src folder provided](https://drive.google.com/file/d/1VSmSmg7iuF-tQzWhlnmtwjsNhzxlPqU8/view?usp=sharing) which contains the required models


![Teleoperate the robot](Teleop.png?raw=true "Teleoperate the robot")


# Launch the Simulator Environment

From within the Virtual Machine, open a terminal and type
```
source ~/catkin_ws/devel/setup.bash
roslaunch penguinpi_gazebo ECE4078.launch
```
You should see a base Gazebo world open with PenguinPi inside an empty map
![Empty World](EmptyWorld.png?raw=true "Empty World")

Open a new terminal and spawn objects in the map by typing the following commands (right click on scene_manager.py and go to "Properties -> Permissions" to double check that the "Execute" box is ticked. This file is under the ~/catkin_ws/src/penguinpi_gazebo/ folder)
```
source ~/catkin_ws/devel/setup.bash
rosrun penguinpi_gazebo scene_manager.py -l map1.txt
```
You should see the empty map now populated with targets and markers.
Play around with the ```rosrun penguinpi_gazebo scene_manager.py``` command using the following variations:
- Run without ```-l map1.txt``` tag to spawn objects at random locations
- Run with ```-d``` tag to remove the spawned objects
- Run with ```-s NAME.txt``` to save a new map

You can change the number of targets and markers in the map by changing obj_class_dict at Line 23 of scene_manager.py

![Simulator Map](SimulatorMap.png?raw=true "Simulator Map")


# Running on the Physical Robot
![PenguinPi Robot](PenguinPi.png?raw=true "PenguinPi Robot")

Note: A 3D-printed cover case is added to the robot so that you can wipe the outside of the robot before and after your lab sessions.

First, switch the robot on with the side switch

Wait for it to boot - this takes about 1 min. When IP addresses appears on the OLED screen, the booting is finished. The IP address is most likely ```192.168.50.1```

Connect your PC's wifi to the PenguinPi's hotspot penguinpi:xx:xx:xx (look for it in your wifi list). The hotspot password is ```egb439123```

To run your codes on the robot, you need to find the IP address of your robot. This is shown on its OLED screen. When you run ```operate.py``` you will need to include the "--ip" and "--port" flags to your python running command, for example ```python3 operate.py --ip 192.168.50.1 --port 8080```. The python script will then be executed on the physical robot, instead of on the simulated robot. 

You can test your connection by opening a web browser and entering the address XXX.XXX.XXX.XXX:8080, where the first part is replaced by your robot's IP address. For example, 192.168.50.5:8080. On this webpage you will be able to test the motors and camera, as well as see a range of diagnostic information relating to your robot. 

![Web browser view of the physical robot](WebRobot.png?raw=true "Web browser view of the physical robot")

You can also conect to the robot using ssh (the ssh password is ```PenguinPi```)

```
ssh -X pi@192.168.50.1
```

You can upload files to the robot using the ```scp``` command:
```
scp -r ./LOCALDIR pi@192.168.50.1:~/REMOTEDIR
```

When you are done with the robot, inside the ssh session run ```sudo halt``` to shut down the robot safely. Once its lights stop flashing you can toggle the power switch. Don't toggle the switch directly to "hard" shut it down.

You can connect an external screen, keyboard, and mouse to the robot, then switch it on and use it as a Rasberry Pi. Inside the Rasberry Pi interface, you can install python packages onboard the robot by running pip in the terminal, e.g., ```python3 -m pip install pynput```. You can also install packages inside the ssh session if your PenguinPi has internet connection (you can set the internet connection up in the Rasberry Pi interface).

# Troubleshooting and Known Issues

- Virtual box not importing the image properly (not able to import and open the image at all): if the error is related to E_INVALIDARG, check if your virtualbox has been installed in C:\ (instead of D:\ or any other drive).
- Don't tick the 3D acceleration option in Virtualbox as it might cause the VM to black-screen or crash with an out-of-memory error.
- Razer graphic cards might have issues with the VM/gazebo/RViz (crashes without apparent error messages).
- If running the VM on a Mac, and encounter a "Kernel Driver Not Installed (rc=-1908 error)" follow [this guide](https://www.howtogeek.com/658047/how-to-fix-virtualboxs-%E2%80%9Ckernel-driver-not-installed-rc-1908-error/) to fix the issue.
- If using the PenguinPi robot in person, please be gentle as they can be quite fragile
- If installing from scratch and the command to install Gazebo: "curl -sSL http://get.gazebosim.org | sh" does nothing, follow the instructions under "Alternative installation: step-by-step" in http://gazebosim.org/tutorials?tut=install_ubuntu 
- Slow VM on Mac: this might be a resolution issue, see [here](https://www.reddit.com/r/virtualbox/comments/houi9k/how_to_fix_virtualbox_61_running_slow_on_mac/) for fixes. You can also try [enabling 3D acceleration](https://superuser.com/questions/172989/virtualbox-running-ubuntu-is-slow-mac-os-x-host) in VistualBox
- If you get the following error (especially for WSL2 users),

![cmakelist error](/Week01-02/cmake_error.png "cmakelist error")

You can try using ```catkin build``` instead of ```catkin_make```. Please try to the following steps:

1. remove the ```devel/``` and ```build/``` folder using ```sudo rm -r devel/ build/```
2. ```catkin build```
3. ```source devel/setup.bash```
4. Try your code again. If this works, then you should replace all future ```catkin_make``` command with ```catkin build```

