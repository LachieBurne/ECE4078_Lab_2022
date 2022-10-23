# M5 Marking Instructions 
You will need to demonstrate your navigation and planning module in both the simulation and the physical robot during the Week 11 lab session.

**Please familiarise yourselves with these steps to ensure the demonstrators can finish marking your team in the allocated time <span style="color:red"> This marking procedure will be used for the final demo. PLEASE MAKE SURE YOU READ THIS.</span>**
- [In-person marking steps](#in-person-marking)
- [Zoom marking steps](#zoom-marking)


Each team will have a **<span style="color:red">VERY STRICT</span>** time limit get marked. Due to the additional complexity of the task, we have incrased the time limits to a total of 45min, **30min for the simulator demo and 15min for the physical robot demo**, according to this [marking schedule](https://docs.google.com/spreadsheets/d/14GB1km85aYwIS4eiDUr7CUI0OCfg7yIZ2AUgJ6iAMlA/edit?usp=sharing). Note that the 45min time limit includes the time for you to submit your map files to Moodle. You may open up the marking checklist, which is a simplified version of the following steps to remind yourself of the marking procedures. You MUST follow all the rules outlined in the [marking scheme](README.md#marking-schemes), make sure you check out all the rules and understand all of them.

**Important note:** 
- For the remote students (Lab 6), you will only need to demonstrate yours in the simulator. Your time limit is 30min.


---
### In-person marking
#### Step 1:
**Do this BEFORE your lab session**
Zip your **whole Week10-11 and catkin_ws folders** to the Moodle submission box (according to your lab session). Due to size limit and time to download, you do not need to submit your ResNet/YOLO model weight file. Each group only needs one submmission. This submission is due by the starting time of the lab session, which means you should **submit your script BEFORE you come to the lab**. 

**Tips:** 
- You may also include a text file in the zip file with a list of commands to use, if you don't know all the commands by heart.
- **Please practise** the marking steps (eg. unzipping your code and running it) to ensure there are no issues during marking.


#### Step 2: 
**Do this BEFORE the demonstrator come to mark your team**
1. <span style="color:red">Please calibrate your physical robot as soon as you receive yours</span>

2. Connect back to eduroam/internet so that you are ready to download your submission from Moodle

1. Close all the windows/applications on your Ubuntu environment

2. Use any team member's account to log in Moodle in the Ubuntu environment (VM/native/WSL2, whichever one you use) and nagviate to the M5 submission box, so that you are ready to download your submitted code when the demonstrator arrives

3. Have an **empty** folder named "LiveDemo" ready at the Ubuntu home directory, ie. it is at ```~/LiveDemo/```. This folder should remain open at all time during marking (Does not apply to WSL2 users)

**Tips:** 
- You could place your fruit detector model weight file and the robot calibration file on your desktop, right next to the "LiveDemo" folder, so that it will be convenient to copy those into the folder later on
- The demonstrators will email you the M5 true map (for setting up the Gazebo environment ONLY) and the [search_list.txt](search_list.txt), 5min before your allocated marking time. You may also place these files on your desktop


#### Step 3:
**During marking**
There 2 key tasks, mapping (ArUco markers + fruits locations) and fruit searching. It is up to you how you want to spend your time to do any of these tasks. 


1. The demonstrator will release the maps ```M5_true_map.txt``` and the ```search_list.txt``` via email. Make sure you have login your gmail to save time. Note that each lab session will get a slightly different map, so there is no point sharing the map with people in the other lab session

2. When the demonstrator starts to mark you, download your submitted zip file from Moodle and unzip its content to the "LiveDemo" folder. 

3. Please the true map, model weight file, robot calibration files to their according folder within the "LiveDemo" folder

4. **[SIM ONLY]** Open a terminal and type ```source ~/LiveDemo/catkin_ws/devel/setup.bash```

5. **[SIM ONLY]** Launch the simulator with ```roslaunch penguinpi_gazebo ECE4078.launch```

6. **[SIM ONLY]** Spawn the map with ```rosrun penguinpi_gazebo scene_manager.py -l M5_true_map.txt```

7. You may now attempt the 2 key tasks in whatever sequence, as many times as you want within the time limit

8. You should rename the ```slam.txt``` and the ```targets.txt``` files after each attempt, please use the following naming format:
    - slam_{robot/sim}\_{attempt_no}_{team_no}.txt, e.g. slam_robot_1_301.txt
    - targets_{robot/sim}\_{attempt_no}_{team_no}.txt, e.g. targets_robot_1_301.txt
    - you may receive zero score by failing to follow this naming format
    - You should also have two different folders to store the maps, one named "robot" and another one named "sim". <span style="color:red">Make sure you do not mix up the sim and robot maps</span>

9. **[SIM ONLY]** You may use the [mapping_eval.py](mapping_eval.py) to check your mapping performance if you want

10. For semi-auto searching, you may enter as many waypoints as you want. For autonomous fruit search, you can only input a single command to start the autonomous search
    - You will receive zero score if we find out that you are teleoperating the robot for the fruit searching task

#### Step 4:
**Right after marking**
1. zip all your "sim" and "robot" map folders, which contains all your ```slam.txt``` and the ```targets.txt``` file, and put them in a folder named M5_maps_{team_number} (e.g. M5_maps_404), your zip file should have the same name, M5_maps_{team_number}.zip. 
2. Upload the M5_maps_{team_number}.zip to the Moodle map submission box
---
### Zoom marking
#### Step 1:
**Do this BEFORE your lab session**
Zip your **whole Week10-11 and catkin_ws folders** to the Moodle submission box (according to your lab session). Due to size limit and time to download, you do not need to submit your ResNet/YOLO model weight file. Each group only needs one submmission. This submission is due by the starting time of the lab session, which means you should **submit your script BEFORE you come to the lab**. 

You will also need to install ```screenkey``` in  your Ubuntu. You can do so with ```sudo apt-get install screenkey```. We use this software to monitor your key press event. You can try this out by typing ```screenkey``` in the terminal, and then just type something. A bar should show up at the bottom of your screen showing what keys you have pressed. You can stop screenkey with this command ```pkill -f screenkey```.

**Tips:** 
- You may also include a text file in the zip file with a list of commands to use, if you don't know all the commands by heart.
- **Please practise** the marking steps (eg. unzipping your code and running it) to ensure there are no issues during marking.


#### Step 2: 
**Do this BEFORE the demonstrator come to mark your team**
1. Close all the windows/applications on your Ubuntu environment

2. Use any team member's account to log in Moodle in the Ubuntu environment (VM/native/WSL2, whichever one you use) and nagviate to the M5 submission box, so that you are ready to download your submitted code when the demonstrator arrives

3. Have an **empty** folder named "LiveDemo" ready at the Ubuntu home directory, ie. it is at ```~/LiveDemo/```. This folder should remain open at all time during marking (Does not apply to WSL2 users)

**Tips:** 
- You could place your fruit detector model weight file and the robot calibration file on your desktop, right next to the "LiveDemo" folder, so that it will be convenient to copy those into the folder later on
- The demonstrators will email you the M5 true map (for setting up the Gazebo environment ONLY) and the [search_list.txt](search_list.txt), 5min before your allocated marking time. You may also place these files on your desktop


#### Step 3:
**During marking**
There 2 key tasks, mapping (ArUco markers + fruits locations) and fruit searching. It is up to you how you want to spend your time to do any of these tasks. 


1. The demonstrator will release the maps ```M5_true_map.txt``` and the ```search_list.txt``` via email. Make sure you have login your gmail to save time. Note that each lab session will get a slightly different map, so there is no point sharing the map with people in the other lab session

1. Please make sure only one display is used during the live demo. You will be asked to show your display setting with:
- Windows: Settings -> System -> Display
- Linux: xrandr | grep connected | wc -l (type this in terminal and then count the lines returned)
- Mac: System Preferences -> Displays

2. When the demonstrator starts to mark you, download your submitted zip file from Moodle and unzip its content to the "LiveDemo" folder. 

3. Open a terminal and type ```screenkey```

3. Please the true map, model weight file, robot calibration files to their according folder within the "LiveDemo" folder

4. Open a terminal and type ```source ~/LiveDemo/catkin_ws/devel/setup.bash```

5. Launch the simulator with ```roslaunch penguinpi_gazebo ECE4078.launch```

6. Spawn the map with ```rosrun penguinpi_gazebo scene_manager.py -l M5_true_map.txt```

7. You may now attempt the 2 key tasks in whatever sequence, as many times as you want within the time limit

8. You should rename the ```slam.txt``` and the ```targets.txt``` files after each attempt, please use the following naming format:
    - slam_{attempt_no}_{team_no}.txt, e.g. slam_sim_1_301.txt
    - targets_{attempt_no}_{team_no}.txt, e.g. targets_sim_1_301.txt
    - you may receive zero score by failing to follow this naming format

9. You may use the [mapping_eval.py](mapping_eval.py) to check your mapping performance if you want

10. For semi-auto searching, you may enter as many waypoints as you want. For autonomous fruit search, you can only input a single command to start the autonomous search
    - You will receive zero score if we find out that you are teleoperating the robot for the fruit searching task

#### Step 4:
**Right after marking**
1. zip all your map files, ```slam.txt``` and the ```targets.txt``` **in a folder** named M5_maps_{team_number} (e.g. M5_maps_404), your zip file should have the same name, M5_maps_{team_number}.zip. Remember your map file should have the following naming format:
    - slam_sim_{attempt_no}_{team_no}.txt, e.g. slam_sim_1_301.txt
    - targets_sim_{attempt_no}_{team_no}.txt, e.g. targets_sim_1_301.txt
    - you may receive zero score by failing to follow this naming format
2. Upload the M5_maps_{team_number}.zip to the Moodle map submission box