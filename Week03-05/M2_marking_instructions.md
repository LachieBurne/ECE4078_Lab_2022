# M2 Marking Instructions 
You will need to demonstrate your SLAM in both the simulation and the physical robot during the Week 6 lab session.

**Please familiarise yourselves with these steps to ensure the demonstrators can finish marking your team in the allocated time**
- [In-person marking steps](#in-person-marking)
- [In-person marking checklist](#in-person-marking-checklist)
- [Zoom marking steps](#zoom-marking)
- [Zoom marking checklist](#zoom-marking-checklist)

Each team will have a **STRICT** time limit get marked, 20min for the simulator demo and 10min for the physical robot demo, according to this [marking schedule](https://docs.google.com/spreadsheets/d/14GB1km85aYwIS4eiDUr7CUI0OCfg7yIZ2AUgJ6iAMlA/edit?usp=sharing). You may open up the marking checklist, which is a simplified version of the following steps to remind yourself of the marking procedures. 

**Note:** For the remote students (Lab 6), you will only need to demonstrate yours in the simulator.

### In-person marking
#### Step 1:
**Do this BEFORE your lab session**
Zip your **whole Week03-05 and catkin_ws folders** (see the picture below) to the Moodle submission box (according to your lab session). Each group only needs one submmission. This submission is due by the starting time of the lab session, which means you should **submit your script BEFORE you come to the lab**. 

**Tips:** 
- You may also include a text file in the zip file with a list of commands to use, if you don't know all the commands by heart.
- **Please practise** the marking steps (eg. unzipping your code and running it) to ensure there are no issues during marking.


#### Step 2: 
**Do this BEFORE the demonstrator come to mark your team**

1. Close all the windows/applications on your Ubuntu environment

2. Use any team member's account to log in Moodle in the Ubuntu environment (VM/native/WSL2, whichever one you use) and nagviate to the M2 submission box, so that you are ready to download your submitted code when the demonstrator arrives

3. Have an **empty** folder named "LiveDemo" ready at the Ubuntu home directory, ie. it is at ```~/LiveDemo/```. This folder should remain open at all time during marking (Does not apply to WSL2 users)

4. Please turn on your robot before the demonstrator comes to you. DO NOT connect to its hotspot yet 

#### Step 3:
**During marking**
Similar to M1, your simulation demo and physical demo will be marked separately. After you receive your robot in the beginning of the lab session, you may divide your team into two groups. One group can start preparing for the simulation demo, and the other group can start to calibrate the robot (**highly recommended especially if you have a different robot**). You will be allowed to copy the physical robot calibration files later on during marking.  Note: within the marking time limit (20min for sim, 10min for robot), you may re-run SLAM as many times as you want. The attempt with the highest score will be your final score. 

1. The demonstrator will release the map ```marking_map.txt``` via Slack in the beginning of the session for the simulation demo. Note that each lab session will get a slightly different map, so there is no point sharing the map with people in the other lab session

2. When the demonstrator starts to mark you, download your submitted zip file from Moodle and unzip its content to the "LiveDemo" folder. 

3. **For sim demo** place the true map inside the ```Week03-05``` folder. **For robot demo** place the robot calibration files in the ```/calibration/param``` folder

3. **[SIM ONLY]** Open a terminal and type ```source ~/LiveDemo/catkin_ws/devel/setup.bash```

4. **[SIM ONLY]** Launch the simulator with ```roslaunch penguinpi_gazebo ECE4078.launch```

5. **[SIM ONLY]** Spawn the markers with ```rosrun penguinpi_gazebo scene_manager.py -l marking_map.txt``` 

6. Open another terminal, or new tab in the existing terminal, navigate to the "Week03-05" folder which contains the operate.py script

7. Run the script with either ```python3 operate.py``` [for sim] or ```python3 operate.py --ip 192.168.50.1 --port 8080``` [for robot]

8. Demonstrate the SLAM by **strategically** driving the robot around arena and search for all the markers
    - You will have up to 5min to do so and you may stop anytime by pressing ```s``` to save the map 
    - If you have not reached your time limit, you may re-run this step as many times as you want

9. **[SIM ONLY]** Run the SLAM evaulation script ```python3 SLAM_eval.py marking_map.txt lab_output/slam.txt``` to check your RMSE. The demonstrator will also note down your RMSEs

10. If you want to have another attempt at the SLAM , please rename your ```lab_output/slam.txt``` to ```lab_output/{sim or robot}_slam_{trial_no}.txt```. For example, if this is your 2nd attempt for the robot demo, the file name should be ```robot_slam_2.txt```

12. Rename your output files to (please include all your attempts, and rename the files according):
- Example files:
    - sim_slam_1.txt
    - sim_slam_2.txt
    - robot_slam_1.txt
    - robot_slam_2.txt
    - robot_slam_3.txt

13. Submit this zip folder to the Moodle map submission box

---

### In-person marking checklist
**BEFORE the lab session**
- [ ] Submit your code to Moodle

**BEFORE the marking**
- [ ] Close all programs and folders
- [ ] In Ubuntu, login Moodle and navigate to the submission box
- [ ] Open an empty folder named "LiveDemo" (located at ```~/LiveDemo/```)
- [ ] Turn on the robot (DO NOT connect to its hotspot yet)

**During the marking**
- [ ] Demonstrator will ask you to download your submission and unzip it to "LiveDemo"
- [ ] Copy the true map [SIM] or the calibration files [ROBOT] to the "LiveDemo" folder 
- [ ] **[SIM ONLY]** ```source ~/LiveDemo/catkin_ws/devel/setup.bash```
- [ ] **[SIM ONLY]** ```roslaunch penguinpi_gazebo ECE4078.launch```
- [ ] **[SIM ONLY]** ```rosrun penguinpi_gazebo scene_manager.py -l marking_map.txt```
- [ ] Demonstrate SLAM in the simulation for up to 5min
- [ ] **[SIM ONLY]** ```python3 SLAM_eval.py marking_map.txt lab_output/slam.txt```
- [ ] rename the ```lab_output/slam.txt``` file(s)
- [ ] zip and submit the maps and screenshots to Moodle

---
### Zoom marking
#### Step 1:
**Do this BEFORE your lab session**
Zip your whole Week03-05 and catkin_ws folders (see the picture below) to the Moodle submission box (according to your lab session). Each group only needs one submmission. This submission is due by the starting time of the lab session, which means you should **submit your script BEFORE you come to the lab**. 

- You may also include a text file in the zip file with a list of commands to use, if you don't know all the commands by heart.
- **Please practise** the marking steps (eg. unzipping your code and running it) to ensure there are no issues during marking.


You will also need to install ```screenkey``` in  your Ubuntu. You can do so with ```sudo apt-get install screenkey```. We use this software to monitor your key press event. You can try this out by typing ```screenkey``` in the terminal, and then just type something. A bar should show up at the bottom of your screen showing what keys you have pressed. You can stop screenkey with this command ```pkill -f screenkey```.


#### Step 2: 
**Do this BEFORE the demonstrator come to mark your team**

1. Close all the windows/applications on your Ubuntu environment

2. Use any team member's account to login Moodle in the Ubuntu environment (VM/native/WSL2, whichever one you use) and nagviate to the M2 submission box, so that you are ready to download your submitted code when the demonstrator arrives

3. Have an **empty** folder named "LiveDemo" ready at the Ubuntu home directory, ie. it is at ```~/LiveDemo/```. This folder should remain open at all time during marking (Does not apply to WSL2 users)

4. If you have a dual monitor setup, please disconnect one of them. You are only allowed to use a single monitor during marking

#### Step 3:
**During marking**
1. Please make sure only one display is used during the live demo. You will be asked to show your display setting with:
- Windows: Settings -> System -> Display
- Linux: xrandr | grep connected | wc -l (type this in terminal and then count the lines returned)
- Mac: System Preferences -> Displays

2. When the demonstrator starts to mark you, download your submitted zip file from Moodle and unzip it to the "LiveDemo" folder

3. Open a terminal and type ```screenkey```

4. type ```source ~/LiveDemo/catkin_ws/devel/setup.bash```

5. Launch the simulator with ```roslaunch penguinpi_gazebo ECE4078.launch```

5. Spawn the markers with ```rosrun penguinpi_gazebo scene_manager.py -l marking_map.txt``` 

6. Open another terminal, or new tab in the existing terminal, navigate to the "Week03-05" folder which contains the operate.py script

7. Run the script with ```python3 operate.py``` 

8. Demonstrate the SLAM by **strategically** driving the robot around arena and search for all the markers. You will have 5min to do so. If you have not reached your time limit, you may re-run this step as many times as you want

9. Run the SLAM evaulation script ```python3 SLAM_eval.py marking_map.txt lab_output/slam.txt``` to check your RMSE. The demonstrator will also note down your RMSEs

10. If you want to have another attempt at the SLAM , please rename your ```lab_output/slam.txt``` to ```lab_output/sim_slam_{trial_no}.txt```. For example, if this is your 2nd attempt, the file name should be ```sim_slam_2.txt```

12. Rename your output files to (please include all your attempts, and rename the files according):
- Example files:
    - sim_slam_1.txt
    - sim_slam_2.txt

12. Submit this zip folder to the Moodle map submission box

---
### Zoom marking checklist
**BEFORE the lab session**
- [ ] Submit your code to Moodle
- [ ] Install ```screenkey``` in Ubuntu 

**BEFORE the marking**
- [ ] Close all programs and folders
- [ ] In Ubuntu, login Moodle and navigate to the submission box
- [ ] Open an empty folder named "LiveDemo" (located at ```~/LiveDemo/```)
- [ ] Only use a **single monitor** during marking

**During the marking**
- [ ] Show that you are only using a single monitor
- [ ] Demonstrator will ask you to download your submission and unzip it to "LiveDemo"
- [ ] Open a terminal and start ```screenkey```
- [ ] ```source ~/LiveDemo/catkin_ws/devel/setup.bash```
- [ ] ```roslaunch penguinpi_gazebo ECE4078.launch```
- [ ] ```rosrun penguinpi_gazebo scene_manager.py -l marking_map.txt```
- [ ] Demonstrate SLAM in the simulation for up to 5min
- [ ] ```python3 SLAM_eval.py marking_map.txt lab_output/slam.txt```
- [ ] rename the ```lab_output/slam.txt``` file(s)
- [ ] zip and submit the maps and screenshots to Moodle
