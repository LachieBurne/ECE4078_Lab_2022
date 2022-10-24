# Milestone 5: Integrated System (trial run for final demo)
- [Introduction](#introduction)
- [Marking schemes](#marking-schemes)
    - [Evaluation](#evaluation)
    - [Rules](#rules)
    - [Marking instructions](#marking-instructions)
- [Tips](#Tips)

## Introduction
Milestone 5 is the integration of all the modules you completed in the previous miletones. Integrate your work from M1 to M4 so that the robot can create a map of the arena containing the estimated poses of 10 ArUco markers ([M2: SLAM](../Week03-05/)),  5 types of fruits, i.e. red apple, lemon, orange, pear and strawberry ([M3: CV](../Week06-07/)), and then perform the fruit searching task ([M4: navigation](../Week08-09/)). The actual task is very similar to the level 3 M4 task, which is

**M5 task:** Given a random list of 3 fruits contained in [search_list.txt](search_list.txt), your task is to autonomously navigate to the given list of fruits in order, while avoiding obstacles along the way. The robot should stop within 0.5m radius of the target for 3 seconds before moving onto the next target. The following also applies to M5:
- A true map will be not given in M5
- You may teleoperate your robot to first generate a map of the arena with your own SLAM, then teleoperate your robot again to estimate the pose of the fruits before running the autonomous fruit search task
- There will be 10 ArUco markers and 7 fruits (3 to search, 4 as obstacles) in the arena. For example, you may need to search 1 x (apple + lemon + orange), then there will be 2x (pear + strawberry) as obstacles
- You may choose to perform manual waypoint navigation (same as [M4 Level 1](../Week08-09/README.md#marking-schemes)) to get some partial marks if you are really struggling to complete the whole task

 **The final demo will follow the same procedure**, M5 is also your trial run for final demo to help you identify improvements to be made before the final demo.

**Integration and improvement to consider:**
- Perform SLAM and object recognition simultaneously, so that after manually driving the robot around the arena once (to save time), you will have both the estimated poses of ArUco markers and the estimated poses of objects
- If you are taking the manaul waypoint navigation approach, improve the map visualization for interactively specifying waypoints during delivery and verifying whether the apples / lemons are inside / outside the 0.5m radius
- Make use of visual information to correct the auto navigation during delivery
- During the search, consider including oppurtunities for the robot to self-correct its delivery plan or reset its location in the arena to prevent errors accumulating during the delivery process
- Check out the [M2 FAQs](../Week03-05#FAQs-M2) for improving your SLAM
- Generate your own test cases and est your CV component's ability to handle occlusion (e.g., an ArUco marker blocking a fruit model) and make pose estimation when seeing more than one of the same type of object in a frame (e.g., seeing 2 apples at the same time)

**Important note**: 
- As usual, you are not allowed to use true poses of robot or objects retrieved from Gazebo, or teleport the robot or objects for the simulation
- Although the final demo is likely to be the same as M5 (at least the task), minor details/rules/marking scheme are still subject to change
- **PLEASE COMMENT YOUR CODE** this will speed up the marking process and allow us to provide better feedback if you clearly explain your workflow in your code
- **[UPDATE]**: Three practice maps with 7 fruits in the arena, along with the corresponding search lists are provided in the [practice_maps](practice_maps) folder
- **[UPDATE]**: The [mapping_eval.py](mapping_eval.py) has been provided to evaluation the mapping performance of your robot. See [line 211](mapping_eval.py#L211) for usage. Examples input files are provided in the [example_mapping_evaluation](example_mapping_evaluation) folder

---
## Marking schemes
### Evaluation
The following point system is the same for both the ```simulation_score``` and the ```robot_score```. We have divided M5 into 3 components:
1. Arena mapping (40pts)
2. Waypoint navigation (10pts)
3. Fruit searching (50pts)

Same as the previous milestones, 
```
final_score = 0.8 x simulation_score + 0.2 x robot_score
```

#### Arena mapping (40pts)
##### SLAM
Before the fruit searching task, you may choose to manually drive the robot around first and create a map as ```slam.txt``` ([M2: SLAM](../Week03-05/)) and the list of detected target poses as ```targets.txt``` ([M3: CV](../Week06-07/)), or you can generate the arena map and the target poses during the autonomous searching task. Eitherway, we will evaulate your ```slam.txt```:

```
mapping_score = (mapping_factor - Aligned_RMSE)/(mapping_factor - 0.02) x 16 + NumberOfFoundMarkers x 0.4
```

where ```mapping_factor = 0.2``` for simulation and ```mapping_factor = 0.15``` for physical robot. The maximum mapping_score is 20pts and minimum mapping_score is 0

##### Target pose estimation
```targets.txt``` should contain the poses of 7 fruits (3x fruit-to-search and 4x obstacle fruits). Similar to M3, the target score for each fruit will be calculated based on the pose estimate error, using

~~~
target_score[fruit] = (1 - estimation_error[fruit])/(1-0.025) x 2.86
~~~

For example, if your robot needs to search for {apple, lemon, orange}, then there will be 2 x {pear, strawberry} as obstacles, and here is an example score 

~~~
target_score[apple_0] = 2.86    # 0 estimation_error
target_score[lemon_0] = 2.86
target_score[orange_0] = 2.86
target_score[pear_0] = 2.86
target_score[pear_1] = 2.86
target_score[strawberry_0] = 1  # some estimation_error
target_score[strawberry_1] = 0  # estimation_error > 1

sum(target_score) = 15.3
~~~

The maximum sum(target_score) is 20pts and minimum sum(target_score) is 0.


#### Waypoint navigation (10pts)
If your robot shows indication of performing waypoint navigation, either through manually entering waypoints or autonomously navigating the arena, and there is evidence of waypoint navigation implementation in your code, then you will receive 10pts. 

#### Fruit searching (50pts)
Your robot has to perform the fruit searching task autonomously in order to get full marks for this sub-task. You will receive 20pts for reaching the first fruit, and 15pts for each of the remaining fruits, which make up a total of 50pts. 

If you choose to do this semi-autonomously (same as M4 Level 1), then you will receieve 40% of the marks (max of 20pts) for this component. You will receive 8pts for reaching the first fruit and 6 pts for each of the remaining fruits.

### Rules
1. Penalty of -2pts for each fruit that the robot collides with

2. Penalty of -5pts for each ArUco marker that the robot collides with

3. Penalty of -5pts each time the robot goes out of the boundary/touches the boundary (+/-1.5m from the origin, in both the x- and y-axis)

4. If you have received three penalties (any of the penalties listed above) during a run, you will receive zero score for that run
	- e.g. zero score for colliding into any object 3 times or collided into 2 objects + out of the boundary once

5. ~~You may stop your run at any point, and then your score that run will be calculated~~ When your robot has stopped moving by itself, you may stop your run, and then your score for that run will be calculated
    - If you stop the run while the robot is moving, or if you manually stop the robot/script, you will receive zero score for that run. This is to prevent teams from stopping the run when the robot reach a fruit by luck

6. The **entire** robot has to be within 0.5m of the target fruit to be considered as a successful collection

7. If the robot reaches the target fruits in the wrong order, you will receive zero score for that run

8. We will check your code to see if you have implemented to appropriate algorithms. To gain credit for the autonomous searching task, we must find evidence of path planning, or obstacle detection and avoidance (respectively) in your code. Successfully collecting fruit and/or avoiding collisions at these levels by luck will not grant you those marks by default

9. When you are manually mapping the arena (if you choose to), you must start at the origin (0, 0, 0)

10. Time limit is **30min for simulation** and **15min for physical robot**
    - You should end your last run ~2min before the time limit, to allow efficient time to submit the map files

11. The best run/attempt will be considered as your final score


### Further clarification
- For the simulation, you are **not allowed** to use the true map for the fruit searching task. The task should be based on the pose estimations given by your SLAM and CV
- You are **not allowed** to use the transformation specified in SLAM_eval.py or mapping_eval.py and apply it to the maps used in subsequent tasks, since this transformation is calculated using the true map
- ~~~You should include everything you need for running your demo in your submission, including the evaluation scripts (SLAM_eval, CV_eval, RetreivePose). If you haven't submitted the evaluation scripts, during demo you can only download the evaluation scripts from github and use these standard evaluation scripts~~~
- During delivery, if you are performing it semi automatically, the waypoints you can provide are x,y coordinates, you can't specify driving instructions, like distance / time to drive or turning angles, same as how you did your M4
- During the search, if you are performing it fully automatically, your search function needs to make a visible attempt, i.e., it needs to try to find a path to the target fruits and try to navigate towards it
- Similar to M2, you will need to submit the slam.txt as well as the targets.txt to the Moodle submission box right after marking for evaluation

### Marking instructions
Refer to [M5_marking_instructions.md]()

---
## Tips
Below are some suggestions on how you may improve your integrated system and live demo performance.

### General remarks
- In the marking arenas of M5 and final demo, there will be 10 ArUco marker blocks, 3 types of fruit (1 each) to search and 2 types (2 each) as obstacles. At the starting location (0, 0, 0) the robot will be able to see at least one marker. The objects will have gaps in between them that the robot should be able to fit through. 
- Make sure you have included everything required to run your demo in the submission. If you can't run the demo from your downloaded submission we can't allow you to run from your local working directory instead. 
- Practice the demo process, especially the set-up steps, so that you are familiar with it
- Consider having a back-up driver in case the lead person running the demo has any kind of unexpected emergencies

### SLAM
- When calibrating your scale and baseline parameter, make sure your the wheel velocities range in wheel_calibrate.py is close to the velocity the pibot usually operates. (i.e, by default 20 ticks/s and 5 ticks/s for forward and turning speed respectively) 
- A [wheel_test.py](wheel_test.py) is also provided to test the accuracy of tuned driving parameters. You can use it to drive the robot x metre forward and rotate the robot x degree.
- To calibrate the noise/covariance matrix of the pibot model, you should estimate the values based on what you observed from the tuning and testing phase. You can get a more accurate set of values by calculating the variance for each driving strategy that you use. For example, for driving straight, drive forward 1m for 10 times and record the variance and convert to tick/s. In general, if the simulation is slow or inconsistent, the values should be higher:
    ```
    if lv and rv == 0: # Stopped
        cov = 0
    elif lv == rv:  # Lower covariance since driving straight is consistent
        cov = 1
    else:
        cov = 2 # Higher covariance since turning is less consistent
    ```

- In [ekf.py](../Week03-05/slam/ekf.py#L129) the second expression in the process noise equation can be commented out since it may add too much noise to the model which accumulates even if the robot is idle. However, you can also add a condition to add this term only when the robot is moving or lower the noise. 
    ```
    Q[0:3,0:3] = self.robot.covariance_drive(raw_drive_meas) #+ 0.01*np.eye(3)
    ```
    
- To test if you implemented derivative_drive and predict correctly, run slam in an empty map (without markers) and record the robot pose. If the robot pose is way off, your calculation is most likely to be wrong. 
- To test if you implemented derivative_drive and predict correctly, run slam with markers and drive around. If RMSE is too high, the calculation for update is most likely to be wrong.

### CV
- Consider to generation your own test cases to test your CV component without the influence of SLAM accuracy. This can easily be done in the simulation 
- Make sure you have taken into account occulusion (e.g., there is a marker in front of the apple, which makes the bounding box shorter than it should be and puts the estimation of the apple further away). Using YOLO would give you more accurate bounding boxes that handles occulusion
- Check how you merge multiple bounding boxes of the same object type. The skeleton code just takes the average, but if you are seeing more than one of the same object type (e.g., two apples in the same frame) merging the bounding boxes can introduce errors
- Check which pose estimations you save at the end. The CV evaluation script will take the first 3 estimations of each object type, so you probably want to check your pose estimations and merge those that are likely to belong to the same object or remove those that are obviously wrong
- If you plan to start the CV component at another position than (0,0,0), make sure to include a pose translation function so that the generated object pose estimation map is realigned to the world frame
- Run SLAM and CV at the same time and generate both maps (SLAM map and object pose estimation map) can give you more time for a second try if needed

### Fruit searching
- Make use of your SLAM component to continuously correct pose estimation during the delivery process and update the path planning
- Make use of visual information to improve the pose estimation and path planning. For example, keeping an object in the center of the robot's view when driving towards it
- Beware of which are the x and y coordinates when reading and saving each map



