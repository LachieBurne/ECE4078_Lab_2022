# Milestone 4: Navigation and Planning
- [Introduction](#Introduction)
- [Waypoint navigation (Week 8)](#waypoint-navigation-week-8)
- Path planning
	- [with a known map (Week 9)](#path-planning-with-a-known-map-week-9)
	- [with a partially known map (Week 9)](#path-planning-with-a-partially-known-map-week-9)
- [Marking schemes](#marking-schemes)

## Introduction
In M4 you will need to implement the fruit searching module. In the 3m x 3m arena, there will be 10 ArUco markers, 1 red apple, 1 yellow lemon, 1 pear, 1 orange and 1 strawberry. Since this milestone focuses on testing your navigation and planning algorithms, we will provide a [true map](M4_true_map_3fruits.txt) for you to simplify the SLAM component. This map will contain: 
- Locations the 10 ArUco markers
- Locations of 3 out of 5 fruits. The locations of the fruits to be searched are known, and the other two fruits will be considered as obstacles with unknown locations

**M4 task:** Given a random list of 3 fruits contained in [search_list.txt](search_list.txt), your task is to autonomously navigate to the given list of fruits in order, while avoiding obstacles along the way. The robot should stop within 0.5m radius of the target for 3 seconds before moving onto the next target

To achieve the M4 task, this fruit searching module can be divided into 3 components. These components build on top of each other, so we hightly recommend you to complete these components in the following order:

1. [Waypoint navigation](#waypoint-navigation-week-8)
2. [Path planning (known map)](#path-planning-with-a-known-map-week-9)
3. [Path planning (partially known map)](#path-planning-with-a-partially-known-map-week-9)

The [auto_fruit_search.py](auto_fruit_search.py) is provided to you as a skeleton code to get you started. Please note that this skeleton code is only a general guide, you can rewrite the whole script if you want. You may also submit multiple scripts/variants of this code for the different marking levels you wish to attempt as described in the [marking scheme](#evaulation).

**Important notes:**
- There will be a penalty for every object that the robot collides into
- The [true map](M4_true_map_3fruits.txt) is only provided to you to simplify this milestone (you may choose not use it), so please be aware that in future milestones you will need to generate your own map
- There are two true maps:
	1. [M4_true_map_3fruits.txt](M4_true_map_3fruits.txt) - only contains 3 fruits + 10 markers, it is used for 
		- spawn the map in Gazebo/real world for Level 2 (refer to [marking scheme](#marking-schemes) for the level definitions)
		- used as the true map input for Level 2 and Level 3
	2. [M4_true_map_5fruits.txt](M4_true_map_5fruits.txt) - contains 5 fruits + 10 markers, it is used for 
		- spawn the map in Gazebo/real world for Level 1 and Level 3 (refer to [marking scheme](#marking-schemes) for the level definitions)
		- used as the true map input for Level 1

---
## Waypoint navigation (Week 8)
As a starting point to complete the M4 task, you will need to develop a module to accept a list of waypoints as input to navigate the robot to the goal position. If you wish to demonstrate this component alone, you may manually enter the target waypoints. Otherwise, this module will take the output of the [path planning module](#path-planning-with-a-known-map-week-9) as input. 

**Key functions to complete:**
1. Control the robot to drive towards a waypoint at auto_fruit_search.py [line 108-124](auto_fruit_search.py#L108)
2. Estimate the robot pose at auto_fruit_search.py [line 130-136](auto_fruit_search.py#L130)
	- We strongly recommend you to use your SLAM code from M2 to estimate the robot's pose. If you choose to do so, you will have to copy the necessary SLAM related functions in [operate.py](../Week03-05/operate.py) from the Week03-05 folder

**General tips:**
- Although it is possible to estimate the robot's pose with odometry (a much simpler implementation), we **STRONGLY RECOMMEND** you to correct the robot's pose after every waypoint navigation step with information from the SLAM module (M2)
- If you wish to demonstrate this component alone, you may implement one of the following to assist you with the demo
	- *Command line waypoint input*: a command line interface to continuously take waypoints as inputs so that you can guide the robot to get to the goal manually 
	- *GUI waypoint input*: a visual-based waypoints selector (with OpenCV or Pygame), so that you can visually select where the robot should go instead of estimating numerical values of the goal coordinates. However this will require more work 


---
## Path planning with a known map (Week 9)
Once you have completed the waypoint navigation function, you can implement a path planning algorithm to allow the robot to generate a path (a set of waypoints) and autonomously nagivate to the goal. This function should only take the [true map](M4_true_map_3fruits.txt), where the locations of all the objects (10 ArUco markers and 3 types of fruit) in the arena are stored as input. It is best to have the file path of the map as an input argument for the Python script. Once the Python script has started, you can enter no more than one command for the robot to start its task. For this component, we assume the positions of all the objects in the arena are known, so that the robot should only need to perform the path planning algorithm once. You may use any path planning algorithm you like, such as rapidly exploring random trees (RRT) or A*.

---
## Path planning with a partially known map (Week 9)
In this component, we will introduce two types of fruit at unknown locations (unknown to the robot) in the arena. Building on top of the components described above, you will need to include
1. A fruit detector (implemented in M3) to detect and estimate the location of the two addition fruits, whose locations are initially unknown to the robot
2. Re-plan the robot's trajectory once an obstacle (new fruit) has been detected


---
## Marking schemes
### Evaulation
The following point system is the same for both the ```simulation_score``` and the ```robot_score```. We have divided M4 into 3 levels based on the 3 main components which you could attempt. The levels are
1. Semi-auto navigation with manual inputs (60pts)
2. Autonomous navigation with a known map (20pts)
3. Autonomous navigation with a partially known map (20pts)

**IMPORTANT!!** The following rules apply:
1. Penalty of -2pts for each fruit that the robot collides with

2. Penalty of -5pts for each ArUco marker that the robot collides with

3. Penalty of -5pts each time the robot goes out of the boundary/touches the boundary (+/-1.5m from the origin, in both the x- and y-axis)

4. If you have received three penalties (any of the penalties listed above) during a run, you are disqualified for that run and will receive zero score
	- e.g. zero score for colliding into any object 3 times or collided into 2 objects + out of the boundary once

5. Then end condition of a run is determined by the robot having stopped by the first two fruits, and clearly stopped within 1m of the last fruit. 1m is the acceptable distance to show that the last fruit was the intended stopping location,  while marks will only be given for stopping distances of <0.5m. With these conditions met, your score for that run will be calculated
	- If you decide to stop the run before the end condition, you will get zero score for that run
	- Your robot has to make a visible attempt to try to find all the target fruits, to at least drive pass them even if the robot may not accurately stop within 0.5m of the target. Therefore, if your robot only reaches the first fruit and just stops there, you will not receive any score, until the condition of a run has been satisfied

6. The **entire** robot has to be within 0.5m of the target fruit to be considered as a successful collection

7. If the robot reaches the target fruits in the wrong order, you will receive zero score for that run

8. We will check your code to see if you have implemented to appropriate algorithms for the levels you have attempted. To gain credit for level 2 or 3, we must find evidence of path planning, or obstacle detection and avoidance (respectively) in your code. Successfully collecting fruit and/or avoiding collisions at these levels by luck will not grant you those marks by default

9. The best run/attempt will be considered as your final score

The true map and a list of 3 target fruits will be given to you in the beginning of your live demo marking, then you will have up to 5min (same as M2) to perform each run to search for the target fruits in the correct order. If you have collected any amount of fruit in Level 2 or Level 3, you will inherit the all of Level 1 points plus Level 2 or Level 3 fruit collection points. For Level 3, the fruit collection point of each fruit will be multiply by 2, to inherit the collection point from Level 2 for each fruit collected. Figure 1 shows the point system for the fruit collection. 
For example, if you have collected 2 fruits in Level 2, you will get 
```
lv2_score = 60 (lv1 points) + 8 (1st fruit) + 6 (2nd fruit) = 74 pts
```
If you have collected 2 fruits in Level 3, you will get
```
lv3_score = 60 + 8 x 2 (1st fruit) + 6 x 2 (2nd fruit) = 88 pts
```

You should first demonstrate the level you are confident with before attempting a more challenging level. You may demonstrate any levels as many times as you want within the given timeframe (**20min for simulation, 10min for physical robot**).  

<p align="center">
	<img src="score_system.png" alt="score system" width="300"/>
</p>
<p align = "center">
	Figure 1. Point system for the different levels and number of fruits collected
</p>
 
Same as the previous milestones, 
```
final_score = 0.8 x simulation_score + 0.2 x robot_score
```

#### Level 1: Semi-auto navigation (60pts)
For this level, the arena will have 10 ArUco markers, 1 red apple, 1 yellow lemon, 1 pear, 1 orange and 1 strawberry. The locations of all the objects in the arena will be given to you in the [M4_true_map_3fruits.txt](M4_true_map_3fruits.txt) (you should have the file name as an input argument to run the script). The search order of the target fruits is given in [search_list.txt](search_list.txt)

You are **not allowed to teleoperate** the robot. You can only enter coordinate of the waypoints as input, or you may choose to do so via a GUI. You can input as many waypoints as you want to get to the targets. 

You will get **20pts for each target fruit the robot can reach**. The robot should be within 0.5m of the target and stop for 3s (timing does not need to be exact, as long as the robot stops) to indicate that it has found a target fruit before moving onto the next target. You will get 0pt for that target if it is not within 0.5m of the target. You should confirm with the demonstrator to check whether the robot is close enough to the target before moving on.

#### Level 2: Autonomous navigation with a known map (20pts)
For this level, the arena will have 10 ArUco markers and 3 types of target fruits. The locations of all the objects in the arena will be given to you in the [M4_true_map_3fruits.txt](M4_true_map_3fruits.txt) (you should have the file name as an input argument to run the script). You are only allowed to enter a **single command** and the robot should perform the task autonomously. 

You will get **8pts for reaching the first target fruit, and 6pts each for reaching the subsequent fruit**. The robot should be within 0.5m of the target and stop for 3s (timing does not need to be exact, as long as the robot stops) to indicate that it has found a target fruit before moving onto the next target. You will get 0pt for that target if it is not within 0.5m of the target.


#### Level 3: Autonomous navigation with a partially known map (20pts)
For this level, the arena will have 10 ArUco markers, 1 red apple, 1 yellow lemon, 1 pear, 1 orange and 1 strawberry. Only the locations of the 10 ArUco markers and 3 types of target fruit will be given to you in the [M4_true_map_3fruits.txt](M4_true_map_3fruits.txt) (you should have the file name as an input argument to run the script). The locations of the other 2 types of fruit will **not** be provided and you will have to rely on your fruit detection method to avoid colliding with them. You are only allowed to enter a **single command** and the robot should perform the task autonomously. 

You will get **8pts for reaching the first target fruit, and 6pts each for reaching the subsequent fruit**. The robot should be within 0.5m of the target and stop for 3s (timing does not need to be exact, as long as the robot stops) to indicate that it has found a target fruit before moving onto the next target. You will get 0pt for that target if it is not within 0.5m of the target.

**Note**: For simulation marking, you will be given the [M4_true_map_5fruits.txt](M4_true_map_5fruits.txt) map to spawn the objects in Gazebo, but you are not allowed to use this map as the input true map for your fruit searching script (Zero score if we find out you use this map for your fruit search algorithm). 

### Marking instructions
Please refer to these [marking instructions](M4_marking_instructions.md)

