# ECE4078_Lab_2022
More detailed steps in simulator_steps.txt and physical_steps.txt.

Summary:
- Run operateEst.py to map the SLAM and the fruits.
    - First map SLAM. Save SLAM map with 's'. Press 'spacebar', map fruits by taking images with 'i'.

- Run TargetPoseEst.py. Add --using_sim if on simulator.

- Can use mapping_eval.py to check if the estimations are good.

- Run slamconverter.py to convert to true map

- Run operate_level2.py --map output_map.txt --search_list search_list.txt for autonomous navigation

- Run auto_fruit_search.py --map output_map.txt for semi-autonomous navigation. This function does not have a search_list modifier so the search list has to be named 'search_list.txt'


