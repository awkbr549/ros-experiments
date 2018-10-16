#! /bin/bash
clear
sleep 1
catkin_make
sleep 1
roslaunch navigation_stage move_base_gmapping_5cm.launch &
sleep 5
roslaunch explore_map explore_map_node.launch 
sleep 1
clear
