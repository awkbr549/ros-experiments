#! /bin/bash

source ~/.bashrc
rkill
rstart
roslaunch set_amclinitpose comparison_sensormodels.launch &
rqt_bag &
sleep 15
roslaunch move_straight moveRobot.launch &
sleep 30
kill -INT $(pgrep rqt)
sleep 1
kill -INT $(pgrep rviz)
kill -INT $(pgrep stageros)
kill -INT $(pgrep roslaunch)
sleep 3
echo "success"
clear
