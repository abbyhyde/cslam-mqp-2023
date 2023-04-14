#!/bin/bash
while getopts n: flag
do
    case "${flag}" in
        n) name=${OPTARG};;
    esac
done
sleep 1
sshpass -p "turtlebot" ssh ubuntu@$name.dyn.wpi.edu "roslaunch turtlebot3_bringup turtlebot3_robot_custom.launch multi_robot_name:='$name' set_lidar_frame_id:='$name/base_scan'"