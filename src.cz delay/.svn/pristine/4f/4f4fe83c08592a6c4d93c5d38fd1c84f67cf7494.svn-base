#!/bin/bash
source /opt/ros/kinetic/setup.bash
sleep 2
source /home/nvidia/catkin_ws/devel/setup.bash
#source ~/robot4/devel/setup.bash
rosclean purge -y
sleep 2
roslaunch srbsmartcar_description smartcar_amcl.launch #>> ~/control_2019_01_22.log
#roslaunch srbsmartcar_description smartcar_amcl.launch >> control_01.log
#roslaunch robot_driver srbot_chassis_driver.launch
