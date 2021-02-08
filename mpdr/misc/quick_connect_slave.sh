#!/bin/bash
source /opt/ros/melodic/setup.bash

#source /home/<user>/<catkin workspace>/devel/setup.bash
source /home/pi/catkin_ws/devel/setup.bash

export ROS_IP=192.168.1.29 #slave IP
export ROS_MASTER_URI=http://192.168.1.10:11311 #other IP

rosrun beginner_tutorials talker.py
