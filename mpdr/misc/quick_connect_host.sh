#!/bin/bash
source /opt/ros/melodic/setup.bash

#source /home/<username>/catkin_ws/devel/setup.bash
source /home/spencer/catkin_ws/devel/setup.bash

export ROS_IP=192.168.1.10 #This is your host computer, where roscore is going to be running
export ROS_MASTER_URI=http://192.168.1.10:11311

roscore&
rosrun beginner_tutorials listener.py
