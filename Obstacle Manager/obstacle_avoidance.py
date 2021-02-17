#! /usr/bin/env python3

import rospy
import time

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point

collide_object = 0

# Reading Data from the LiDAR
def cb_lidar_reading(msg):
    regions ={
        'right': min(msg.ranges[270:306]),
        'front_right': min(msg.ranges[306:342]),
        'front': min(min(msg.ranges[342:359]),min(msg.ranges[0:18])),
        'front_left': min(msg.ranges[18:54]),
        'left': min(msg.ranges[54:90])
    }
    #Run "move_it" function 
    move_it(regions)

def cb_bump_reading(msg):
    global collide_object
    collide_object = msg.x

# This function is responsible to move the robot and avoid the obstacles
def move_it(regions):
    msg = Twist() # For Linear And Angular Speeds

    # Variables Initialization
    linear_x = 0  
    angular_z = 0

    linear_speed = 0.3
    angular_speed = 0.3

    current_state = ''
    red_zone = 0.5


    # If the bump sensor did detect any collision
    if not collide_object:
        # No Objects
        if regions['front_right'] > red_zone and regions['front'] > red_zone and regions['front_left'] > red_zone:
            current_state = 'case 1 - nothing'
            linear_x = linear_speed
            angular_z = 0

        # Object on right 
        elif regions['front_right'] < red_zone and regions['front'] > red_zone and regions['front_left'] > red_zone:
            current_state = 'case 2 - front_right'
            linear_x = 0
            angular_z = angular_speed

        # Object on front 
        elif regions['front_right'] > red_zone and regions['front'] < red_zone and regions['front_left'] > red_zone:
            current_state = 'case 3 - front'
            linear_x = 0
            angular_z = angular_speed

        # Object on left
        elif regions['front_right'] > red_zone and regions['front'] > red_zone and regions['front_left'] < red_zone:
            current_state = 'case 4 - front_left'
            linear_x = 0
            angular_z = -angular_speed
        # Object on front and right
        elif regions['front_right'] < red_zone and regions['front'] < red_zone and regions['front_left'] > red_zone:
            current_state = 'case 5 - front_right & front'
            linear_x = 0
            angular_z = angular_speed

        # Object on front and left
        elif regions['front_right'] > red_zone and regions['front'] < red_zone and regions['front_left'] < red_zone:
            current_state = 'case 6 - front & front_left'
            linear_x = 0
            angular_z = -angular_speed

        # Object on right and left
        elif regions['front_right'] < red_zone and regions['front'] > red_zone and regions['front_left'] < red_zone:
            current_state = 'case 7 - front_right & front_left'
            linear_x = linear_speed
            angular_z = 0

        # Object on right and left
        elif regions['front_right'] < red_zone and regions['front'] < red_zone and regions['front_left'] < red_zone:
            current_state = 'case 8 - ront_right & front & front_left'
            linear_x = 0
            angular_z = angular_speed

        # Other Cases
        else:
            current_state = "Unknow Case"
            linear_speed = 0
            angular_speed = 0
            rospy.loginfo(regions)
    # When the bump sensor collide something        
    else:
        current_state = 'case 9 - collide_object'
        linear_speed = 0
        angular_speed = 0


    rospy.loginfo(current_state) # Print the current state
    msg.linear.x = linear_x # Set msg new linear speed
    msg.angular.z = angular_z # Set msg new angualr speed
    pub.publish(msg) # Publish the new speeds to the /cmd_vel


def main():

    # Node Initialization
    rospy.init_node('obstacle_avoidance')

    # Subscribers
    sub = rospy.Subscriber('/scan', LaserScan, cb_lidar_reading)
    sub = rospy.Subscriber('/bump', Point, cb_bump_reading)

    # Publishers
    global pub
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    # ROSPY SPIN !
    rospy.spin()


if __name__== '__main__':
    main()
