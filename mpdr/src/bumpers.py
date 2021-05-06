#!/usr/bin/env python3

import math
import rospy
from gpiozero import Button
from geometry_msgs.msg import Point
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

class Bumper:
    def __init__(self):
    
        # Set up GPIO pins
        GPIO.setup(17,GPIO.IN,GPIO.PUD_DOWN)
        GPIO.setup(27,GPIO.IN,GPIO.PUD_DOWN)

        self.bumper_pub = rospy.Publisher('bump',Point, queue_size=10)
        self.rate = rospy.Rate(10)
        self.bumpStatus = Point()

    def get_bump(self):
        while not rospy.is_shutdown():
            # Bump sensor activated (we've been hit)
            if GPIO.input(17) or GPIO.input(27):
                self.bumpStatus.x = 1
            # Bump sensor not activated
            else:
                self.bumpStatus.x = 0

            self.bumper_pub.publish(self.bumpStatus)
            self.rate.sleep()
        

if __name__ == '__main__':
    try:
        rospy.init_node('bumper')
        bumper = Bumper()
        bumper.get_bump()
        
    except:
        print("Failed to run bumper script")


