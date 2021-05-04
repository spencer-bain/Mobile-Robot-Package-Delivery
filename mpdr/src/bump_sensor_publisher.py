#!/usr/bin/env python3

import math
import rospy
from gpiozero import Button
from geometry_msgs.msg import Point
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

def bump_publisher():
    pub = rospy.Publisher('bump',Point, queue_size=10)
    rospy.init_node('bump_sensor', anonymous=True)
    rate = rospy.Rate(10)
    #button4 = Button(26, pull_up=False)
    #button = Button(17, pull_up=False, hold_time=0.1)
    #button2 = Button(27, pull_up=False, hold_time=0.1)
    GPIO.setup(17,GPIO.IN,GPIO.PUD_DOWN)
    GPIO.setup(27,GPIO.IN,GPIO.PUD_DOWN)
    while not rospy.is_shutdown():
        data = Point()

        if GPIO.input(17) or GPIO.input(27):
            data.x = 1
        else:
            data.x = 0

        pub.publish(data)
        rate.sleep()

if __name__ == '__main__':
    try:
        bump_publisher()
    except rospy.ROSInterruptException:
        pass


