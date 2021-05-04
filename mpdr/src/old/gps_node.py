#!/usr/bin/env python
import rospy
from sensor_msgs.msg import NavSatFix
import os
from gps import *
import time

def Gps_publisher():
    pub = rospy.Publisher('gps_info', NavSatFix, queue_size=10)
    rospy.init_node('gps',anonymous=True)
    rate = rospy.Rate(5)
    #while not rospy.is_shutdown():
    gps_messured = gps(mode=WATCH_ENABLE)
    while True:
        #This is where you form your message
        
        gps_info = NavSatFix()
        gps_info.latitude = gps_messured.fix.latitude
        gps_info.longitude = gps_messured.fix.longitude
        gps_info.altitude = gps_messured.fix.altitude
        pub.publish(gps_info)
        gps_messured.next()
        time.sleep(0.5)
        rate.sleep()

if __name__ == '__main__':
    try:
        Gps_publisher()
    except rospy.ROSInterruptException:
        pass
