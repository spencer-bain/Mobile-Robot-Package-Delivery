#!/usr/bin/env python
import rospy
from sensor_msgs.msg import NavSatFix

def gps_callback(data):
    print 'Latitude: ', data.latitude
    print 'Longitude: ', data.longitude
    print 'Altitude: ', data.altitude
    print

def gps_listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('gps_info',NavSatFix,gps_callback)
    rospy.spin()

if __name__ == '__main__':
    gps_listener()

