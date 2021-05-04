#!/usr/bin/env python
'''
Genral Notes
This node isn't subed to bump sensor, only the obstical manager is. That data needs to get
passed through the obstical manager or we need to be subbed to it. In this code I have 
subbed the movement manager to the /bump topic.
'''

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point


FALLOW_PATH      = 0
STOP             = 1
SLOW_FALLOW_PATH = 2
AVOID            = 3
IDLE             = 4
MANUAL           = 5
DELIVERY         = 6

class movement_manager(object):
    def __init__(self):
        self.priority = STOP
        
        self.website_override = MANUAL #Weather the robot is in manual or delivery mode
                                       #chosen by the user

        self.path_sub = rospy.Subscriber('/move', Twist, self.path_callback)
        self.obstical_sub = rospy.Subscriber('/action', Twist, self.obstical_callback)
        self.manual_sub = rospy.Subscriber('/man_cmd', Twist, self.manual_callback)
        self.bump_sub = rospy.Subscriber('/bump', Point, self.bump_callback)

        idle_handler()


    def path_callback(self,data):
        if(self.priority == FALLOW_PATH): 
            #TODO set wheels to move in acordance to the the /move twist command
            pass
        elif(self.priority == SLOW_FALLOW_PATH):
            #TODO set wheels to move in acordance to the the /move twist command
            #     but slower
            pass
    
    def obstical_callback(self,data):
        if(self.priority == AVOID):
            #TODO set wheels to move in acordance to the the /action twist command
            pass

    def manual_callback(self,data):
        if(self.priority == MANUAL):
            #TODO set wheels to move in acordance to the the /man_cmd twist command
            pass

    def bump_callback(self, data):
        if(data.x == 1):
            self.priority = STOP

    def idle_handler(self):
        if(self.website_override == MANUAL):
            self.priority = MANUAL
        elif(self.wesite_override == DELIVERY):
            self.priority = FALLOW_PATH

if __name__ == "__main___":
    try:
        movement_manager()
    except:
        pass
