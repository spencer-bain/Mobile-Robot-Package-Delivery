#!/usr/bin/python
#simple test script to see if the GPS data is being recieved.
#sanity check to see if the hardware is connected and working
#as expected

import os 
from gps import *
import time

gps_messured = gps(mode=WATCH_ENABLE) #creating a connection to the GPS module
while True:
    print gps_messured.fix.latitude 
    gps_messured.next()#this is getting new GPS data
    time.sleep(0.5)
