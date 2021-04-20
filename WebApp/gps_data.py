#! /usr/bin/env python3
# pip install 'pubnub>='4.1.4'

import rospy
from sensor_msgs.msg import NavSatFix

from pubnub.pnconfiguration import PNConfiguration
from pubnub.pubnub import PubNub
from pubnub.exceptions import PubNubException

# Connect to Raspi to pubnub
pnChannel = "raspi-tracker";

pnconfig = PNConfiguration()

# Keys
pnconfig.publish_key = "pub-c-74e6b463-6ec6-4cea-9eb6-9f692a6506a0"
pnconfig.subscribe_key = "sub-c-efdc1f2e-2aa6-11eb-9713-12bae088af96"
pnconfig.ssl = False

pubnub = PubNub(pnconfig)
pubnub.subscribe().channels(pnChannel).execute()

def gpsReading_cb(msg):
        print(msg.latitude)
        print(msg.longitude)
        print(msg.altitude)

        lat = msg.latitude
        lng = msg.longitude

        try:
            #push variables to pubnub
            envelope = pubnub.publish().channel(pnChannel).message({ 'lat':lat,'lng':lng}).sync()
            print("publish timetoken: %d" % envelope.result.timetoken)
        except PubNubException as e:
            handle_exception(e)

rospy.init_node('gps_reader')
gpsSub = rospy.Subscriber('/ublox/fix', NavSatFix, gpsReading_cb)
rospy.spin()
