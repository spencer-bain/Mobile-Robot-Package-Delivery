#! /usr/bin/env python3
# pip install 'pubnub>='4.1.4'

import rospy
from sensor_msgs.msg import MagneticField
from sensor_msgs.msg import Imu

from pubnub.pnconfiguration import PNConfiguration
from pubnub.pubnub import PubNub
from pubnub.exceptions import PubNubException

# Connect to Raspi to pubnub
pnChannel = "imu_data";

pnconfig = PNConfiguration()

# Keys
pnconfig.publish_key = "pub-c-74e6b463-6ec6-4cea-9eb6-9f692a6506a0"
pnconfig.subscribe_key = "sub-c-efdc1f2e-2aa6-11eb-9713-12bae088af96"
pnconfig.ssl = False

pubnub = PubNub(pnconfig)
pubnub.subscribe().channels(pnChannel).execute()

def mag_cb(msg):

    mag_x = msg.magnetic_field.x
    mag_y = msg.magnetic_field.y
    mag_z = msg.magnetic_field.z

    #print("Mag x", mag_x)
    #print("Mag y", mag_y)
    #print("Mag z", mag_z)

    try:
        #push variables to pubnub
        envelope = pubnub.publish().channel(pnChannel).message({'mag_x':mag_x,'mag_y':mag_y,'mag_z':mag_z}).sync()
        print("publish timeout: %d" % envelope.result.timetoken)
    except PubNubException as e:
        handle_exception(e)

def raw_cb(msg):

    ang_x = msg.angular_velocity.x
    ang_y = msg.angular_velocity.y
    ang_z = msg.angular_velocity.z

    linear_x = msg.linear_acceleration.x
    linear_y = msg.linear_acceleration.y
    linear_z = msg.linear_acceleration.z

    ori_x = msg.orientation.x
    ori_y = msg.orientation.y
    ori_z = msg.orientation.z
    ori_w = msg.orientation.w

    #print("Ang x:", ang_x)
    #print("Ang y:", ang_y)
    #print("Ang z:", ang_z)

    #print("linear x:", linear_x)
    #print("linear y:", linear_y)
    #print("linear z:", linear_z)

    #print("ori x:", ori_x)
    #print("ori y:", ori_y)
    #print("ori z:", ori_z)
    #print("ori w:", ori_w)

    try:
        #push variables to pubnub
        envelope = pubnub.publish().channel(pnChannel).message({'ang_x':ang_x,'ang_y':ang_y,'ang_z':ang_z,'linear_x':linear_x,'linear_y':linear_y,'linear_z':linear_z,'ori_x':ori_x,'ori_y':ori_y,'ori_z':ori_z,'ori_w':ori_w}).sync()
        print("publish timeout: %d" % envelope.result.timetoken)
    except PubNubException as e:
        handle_exception(e)



rospy.init_node('imu_tracker')
gpsSub = rospy.Subscriber('/bno08x/raw', Imu, raw_cb)
gpsSub = rospy.Subscriber('/bno08x/mag', MagneticField, mag_cb)
rospy.spin()
