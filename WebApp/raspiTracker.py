# before testing, run:
# pip install 'pubnub>=4.1.4'
import rospy
import serial
import time
import string
import pynmea2
from pubnub.pnconfiguration import PNConfiguration
from pubnub.pubnub import PubNub
from pubnub.exceptions import PubNubException
from sensor_msgs.msg import NavSatFix.msg


# Connect raspi to pubnub
pnChannel = "raspi-tracker";

pnconfig = PNConfiguration()

# Keys
pnconfig.publish_key = "pub-c-74e6b463-6ec6-4cea-9eb6-9f692a6506a0"
pnconfig.subscribe_key = "sub-c-efdc1f2e-2aa6-11eb-9713-12bae088af96"
pnconfig.ssl = False

pubnub = PubNub(pnconfig)
pubnub.subscribe().channels(pnChannel).execute()
# 
# def gpsReading_cb(msg):
#     print(msg.latitude)
#     print(msg.longitude)
#     print(msg.altitude)

# Subscribe
while True:
    # rospy.init_node('gps_reader')
    # gpsSub = rospy.Subscriber('/..../gps', NavSatFix, gpsReading_cb)
    # rospy.spin()

    port="/dev/ttyAMA0"
    ser=serial.Serial(port, baudrate=9600, timeout=0.5)
    dataout = pynmea2.NMEAStreamReader()
    newdata=ser.readline()

    if newdata[0:6] == "$GPRMC":
        # Parse GPS data
        # Assign latitude and longitude to variables
        newCoords = pynmea2.parse(newdata)
        lat = newCoords.latitude
        lng = newCoords.longitude
        try:
            # Push variables to pubnub
            envelope = pubnub.publish().channel(pnChannel).message({
            'lat':lat,
            'lng':lng
            }).sync()
            print("publish timetoken: %d" % envelope.result.timetoken)
        except PubNubException as e:
            handle_exception(e)
