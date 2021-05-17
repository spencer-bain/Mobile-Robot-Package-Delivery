from pubnub.pnconfiguration import PNConfiguration
from pubnub.pubnub import PubNub
from pubnub.exceptions import PubNubException

pnChannel = "raspi-tracker";

pnconfig = PNConfiguration()
pnconfig.publish_key = "pub-c-74e6b463-6ec6-4cea-9eb6-9f692a6506a0"
pnconfig.subscribe_key = "sub-c-efdc1f2e-2aa6-11eb-9713-12bae088af96"
pnconfig.ssl = False

pubnub = PubNub(pnconfig)
pubnub.subscribe().channels(pnChannel).execute()

def gpsReading_cb():
    # Get input from cmd line
    while True:
        print("Enter: ")
        userInput = input() 
        if(userInput == "start"):
            lat = 44.566213
            lng = -123.279874
            print("lat: ", lat)
            print("lng: ", lng)

        if(userInput == "mid"):
            lat = 44.566214
            lng = -123.278800
            print("lat: ", lat)
            print("lng: ", lng)

        if(userInput == "end"):
            lat = 44.566215
            lng = -123.277888
            print("lat: ", lat)
            print("lng: ", lng)

        if(userInput == "pick up"):
           lat = 44.566215
           lng = -123.277890
           print("Receiver has picked up the package!")

        try:
            envelope = pubnub.publish().channel(pnChannel).message({'lat':lat,'lng':lng}).sync()
            print("publish timetoken: %d" % envelope.result.timetoken)
        except PubNubException as e:
            handle_exception(e)

gpsReading_cb()
