#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Point
from time import sleep
import RPi.GPIO as GPIO
import serial


MAX_SPEED = 0.5 #Meter per second

# **** Robot info in meters ****
WHEEL_RADIUS = 0.1016
ROBOT_RADIUS = 0.2794 
ROBOT_WIDTH  = 2*ROBOT_RADIUS

# **** Angular speeds for the wheels ****
ANGULAR_MAX = 6.4             # Rad/secod
#TEST_ANGULAR_VELOCITY = 0.001 # Should get 30% duty cycle, really slow wheel movement
#TEST_ANGULAR_VELOCITY = 0.15956 # Should get 30% duty cycle, really slow wheel movement
TEST_ANGULAR_VELOCITY = 0.2 # Should get 30% duty cycle, really slow wheel movement

# **** Value bounds for calculations ****
# Max, min and 0 value to turn(and not turn) the motors)
serial_bounds = (-63,-6,0,6,63) 
linear_vel_bounds = (-1.7885,-0.15956,0,0.15956,1.7885) # m/s
angluar_vel_bounds = (-6.4,6.4)                         # rad/s


# Commented out from Spencer's version (was already commented out
#linear_vel_bounds = (-1.7885,-0.001,0,0.001,1.7885)#linear velocit meters per second


# **** Transforms one number range to another ****
def linear_transform(DOMAIN,RANGE,debug=0):
    scale = (RANGE[1]-RANGE[0])/(DOMAIN[1]-DOMAIN[0])
    DOMAIN_SHIFT = DOMAIN[0]+(DOMAIN[1]-DOMAIN[0])/2
    RANGE_SHIFT = RANGE[0]+(RANGE[1]-RANGE[0])/2

    function = lambda x:scale*(x - DOMAIN_SHIFT) + RANGE_SHIFT
    if debug != 0:
        print("scale: " + str(scale))
        print("RANGE_SHIFT: " + str(RANGE_SHIFT))
        print("DOMAIN_SHIFT: " + str(DOMAIN_SHIFT))
    return function


class MotorDriver:
    def __init__ (self):
        
        # Status variables defaults
        self.dirPin_R = 0       # 0-CW, 1-CCW
        self.dirPin_L = 0       # 0-CW, 1-CCW
        self.speedPin_L = 0     # serial speed, 0-63
        self.speedPin_R = 0     # serial speed, 0-63

        # Set up linear mapping from linear_vel to serial commands values
        self.linear2serial = linear_transform((linear_vel_bounds[3],linear_vel_bounds[4]),
                                              (serial_bounds[3],serial_bounds[4]))
        self.ser = serial.Serial("/dev/ttyAMA1", baudrate = 9600, parity=serial.PARITY_NONE, 
                             stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS)
                             
        # Emergency stop start as false
        self.stop = 0
  
        # Setting up ROS publishers and subcribers
        self.serial_pub = rospy.Publisher('serial_speed', String, queue_size=10)
        self.twist_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        self.bump_sub  = rospy.Subscriber('/bump', Point, self.bump_callback)
    
    # Generates and sends a 8 bit cmd message to the motor controller via serial
    # Bit 7 = channel (0-left, 1-right)
    # Bit 6 = direction (0-CW, 1-CCW)
    # Bits 5-0 = speed (0b000000 = 0 stop, 0b111111 = 63 full speed)
    def wheelVeltoSerial(self, wheel_vel, wheel):
        
        # Starting to build the serial command message
        if wheel == "left":          # left wheel
            if wheel_vel >= 0:      # CW
                chan_dir = 0;       # channel=left(0 << 7), dir=CW(0 << 6)
            elif wheel_vel < 0:     # CCW
                chan_dir = 0x40     # channel=left(0 << 7), dir=CCW(1 << 6)

        elif wheel == "right":       # right wheel
            if wheel_vel >= 0:      # CCW
                chan_dir = 0xC0     # channel=right(1 << 7), dir=CCW(1 << 6)
            elif wheel_vel < 0:     # CW
                chan_dir = 0x80     # channel=right(1 << 7), dir=CW(0 << 6)
        
        # Maps the actual wheel velocity to a speed value for the motor controller
        wheel_speed = self.linear2serial(abs(wheel_vel))

        # Finishes building serial command message and sends
        serialMsg = chan_dir + int(wheel_speed)
        self.serial_pub.publish(str(serialMsg))
        self.ser.write(bytes([serialMsg]))

        # Debugging
        #print(str(wheel) + " " + str(chan_dir) + " " + str(int(wheel_speed)) + " " + str(serialMsg))


    # Converts cmd_vel twist commands to wheel velocities
    def cmd_vel_callback(self,data):
        # Check if bumpers have been hit, if not - continue
        if self.stop == 0:
            # Calculating individual wheel velocity
            wheel_vel_L = data.linear.x - data.angular.z*ROBOT_WIDTH/2
            wheel_vel_R = data.linear.x + data.angular.z*ROBOT_WIDTH/2

            # Debug
            print("Raw init wheel_vel_L: ", wheel_vel_L)
            print("Raw init wheel_vel_R: ", wheel_vel_R)

            # Capping velocity for max CW and CCW values
            if wheel_vel_L > MAX_SPEED:
                wheel_vel_L = MAX_SPEED
            elif wheel_vel_L < -MAX_SPEED:
                wheel_vel_L = -MAX_SPEED

            if wheel_vel_R > MAX_SPEED:
                wheel_vel_R = MAX_SPEED
            elif wheel_vel_R < -MAX_SPEED:
                wheel_vel_R = -MAX_SPEED
            
            # Make sure that the linear2serial function takes care of the "deadzone".
            # The lowerbound for the linear mapping is 0.159 m/s which is mapped to 6% 
            # duty cycle. The two line below takes care of if we send a speed
            # less than 0.159(meter/sec), because we can't reach those speeds with our current
            # implimentation. linear_vel_bounds[3] = 0.159, linear_vel_bounds[1] = -0.159 
            if abs(wheel_vel_L) < linear_vel_bounds[3] and abs(wheel_vel_L > linear_vel_bounds[1]):
                wheel_vel_L = 0
            if abs(wheel_vel_R) < linear_vel_bounds[3] and abs(wheel_vel_R > linear_vel_bounds[1]):
                wheel_vel_R = 0

            # Convert wheel velocities to serial commands and sent to motor controller
            self.wheelVeltoSerial(wheel_vel_L, "left")
            self.wheelVeltoSerial(wheel_vel_R, "right")

    # Bumper handler
    def bump_callback(self,data):
        if data.x == 1.0:
            self.stop = 1
            print("Bumper have been HIT")
            self.wheelVeltoSerial(0, "left")
            self.wheelVeltoSerial(0, "right")


if __name__ == "__main__":
    
    try:
        rospy.init_node('motor_controller')
        MotorDriver()
        rospy.spin()

    except:
        print("Failed to run motor controller script")

