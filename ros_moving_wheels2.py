#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Point
from time import sleep
import RPi.GPIO as GPIO

TEST_ANGULAR_VELOCITY = 0.001 #should get 30% duty cycle, really slow wheel movement

WHEEL_RADIUS = 0.1016#meters
ROBOT_RADIUS = 0.2794#meters
ROBOT_WIDTH  = 2*ROBOT_RADIUS


ANGULAR_MAX = 6.4#rad/secod

pwm_bounds = (-100,-6,0,6,100)#piece-wise duty cycles
#linear_vel_bounds = (-1.7885,-0.001,0,0.001,1.7885)#linear velocit meters per second
linear_vel_bounds = (-1.7885,-0.15956,0,0.15956,1.7885)#linear velocit meters per second
angluar_vel_bounds = (-6.4,6.4)

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

def status2useful(status):
    #status = (duty_cycle_L, duty_cycle_R, dir_L, dir_R)
    function = linear_transform((pwm_bounds[3],pwm_bounds[4]),(linear_vel_bounds[3],linear_vel_bounds[4]))

    wheel_vel_L = function(status[0])
    wheel_vel_R = function(status[1])

    if status[2] == 1:
        wheel_vel_L = -wheel_vel_L
    if status[3] == 0:
        wheel_vel_R = -wheel_vel_R
    print("")
    print("Left Wheel's: Velocity (m/s) " + str(wheel_vel_L) + " Duty Cycle (%)" + str(status[0]))
    print("Right Wheel's: Velocity (m/s) " + str(wheel_vel_R) + " DUty Cycle (%)" + str(status[1]))
    print("")



class MotorDriver:
    def __init__ (self):
        self.dirPin_R = 17      #GPIO 17, INT1
        self.dirPin_L = 27      #GPIO 27, INT2
        self.speedPin_L = 12    #GPIO 12 (PWM0), AIN2
        self.speedPin_R = 13    #GPIO 13 (PWM1), AIN1

        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)

        GPIO.setup(self.dirPin_R, GPIO.OUT)
        GPIO.setup(self.dirPin_L, GPIO.OUT)
        GPIO.setup(self.speedPin_R, GPIO.OUT)
        GPIO.setup(self.speedPin_L, GPIO.OUT)

        self.piPWM_L = GPIO.PWM(self.speedPin_L, 1000)    #Creates PWM instance with frequency
        self.piPWM_R = GPIO.PWM(self.speedPin_R, 1000)   

        self.dir_L = 0 # 0-CCW, 1-CW
        self.dir_R = 0 # 0-CCW, 1-CW
        
        self.motorInit()

        self.duty_cycle_L = 0 #0-100 => 0-4mph
        self.duty_cycle_R = 0 #0-100 => 0-4mph
        self.linear2pwm = linear_transform((linear_vel_bounds[3],linear_vel_bounds[4]),(pwm_bounds[3],pwm_bounds[4]))

    def motorInit(self):
        self.piPWM_L.start(0)
        self.piPWM_R.start(0)


    def status(self):
        data = []
        data.append(self.duty_cycle_L)
        data.append(self.duty_cycle_R)
        data.append(self.dir_L)
        data.append(self.dir_R)
        return data

    def set_linear_speed(self,lin_vel):
        if lin_vel >= 0:#Forward
            self.dir_L = 1
            self.dir_R = 0
            GPIO.output(self.dirPin_L, self.dir_L)
            GPIO.output(self.dirPin_R, self.dir_R)
        else:#back
            self.dir_L = 0
            self.dir_R = 1
            GPIO.output(self.dirPin_L, self.dir_L)
            GPIO.output(self.dirPin_R, self.dir_R)

        self.duty_cycle_L = self.linear2pwm(abs(lin_vel))
        self.duty_cycle_R = self.linear2pwm(abs(lin_vel))
        self.piPWM_L.ChangeDutyCycle(self.duty_cycle_L)
        self.piPWM_R.ChangeDutyCycle(self.duty_cycle_R)

        sleep(2)
        self.piPWM_L.ChangeDutyCycle(0)
        self.piPWM_R.ChangeDutyCycle(0)

    def set_wheel_vel_R(self,lin_vel):
        if lin_vel >= 0:#forward
            self.dir_R = 0
        else:#background
            self.dir_R = 1
        if lin_vel != 0:
            self.duty_cycle_R = self.linear2pwm(abs(lin_vel))
        GPIO.output(self.dirPin_R, self.dir_R)
        self.piPWM_R.ChangeDutyCycle(self.duty_cycle_R)

    def set_wheel_vel_L(self,lin_vel):
        if lin_vel >= 0:#forward
            self.dir_L = 1
        else:#background
            self.dir_L = 0
        if lin_vel != 0:
            self.duty_cycle_L = self.linear2pwm(abs(lin_vel))
        GPIO.output(self.dirPin_L, self.dir_L)
        self.piPWM_L.ChangeDutyCycle(self.duty_cycle_L)

    def twist_callback(self,data):
        #getting wheel velocity
        left_speed_out = data.linear.x - data.angular.z*ROBOT_WIDTH/2
        right_speed_out = data.linear.x + data.angular.z*ROBOT_WIDTH/2

        #capping velocity
        if left_speed_out > 1.7:
            left_speed_out = 1.7
        elif left_speed_out < -1.7:
            left_speed_out = -1.7

        if right_speed_out > 1.7:
            right_speed_out = 1.7
        elif right_speed_out < -1.7:
            right_speed_out = -1.7

        if left_speed_out < linear_vel_bounds[3] and left_speed_out > linear_vel_bounds[1]:
            left_speed_out = 0
        if right_speed_out < linear_vel_bounds[3] and right_speed_out > linear_vel_bounds[1]:
            right_speed_out = 0

        self.set_wheel_vel_L(left_speed_out)
        self.set_wheel_vel_R(right_speed_out)
        sleep(1)
        self.piPWM_L.ChangeDutyCycle(0)
        self.piPWM_R.ChangeDutyCycle(0)


        info = self.status()
        print("Linear Speed: " + str(data.linear.x) + " (m/s), Angular Speed: " + str(data.angular.z) + " (rad/s)")
        status2useful(info)




def listener():

    motor_driver = MotorDriver()
    #both wheels moving
    #motor_driver.set_linear_speed(TEST_ANGULAR_VELOCITY)

    #one wheel at a time
    #motor_driver.set_wheel_vel_L(TEST_ANGULAR_VELOCITY)
    #motor_driver.set_wheel_vel_R(TEST_ANGULAR_VELOCITY)

    rospy.init_node('twist_listener', anonymous=True)
    twits_sub = rospy.Subscriber('/cmd_vel', Twist, motor_driver.twist_callback)
    data = motor_driver.status()
    print(data)
    rospy.spin()


if __name__ == "__main__":
    try:
        listener()
    except:
        pass

