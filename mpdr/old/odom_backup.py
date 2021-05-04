#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
import spidev
import tf
import time
import math
from math import sin, cos, pi
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

WHEEL_RADIUS = 0.1016#meters
ROBOT_RADIUS = 0.2794#meters

DistancePerCount = (3.14159265 * ROBOT_RADIUS) / 16383; 

def encoder_node():
    #TODO
    #initiate node
    rospy.init_node('encoders', anonymous=True)
    pub = rospy.Publisher('odom',Odometry, queue_size=50)
    r = rospy.Rate(100)
    odom_broadcaster = tf.TransformBroadcaster()

    #initial spi
    lw_spi = spidev.SpiDev()
    lw_spi.open(0,0)    #headerpin 7
    lw_spi.max_speed_hz = 5000
    lw_spi.mode = 0b01

    rw_spi = spidev.SpiDev()
    rw_spi.open(0,1) #headerpin 8
    rw_spi.max_speed_hz = 5000
    rw_spi.mode = 0b01

    data2degree = linear_transform((0,16383),(0,360))

    x = 0.0
    y = 0.0
    th = 0.0

    #These got to be set to something
    vx = 0.1
    vy = -0.1
    vth = 0.1

    _PreviousLeftEncoderCounts = 0
    _PreviousRightEncoderCounts = 0
    
    current_time = rospy.Time.now()
    last_time = rospy.Time.now()

    list_of_bytes = [0xFF,0xFF]
    i = 0

    while not rospy.is_shutdown():
        current_time = rospy.Time.now()
        
        something_r = rw_spi.xfer2(list_of_bytes,2)
        byte_data_r = rw_spi.readbytes(2)
        data_r = get_data(byte_data_r)
        
        something_l = lw_spi.xfer2(list_of_bytes,2)
        byte_data_l = lw_spi.readbytes(2)
        data_l = get_data(byte_data_l)

        tick_x = _PreviousLeftEncoderCounts - data_l
        tick_y = _PreviousRightEncoderCounts - data_r
        
        #https://answers.ros.org/question/241602/get-odometry-from-wheels-encoders/
        deltaLeft = tick_x - _PreviousLeftEncoderCounts
        deltaRight = tick_y - _PreviousRightEncoderCounts

        v_left = (deltaLeft * DistancePerCount) / (current_time - last_time).to_sec()
        v_right = (deltaRight * DistancePerCount) / (current_time - last_time).to_sec()

        #v_left = omega_left * WHEEL_RADIUS 
        #v_right = omega_right * WHEEL_RADIUS

        vx = ((v_right + v_left) / 2)
        vy = 0;
        vth = ((v_right - v_left)/ROBOT_RADIUS)
        


        # compute odometry in a typical way given the velocities of the robot
        dt = (current_time - last_time).to_sec()
        delta_x = (vx * cos(th) - vy * sin(th)) * dt
        delta_y = (vx * sin(th) + vy * cos(th)) * dt
        delta_th = vth * dt

        x += delta_x
        y += delta_y
        th += delta_th

        # since all odometry is 6DOF we'll need a quaternion created from yaw
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)

        # first, we'll publish the transform over tf
        odom_broadcaster.sendTransform(
            (x, y, 0.),
            odom_quat,
            current_time,
            "base_link",
            "odom"
        )

        # next, we'll publish the odometry message over ROS
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"

        # set the position
        odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))

        # set the velocity
        odom.child_frame_id = "base_link"
        odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))

        # publish the message
        pub.publish(odom)
        
        _PreviousLeftEncoderCounts = tick_x
        _PreviousRightEncoderCounts = tick_y

        last_time = current_time
        r.sleep()

        pass


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

def main():
    lw_spi = spidev.SpiDev()
    lw_spi.open(0,0)    #headerpin 7
    lw_spi.max_speed_hz = 5000
    lw_spi.mode = 0b01

    rw_spi = spidev.SpiDev()
    rw_spi.open(0,1) #headerpin 8
    rw_spi.max_speed_hz = 5000
    rw_spi.mode = 0b01

    data2degree = linear_transform((0,16383),(0,360))
    #want to write to adress 0xFF
    #13:0 address to read/write
    #14   0->write, 1->read
    #15   even parity. 0-> sum of ones is even
    #                  1-> sum of ones is odd

    #address to read 0x3FFF
    #reading -> 1
    # 2 + 4 + 4 + 4 + 1 = 15 -> odd -> parity bit = 1

    #xfer3 sends 2 bytes across MOSI the pulses CS/SS

    list_of_bytes = [0xFF,0xFF]
    i = 0
    while True:
        i += 1
        something_r = rw_spi.xfer3(list_of_bytes,2)
        byte_data_r = rw_spi.readbytes(2)
        data_r = get_data(byte_data_r)
        
        something_l = lw_spi.xfer3(list_of_bytes,2)
        byte_data_l = lw_spi.readbytes(2)
        data_l = get_data(byte_data_l)

        if data_r != 0x80:
            print("R " + str(i + 1) + ": " + str(data2degree(data_r)) + "   " + str(hex(byte_data_r[0])) + ", " + str(hex(byte_data_r[1])) + "  " + str(bin(byte_data_r[0])) + ", " + str(bin(byte_data_r[1])))
        
        if data_l != 0x80:
            print("L " + str(i + 1) + ": " + str(data2degree(data_l)) + "   " + str(hex(byte_data_l[0])) + ", " + str(hex(byte_data_l[1])) + "  " + str(bin(byte_data_l[0])) + ", " + str(bin(byte_data_l[1])))
        
        time.sleep(1)

    rw_spi.close()
    lw_spi.close()


def get_error_bit(byte):
    error_bit = (~(0b10111111) & byte) >> 6
    if error_bit == 1:#error!!
        return False  #There was an error in communication
    else:#
        return True

def get_pard_bit(byte):
    pard_bit = (~(0b10111111) & byte) >> 7

    SUM = 0
    for i in range(7):
        bit_check = (byte >> i) & 0x00000001
        if bit_check == 1:
            SUM += 1

    pard_bit = (~(0b01111111) & byte) >> 7
    if pard_bit == 0 and SUM % 2 == 0:
        return True
    elif pard_bit == 1 and SUM % 2 != 0:
        return True
    else:
        return False

def get_data(list_of_2bytes):
    #list_of_2bytes = [MSB,LSB]
    if not (get_pard_bit(list_of_2bytes[0]) or get_error_bit(list_of_2bytes[0])):
        return 0x80
    else:
        data = ~(0b11000000) & list_of_2bytes[0]
        data = (data << 8) + list_of_2bytes[1]
        return data

    
def reverse_byte(data,size=8):
    actual_data = 0x0
    #print(int(size))
    for i in range(int(size/2)):
        actual_data += (data & (0b10000000 >> i)) >> (2*(3-i)+1)
    for i in range(int(size/2)): 
        actual_data += (data & (0b00001000 >> i)) << (2*i+1)

    return actual_data

if __name__ == "__main__":
#    main()
    encoder_node()
