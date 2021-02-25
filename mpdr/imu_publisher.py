#!/usr/bin/env python3
from sensor_msgs.msg import Imu
import time
import board
import busio
#from adafruit_bno08x import BNO_REPORT_ACCELEROMETER BNO_REPORT_GYROSCOPE BNO_REPORT_MAGNETOMETER BNO_REPORT_ROTATION_VECTOR
from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_MAGNETOMETER,
    BNO_REPORT_ROTATION_VECTOR,
)
#from adafruit_bno08x import *
import rospy
from sensor_msgs.msg import Imu
from adafruit_bno08x.i2c import BNO08X_I2C

def imu_publisher():
    pub = rospy.Publisher('chatter',Imu, queue_size=10)
    rospy.init_node('Hanna_is_pretty_cool', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)
    bno = BNO08X_I2C(i2c)

    bno.enable_feature(BNO_REPORT_ACCELEROMETER)
    bno.enable_feature(BNO_REPORT_GYROSCOPE)
    bno.enable_feature(BNO_REPORT_MAGNETOMETER)
    bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
    while not rospy.is_shutdown():
        try:
            imu_data = Imu()

            accel_x, accel_y, accel_z = bno.acceleration  # pylint:disable=no-member
            imu_data.orientation_covariance[0] = accel_x
            imu_data.orientation_covariance[1] = accel_y
            imu_data.orientation_covariance[2] = accel_z

            gyro_x, gyro_y, gyro_z = bno.gyro  # pylint:disable=no-member
            imu_data.angular_velocity_covariance[0] = gyro_x
            imu_data.angular_velocity_covariance[1] = gyro_y
            imu_data.angular_velocity_covariance[2] = gyro_z

            mag_x, mag_y, mag_z = bno.magnetic  # pylint:disable=no-member
            imu_data.linear_acceleration_covariance[0] = mag_x
            imu_data.linear_acceleration_covariance[1] = mag_y
            imu_data.linear_acceleration_covariance[2] = mag_z

            quat_i, quat_j, quat_k, quat_real = bno.quaternion  # pylint:disable=no-member
            pub.publish(imu_data)
        except:
            pass

        rate.sleep()

if __name__ == '__main__':
    imu_publisher()
