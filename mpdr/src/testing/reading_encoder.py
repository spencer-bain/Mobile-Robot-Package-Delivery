#!/usr/bin/env python3

#import rospy
#from nav_msgs.msg import Odometry
import spidev
import time


def encoder_node():
    #TODO
    #initiate node
    pub = rospy.Publisher('odom',Odometry, queue_size=10)
    rospy.init_node('encoders', anonymous=True)
    rate = rospy.Rate(100)

    #initial spi
    lw_spi = spidev.SpiDev()
    lw_spi.open(0,0)    #gpio8
    lw_spi.max_speed_hz = 5000
    lw_spi.mode = 0b01

    rw_spi = spidev.SpiDev()
    rw_spi.open(0,1) #gpio7
    rw_spi.max_speed_hz = 5000
    rw_spi.mode = 0b01

    data2degree = linear_transform((0,16383),(0,360))

    while not rospy.is_shutdown():
        odom_data = Odometery()
        odom_data.header = get_rostime()
        #Get past time data
        #log time data
        now = rospy.get_rostime()        

        #messure angle
    
        #calculate velocity

        #get covariance
        #publish odom data to /odom 
        past_time = now
        rate.sleep()
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
    main()

