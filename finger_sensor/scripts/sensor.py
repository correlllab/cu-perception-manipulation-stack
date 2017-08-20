#!/usr/bin/env python
from __future__ import division, print_function

import serial
import rospy
from std_msgs.msg import Int32MultiArray, MultiArrayLayout, MultiArrayDimension
import numpy as np

BAUD_RATE = 115200
PORT_NAME = '/dev/ttyACM0'
NUM_SENSORS = 2  # CHANGE this to no. of sensor values read

def sensor_node():


    pub = rospy.Publisher('/sensor_values', Int32MultiArray, queue_size=1)
    rospy.init_node('sensor_node')
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        with serial.Serial(PORT_NAME, BAUD_RATE, timeout=0.1) as ser:
            data_canditate = ser.readline().decode('utf-8')
            # print (data_candidate)
            data_candidate = ser.readline().decode('utf-8')
            data_candidate = np.fromstring(data_candidate,sep=' ',dtype=np.uint32)

            if data_candidate.shape[0] == NUM_SENSORS:
                values = data_candidate
                msg = Int32MultiArray(
                    MultiArrayLayout(
                    [MultiArrayDimension('sensor data', NUM_SENSORS, 1)],1),values) # CHANGE the second arg to no. of sensor values read values)

                pub.publish(msg)

            else:
            	rospy.signal_shutdown("\n"+"No data available on serial port. Shutting down!"+"\n")
                print ("\n"+"No data available on serial port. Shutting down!"+"\n")

            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('sensor_node')
    s = sensor_node()
