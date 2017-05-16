#!/usr/bin/env python
from __future__ import division, print_function

import serial
import rospy
from std_msgs.msg import Int32MultiArray, MultiArrayLayout, MultiArrayDimension
import numpy as np

BAUD_RATE = 57600
PORT_NAME = '/dev/ttyACM0'
NUM_SENSORS = 16  # CHANGE this to no. of sensor values read


ave_values = []
def collect_data(port=PORT_NAME):
    with serial.Serial(port, BAUD_RATE, timeout=0.1) as ser:
        ser.flushInput()
        # Give it some time to initialize
        data = []
        N = 1
        for i in range(N):
            data.append(ser.read(ser.inWaiting()))
            rospy.sleep(1)
        # print('\n'.join(filter(None, data)))
        buffer = []
        r = rospy.Rate(100)
        while not rospy.is_shutdown():
            
            buffer.append(ser.read(ser.inWaiting()))
            foo = ''.join(buffer).splitlines()
            try:
                last_full_line = foo[-2]
                #print(last_full_line)
            except IndexError:
                r.sleep()
                continue
            try:
                values = [int(i) for i in last_full_line.split()]
                if len(values) == NUM_SENSORS:  
                    yield values
            except ValueError:
                rospy.loginfo(last_full_line)
                r.sleep()
                continue
            buffer = [foo[-1]]
            r.sleep()


def sensor_node():
 
    c = collect_data()
    ave = np.empty([10,16])
    #average baseline for sensors
    #ave = [6569,5297,5885,5870,5564,6122,6033,5483,7108,5863,5864,5231,5666,5514,6421,6486]
    ave = [6643,5442,5048,5092,6036,5040,5061,0,5361,5506,4655,5262,4931,4424,4554,2464,]

    index = 0
    pub = rospy.Publisher('/sensor_values', Int32MultiArray, queue_size=1)
    rospy.init_node('sensor_node')
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        values = next(c)
        msg = Int32MultiArray(
            MultiArrayLayout([MultiArrayDimension('sensor data', NUM_SENSORS, 1)], 1), # CHANGE the second arg to no. of sensor values read
            #values) #uncomment to publish raw values
            np.subtract(values,ave)) #uncomment to publish values with base subtracted out

        print(np.subtract(values,ave))
        pub.publish(msg)
        '''#uncomment to calculate knew average base for each sensor
        ave[index] = values
        index += 1
        if(index >= 10):
            print(np.mean(ave, axis =0))
            index = 0
        '''
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('sensor_node')
    s = sensor_node()


