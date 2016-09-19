#!/usr/bin/env python
from __future__ import division, print_function

import serial
import rospy
from std_msgs.msg import Int32MultiArray, MultiArrayLayout, MultiArrayDimension


def collect_data(port='/dev/ttyACM0'):
    with serial.Serial(port, 57600, timeout=0.1) as ser:
        ser.flushInput()
        # Give it some time to initialize
        data = []
        N = 1
        for i in range(N):
            data.append(ser.read(ser.inWaiting()))
            rospy.loginfo("Waiting for {} s more".format(N-i))
            rospy.sleep(1)
        print('\n'.join(filter(None, data)))
        buffer = []
        r = rospy.Rate(100)
        while not rospy.is_shutdown():
            buffer.append(ser.read(ser.inWaiting()))
            foo = ''.join(buffer).splitlines()
            try:
                last_full_line = foo[-2]
            except IndexError:
                r.sleep()
                continue
            try:
                values = [int(i) for i in last_full_line.split()]
                if len(values) == 16:
                    rospy.loginfo(values)
                    yield values
            except ValueError:
                rospy.loginfo(last_full_line)
                r.sleep()
                continue
            buffer = [foo[-1]]
            r.sleep()


def sensor_node():
    c = collect_data()
    #c = collect_data(port='/dev/ttyACM1')
    pub = rospy.Publisher('/sensor_values', Int32MultiArray, queue_size=1)
    rospy.init_node('sensor_node')
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        values = next(c)
        msg = Int32MultiArray(
            MultiArrayLayout([MultiArrayDimension('sensor data', 16, 1)], 1),
            values)
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('sensor_node')
    s = sensor_node()
