#! /usr/bin/env python
from __future__ import print_function, division, absolute_import

from collections import deque
from time import time

import numpy as np
from scipy import signal

import rospy

from std_msgs.msg import Int32MultiArray, Float64
from keyboard.msg import Key
from sensor_msgs.msg import Imu


class FilterSignal(object):
    def __init__(self):
        self.recording = False
        self.names = ('sair', 'sail', 'fai', 'faii')
        self.data = {n: [] for n in self.names}
        # TODO hardcoded for left arm
        self.acc_sub = rospy.Subscriber(
            '/robot/accelerometer/left_accelerometer/state',
            Imu,
            self.handle_acc)

        self.sensor_sub = rospy.Subscriber(
            '/sensor_values',
            Int32MultiArray,
            self.handle_sensor)

        self.sair_pub = rospy.Publisher(
            '/finger_sensor/sair',
            Float64,
            queue_size=5)
        self.sail_pub = rospy.Publisher(
            '/finger_sensor/sail',
            Float64,
            queue_size=5)
        self.fai_pub = rospy.Publisher(
            '/finger_sensor/fai',
            Float64,
            queue_size=5)
        self.faii_pub = rospy.Publisher(
            '/finger_sensor/faii',
            Float64,
            queue_size=5)

        self.kb_sub = rospy.Subscriber('/keyboard/keyup',
                                       Key,
                                       self.keyboard_cb, queue_size=10)

        # Every queue should hold about 4 seconds of data
        self.sensor_t = deque(maxlen=80)
        self.sensor_values = deque(maxlen=80)

        self.acc_t = deque(maxlen=400)
        self.acc = deque(maxlen=400)

        # 0.66pi rad/sample (cutoff frequency over nyquist frequency
        # (ie, half the sampling frequency)). For the wrist, 33 Hz /
        # (100 Hz/ 2). TODO Double check arguments
        self.b1, self.a1 = signal.butter(1, 0.66, 'high', analog=False)
        # 0.5p rad/sample. For the tactile sensor, 5 Hz / (20 Hz / 2).
        self.b, self.a = signal.butter(1, 0.5, 'high', analog=False)

    def keyboard_cb(self, msg):
        character = chr(msg.code)
        if character == 'r':
            self.recording = not self.recording
            if self.recording:
                rospy.loginfo('Recording enabled')
            else:
                rospy.loginfo('Recording disabled')

    def handle_acc(self, msg):
        # Check header of Imu msg
        t = msg.header.stamp.secs
        acc = (msg.linear_acceleration.x,
               msg.linear_acceleration.y,
               msg.linear_acceleration.z)
        # TODO check how uniform t is, if not investigate correct
        # filtering option
        self.acc_t.append(t)
        self.acc.append(acc)

    def handle_sensor(self, msg):
        # TODO maybe time stamp sensor values with header
        # TODO rospy.get_rostime() vs rospy.Time.now()?
        # print(rospy.get_rostime(), rospy.Time.now())  # They're different
        self.sensor_t.append(rospy.Time.now())
        self.sensor_values.append(msg.data)

    def compute_sai(self):
        # Skipping the front tip sensor (idx 7 and 15)
        right = sum(self.sensor_values[-1][8:15])
        left = sum(self.sensor_values[-1][0:7])
        self.sair_pub.publish(Float64(right))
        self.sail_pub.publish(Float64(left))
        if self.recording:
            t = time()
            self.data['sair'].append((t, right))
            self.data['sail'].append((t, left))

    def compute_fai(self):
        # Original code began by substracting the minimum ever seen,
        # maybe we "zero" the sensor instead? But in fact, with a
        # highpass filter, it won't matter anyway so we don't do
        # i... Assumption tested successfully too.
        filtered_values = signal.lfilter(self.b, self.a,
                                         self.sensor_values, axis=0)
        self.fai = filtered_values.sum(axis=1)
        val = self.fai[-1]
        self.fai_pub.publish(Float64(val))
        if self.recording:
            t = time()
            self.data['fai'].append((t, val))

    def compute_faii(self):
        filtered_acc = signal.lfilter(self.b1, self.a1, self.acc, axis=0)
        self.faii = np.sqrt((filtered_acc**2).sum(axis=1))
        # Publish just the last value
        val = self.faii[-1]
        self.faii_pub.publish(Float64(val))
        if self.recording:
            t = time()
            self.data['faii'].append((t, val))

    def save(self):
        import pickle
        with open('/home/jc/ros/baxter_ws/latest_data', 'w') as f:
            pickle.dump(self.data, f)
        #print(pickle.dumps(self.data))

if __name__ == '__main__':
    rospy.init_node('filter_signals')
    f = FilterSignal()
    # Give it some time to collect some data
    rospy.sleep(3)
    r = rospy.Rate(30)
    while not rospy.is_shutdown():
        f.compute_sai()
        f.compute_fai()
        f.compute_faii()
        r.sleep()
    f.save()
