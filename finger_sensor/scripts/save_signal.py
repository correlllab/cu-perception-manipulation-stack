#!/usr/bin/env python
from __future__ import division, print_function

from collections import deque

from scipy import stats

import rospy
from std_msgs.msg import (Float64, Float64MultiArray,
                          MultiArrayLayout, MultiArrayDimension)


class ComputeMoments(object):
    def __init__(self, topic, window=20):
        self.vals = deque(maxlen=window)

        self.sub = rospy.Subscriber(topic, Float64, self.handle_msg)
        self.pub = rospy.Publisher(topic + '/moments',
                                   Float64MultiArray,
                                   queue_size=1)

    def handle_msg(self, msg):
        val = msg.data
        self.vals.append(val)
        l = list(self.vals)
        moments = []
        for i in range(1, 11):
            moments.append(stats.moment(l, moment=i))
        msg = Float64MultiArray(
            MultiArrayLayout([MultiArrayDimension('moments', 10, 1)], 1),
            moments)
        self.pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node('moment_computation')
    c1 = ComputeMoments('/finger_sensor/sair')
    c2 = ComputeMoments('/finger_sensor/sail')
    c3 = ComputeMoments('/finger_sensor/fai')
    c4 = ComputeMoments('/finger_sensor/faii')
    rospy.spin()
