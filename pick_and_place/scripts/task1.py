#! /usr/bin/env python
from __future__ import division, print_function, absolute_import
import rospy
from pap.robot import Jaco
import tf


def pick_peas():
    listen = tf.TransformListener()
    if listen.frameExists("/spoon_position"):
        print ("we have the frame")


if __name__ == '__main__':
    rospy.init_node("task_1")
    pick_peas()
    rospy.spin()
