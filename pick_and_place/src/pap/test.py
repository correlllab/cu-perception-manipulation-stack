#!/usr/bin/env python

import rospy
import numpy as np
import tf
from pap.jaco import Jaco, JacoGripper
import commands
import time

if __name__ == '__main__':
    rospy.init_node("test")
    rate = rospy.Rate(100)
    jg = JacoGripper()
    jg.open()
    print(jg.currentFingerPercent)
    jg.close()
    print(jg.currentFingerPercent)
    rospy.spin()
