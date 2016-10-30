#!/usr/bin/env python

import rospy
import numpy as np
import tf
from pap.jaco import Jaco, JacoGripper
import commands
import time
import actionlib
import kinova_msgs.msg
from std_msgs.msg import Bool

if __name__=='__main__':
    rospy.init_node('cal_pos')
    jn = Jaco()
    jn.move_joints([203.327209473,279.609375,143.161758423,149.386367798,119.659095764,111.204551697])
