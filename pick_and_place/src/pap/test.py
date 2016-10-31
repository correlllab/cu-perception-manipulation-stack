#!/usr/bin/env python

import rospy
import numpy as np
import tf
import action_database
import smach
from pap.jaco import JacoGripper, Jaco
from kinova_msgs.msg import JointAngles, PoseVelocity



def main():
    rospy.init_node("test")
    jg = JacoGripper()
    jg.set_position([50,50,0])
    j = Jaco()
    rate = rospy.Rate(100)
    cart_velo = ([.05,.05,0.0,0.0,0.0,0.0])
    msg = PoseVelocity(
        twist_linear_x=cart_velo[0],
        twist_linear_y=cart_velo[1],
        twist_linear_z=cart_velo[2],
        twist_angular_x=cart_velo[3],
        twist_angular_y=cart_velo[4],
        twist_angular_z=cart_velo[5])
    for i in range(100):
        j.kinematic_control(msg)
        rate.sleep()



if __name__ == '__main__':
    main()
