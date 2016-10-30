#! /usr/bin/env python
from __future__ import division, print_function, absolute_import
import sys
import rospy
from pap.jaco import Jaco
from pap.manager import PickAndPlaceNode
from kinova_msgs.msg import JointAngles, PoseVelocity

from std_msgs.msg import Header, Int32MultiArray, Bool
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
import numpy as np

import pose_action_client
import fingers_action_client
import joints_action_client

import commands
import tf
import smach

class GotoObject(smach.State):
    def __init__(self,frame):
        smach.State.__init__(self, outcomes=['there','no_tf_found'])
        self.frame = frame
        self.jn = Jaco()
        self.listen = tf.TransformListener()

    def execute(self,userdata):
        if self.listen.frameExists("/root") and self.listen.frameExists(self.frame):
            self.listen.waitForTransform('/root',self.frame,rospy.Time(),rospy.Duration(100.0))
            t = self.listen.getLatestCommonTime("/root", self.frame)
            translation, quaternion = self.listen.lookupTransform("/root", self.frame, t)

            translation =  list(translation)
            quaternion = list(quaternion)
            msg = self.make_pose_stamped_msg(translation,quaternion)
            self.jn.move_ik(msg)
            return 'there'

        else:
            return 'no_tf_found'

    def make_pose_stamped_msg(self,translation,orientation,frame='/j2n6a300_link_base'):
        msg = PoseStamped()
        msg.header.frame_id = frame
        msg.header.stamp = rospy.Time.now()
        msg.pose.position.x = translation[0]
        msg.pose.position.y = translation[1]
        msg.pose.position.z = translation[2]
        msg.pose.orientation.x = orientation[0]
        msg.pose.orientation.y = orientation[1]
        msg.pose.orientation.z = orientation[2]
        return msg
