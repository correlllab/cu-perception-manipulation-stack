#! /usr/bin/env python
from __future__ import division, print_function, absolute_import
import sys
import rospy
from pap.jaco import Jaco, JacoGripper
from pap.manager import PickAndPlaceNode
from kinova_msgs.msg import JointAngles, PoseVelocity
from finger_sensor_msgs.msg import FingerDetect, FingerTouch

from std_msgs.msg import Header, Int32MultiArray, Bool
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
import numpy as np

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

class GraspObject(smach.State):
    def __init__(self,start,third_finger=True):
        smach.State.__init__(self, outcomes=['grasped', 'not_grasped'])
        self.finger_touch_sub = rospy.Subscriber('/finger_sensor/touch',
                                                FingerTouch,
                                                self.set_finger_touch)
        self.finger_touch = [None, None, None]
        self.jgn = JacoGripper()
        self.start = start
        self.third_finger = third_finger

    def set_finger_touch(self,msg):
        self.finger_touch[0] = msg.finger1
        self.finger_touch[1] = msg.finger2
        self.finger_touch[2] = msg.finger3

    def execute(self,userdata):
        status = self.jgn.close_with_feedback(self.start,self.third_finger)
        if np.any(np.array(self.finger_touch) == True) and status == 'done':
            return 'grasped'
        else:
            return 'not_grasped'

class SearchObject(smach.State):
    def __init__(self,detect_goal):
        smach.State.__init__(self, outcomes=['found', 'not_found'])
        self.finger_detect_sub = rospy.Subscriber('/finger_sensor/obj_detected',
                                                FingerDetect,
                                                self.set_finger_detect)
        self.finger_detect = [None, None, None]
        self.jn = Jaco()
        self.detect_goal = detect_goal
        self.listen = tf.TransformListener()
        self.transformer = tf.TransformerROS()

    def set_finger_detect(self,msg):
        self.finger_detect[0] = msg.finger1
        self.finger_detect[1] = msg.finger2
        self.finger_detect[2] = msg.finger3

    def create_pose_velocity_msg(self,cart_velo):
        if self.listen.frameExists("/root") and self.listen.frameExists("/j2n6a300_end_effector"):
            self.listen.waitForTransform('/root',"/j2n6a300_end_effector",rospy.Time(),rospy.Duration(100.0))
            t = self.listen.getLatestCommonTime("/root", "/j2n6a300_end_effector")
            translation, quaternion = self.listen.lookupTransform("/root", "/j2n6a300_end_effector", t)
        transform = self.transformer.fromTranslationRotation(translation, quaternion)
        cart_velo[0:3] = np.dot(transform,(cart_velo[0:3]+[1.0]))[0:3]
        msg = PoseVelocity(
            twist_linear_x=-cart_velo[0],
            twist_linear_y=-cart_velo[1],
            twist_linear_z=cart_velo[2],
            twist_angular_x=cart_velo[3],
            twist_angular_y=cart_velo[4],
            twist_angular_z=cart_velo[5])
        return msg

    def execute(self,userdata):
        if np.all(np.array(self.finger_detect) == np.array(self.detect_goal)):
            return 'found'
        else:
            found = self.back_forth_search_xy()
            return found

    def back_forth_search_xy(self):
        rate = rospy.Rate(10)
        msg = self.create_pose_velocity_msg([-.05,0.0,0.0,0.0,0.0,0.0])
        for i in range(25):
            if np.all(np.array(self.finger_detect) == np.array(self.detect_goal)):
                return 'found'
            self.jn.kinematic_control(msg)
            rate.sleep()

        return 'not_found'
