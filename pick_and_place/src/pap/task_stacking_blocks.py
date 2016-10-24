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

import tf
import commands

class PickPlaceClass(object):
    def __init__(self):
        self.j = Jaco()
        self.listener = tf.TransformListener()
        self.current_joint_angles = [0]*6


        self.velocity_pub = rospy.Publisher('/j2n6a300_driver/in/cartesian_velocity',
                                            PoseVelocity, queue_size=1)

        self.obj_det_sub = rospy.Subscriber('/finger_sensor/obj_detected',
                                            Bool, self.set_obj_det)

        self.finger_m_touch_sub = rospy.Subscriber('/finger_sensor_middle/touch',
                                                    Bool, self.set_m_touch)

        self.finger_r_touch_sub = rospy.Subscriber('/finger_sensor_right/touch',
                                                    Bool, self.set_r_touch)

        self.joint_angles_sub = rospy.Subscriber("/j2n6a300_driver/out/joint_angles",
                                                JointAngles, self.callback)

        self.calibrate_obj_det_pub = rospy.Publisher("/finger_sensor/calibrate_obj_det",
                                                    Bool,
                                                    queue_size=1)

        self.calibrate_obj_det_sub = rospy.Subscriber("/finger_sensor/obj_det_calibrated",
                                                    Bool,
                                                    self.set_calibrated)

        self.obj_det = False
        self.m_touch = False
        self.r_touch = False
        self.calibrated = False



    def set_obj_det(self,msg):
        self.obj_det = msg.data

    def set_m_touch(self,msg):
        self.m_touch = msg.data

    def set_r_touch(self,msg):
        self.r_touch = msg.data

    def callback(self,data):
        # self.current_joint_angles[0] = data.joint1
        self.current_joint_angles[1] = data.joint2
        self.current_joint_angles[2] = data.joint3
        self.current_joint_angles[3] = data.joint4
        self.current_joint_angles[4] = data.joint5
        self.current_joint_angles[5] = data.joint6
        # print (self.current_joint_angles)


    def cmmnd_CartesianPosition(self, pose_value, relative):
        pose_action_client.getcurrentCartesianCommand('j2n6a300_')
        pose_mq, pose_mdeg, pose_mrad = pose_action_client.unitParser('mq', pose_value, relative)
        poses = [float(n) for n in pose_mq]
        orientation_XYZ = pose_action_client.Quaternion2EulerXYZ(poses[3:])

        try:
            poses = [float(n) for n in pose_mq]
            result = pose_action_client.cartesian_pose_client(poses[:3], poses[3:])
        except rospy.ROSInterruptException:
            print ("program interrupted before completion")

    def cmmnd_FingerPosition(self, finger_value):
        commands.getoutput('rosrun kinova_demo fingers_action_client.py j2n6a300 percent -- {0} {1} {2}'.format(finger_value[0],finger_value[1],finger_value[2]))
        # fingers_action_client.getCurrentFingerPosition('j2n6a300_')
        #
        # finger_turn, finger_meter, finger_percent = fingers_action_client.unitParser('percent', finger_value, '-r')
        # finger_number = 3
        # finger_maxDist = 18.9/2/1000  # max distance for one finger in meter
        # finger_maxTurn = 6800  # max thread turn for one finger
        #
        # try:
        #     if finger_number == 0:
        #         print('Finger number is 0, check with "-h" to see how to use this node.')
        #         positions = []  # Get rid of static analysis warning that doesn't see the exit()
        #         exit()
        #     else:
        #         positions_temp1 = [max(0.0, n) for n in finger_turn]
        #         positions_temp2 = [min(n, finger_maxTurn) for n in positions_temp1]
        #         positions = [float(n) for n in positions_temp2]
        #
        #     print('Sending finger position ...')
        #     result = fingers_action_client.gripper_client(positions)
        #     print('Finger position sent!')
        #
        # except rospy.ROSInterruptException:
        #     print('program interrupted before completion')

    def cmmnd_JointAngles(self,joints_cmd, relative):
        joints_action_client.getcurrentJointCommand('j2n6a300_')
        joint_degree, joint_radian = joints_action_client.unitParser('degree', joints_cmd, relative)


        try:
            # print("dafuq")
            positions = [float(n) for n in joint_degree]
            result = joints_action_client.joint_angle_client(positions)
        except rospy.ROSInterruptException:
            print('program interrupted before completion')

    def cmmnd_CartesianVelocity(self,cart_velo):
        msg = PoseVelocity(
            twist_linear_x=cart_velo[0],
            twist_linear_y=cart_velo[1],
            twist_linear_z=cart_velo[2],
            twist_angular_x=cart_velo[3],
            twist_angular_y=cart_velo[4],
            twist_angular_z=cart_velo[5])
        # rate = rospy.Rate(100)
        # while not rospy.is_shutdown():
        self.velocity_pub.publish(msg)
            # rate.sleep()

    def set_calibrated(self,msg):
        self.calibrated = msg.data


if __name__ == '__main__':
    rospy.init_node("task_1")
    rate = rospy.Rate(100)
    p = PickPlaceClass()
    print (p.listener.getFrameStrings())
    rospy.spin()
