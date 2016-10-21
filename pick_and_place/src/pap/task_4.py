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

class pick_peas_class(object):
    def __init__(self):
        self.j = Jaco()
        self.listen = tf.TransformListener()
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


        self.obj_det = False
        self.m_touch = False
        self.r_touch = False
        self.calibrated = False


    def lift_shaker(self):
        rate = rospy.Rate(100) # NOTE to publish cmmds to velocity_pub at 100Hz
        # self.move_fingercmmd([0, 0, 0])
        while self.m_touch != True:
            self.cmmnd_CartesianVelocity([0.02,0,0,0,0,0,1])
            rate.sleep()
        self.r_touch = False
        # while not(self.m_touch and self.r_touch):
        #     self.cmmnd_CartesianVelocity([0.02,0,0,0,0,0,1])
            # self.move_joints([0,0,0,0,0,-5])
            # rate.sleep()




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


    def pick_shaker(self):
        if self.listen.frameExists("/root") and self.listen.frameExists("/shaker_position"):
            self.listen.waitForTransform('/root','/shaker_position',rospy.Time(),rospy.Duration(100.0))
            # print ("we have the bowl frame")
            # t1 = self.listen.getLatestCommonTime("/root", "bowl_position")
            translation, quaternion = self.listen.lookupTransform("/root", "/shaker_position", rospy.Time(0))

            translation =  list(translation)
            quaternion = list(quaternion)
            pose_value = translation + quaternion
            #second arg=0 (absolute movement), arg = '-r' (relative movement)
            self.cmmnd_CartesianPosition(pose_value, 0)
        else:
            print ("we DONT have the bowl frame")

    def goto_plate(self):
        if self.listen.frameExists("/root") and self.listen.frameExists("/plate_position"):
            self.listen.waitForTransform('/root','/plate_position',rospy.Time(),rospy.Duration(100.0))
            # t1 = self.listen.getLatestCommonTime("/root", "bowl_position")
            translation, quaternion = self.listen.lookupTransform("/root", "/plate_position", rospy.Time(0))

            translation =  list(translation)
            quaternion = list(quaternion)
            #quaternion = [0.8678189045198146, 0.0003956789257977804, -0.4968799802988633, 0.0006910675928639343]
            #quaternion = [0]*4
            pose_value = translation + quaternion
            #second arg=0 (absolute movement), arg = '-r' (relative movement)
            self.cmmnd_CartesianPosition(pose_value, 0)

        else:
            print ("we DONT have the bowl frame")



if __name__ == '__main__':
    rospy.init_node("task_1")
    rate = rospy.Rate(100)
    p = pick_peas_class()
    p.j.home()

    p.cmmnd_FingerPosition([0,0,0])

    while not (p.listen.frameExists("/root") and p.listen.frameExists("/shaker_position")): # p.listen.frameExists("bowl_position"):
        pass

    print ("Starting task. . .\n")
    p.pick_shaker()
    # p.cmmnd_JointAngles([0,0,0,0,20,-15],'-r')
    # p.cmmnd_CartesianPosition([0,0,-0.02,0,0,0,0], '-r')

    p.cmmnd_FingerPosition([100,100,100])

    # p.lift_shaker()
    # p.cmmnd_FingerPosition([30, 30, 30])
    # p.cmmnd_FingerPosition([90,90, 90])
    p.cmmnd_CartesianPosition([0,0,0.15,0,0,0,1], '-r')
    #
    p.goto_plate()
    #
    p.cmmnd_JointAngles([0,0,0,0,0,45],'-r')

    for i in range(250):
        p.cmmnd_JointAngles([0,0,0,0,0,-180],'-r')
        # p.cmmnd_CartesianPosition([0,0,0.15,0,0,0,1], '-r')
        # p.cmmnd_CartesianPosition([0,0,0-.15,0,0,0,1], '-r')
        p.cmmnd_JointAngles([0,0,0,0,0,180],'-r')
        # p.cmmnd_CartesianPosition([0,0,0.15,0,0,0,1], '-r')
        # p.cmmnd_CartesianPosition([0,0,0-.15,0,0,0,1], '-r')
