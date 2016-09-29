#! /usr/bin/env python
from __future__ import division, print_function, absolute_import
import sys
import rospy
from pap.jaco import Jaco
from pap.manager import PickAndPlaceNode
from kinova_msgs.msg import JointAngles, PoseVelocity

from std_msgs.msg import Header, Int32MultiArray
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion

import pose_action_client

import tf

currentCartesianCommand = [0.212322831154, -0.257197618484, 0.509646713734, 1.63771402836, 1.11316478252, 0.134094119072] # default home in unit mq

class stirCup(object):
    def __init__(self):
        self.j = Jaco()
        self.listen = tf.TransformListener()
        self.joint_angles = [0]*6
        # self.sub3 = rospy.Subscriber('/sensor_values', Int32MultiArray,
        #                              self.callback, queue_size=1)
        self.cart_vel_pub = rospy.Publisher('/j2n6a300_driver/in/cartesian_velocity',
                                                            PoseVelocity, queue_size=1)


    def readJointAngles(self):
        self.joint_angles_sub = rospy.Subscriber("/j2n6a300_driver/out/joint_angles",
                                                                JointAngles, self.callback)

    def callback(self,data):
        self.joint_angles[0] = data.joint1
        self.joint_angles[1] = data.joint2
        self.joint_angles[2] = data.joint3
        self.joint_angles[3] = data.joint4
        self.joint_angles[4] = data.joint5
        self.joint_angles[5] = data.joint6
        #print (self.joint_angles)


    def move_calib_position(self):
        # move arm to the calibration position
        self.calib_pose = PoseStamped(
            Header(0, rospy.Time(0), self.j.base),
            Pose(Point(0.338520675898,-0.175860479474,0.0356775075197),
                 Quaternion(-0.0183493755758,0.708424150944, 0.704712092876, 0.0343413949013)))
        self.j.move_ik(self.calib_pose)

    def move_cartcmmd(self, pose_value, relative):
        pose_action_client.getcurrentCartesianCommand('j2n6a300_')
        pose_mq, pose_mdeg, pose_mrad = pose_action_client.unitParser('mq', pose_value, relative)

        poses = [float(n) for n in pose_mq]
        orientation_XYZ = pose_action_client.Quaternion2EulerXYZ(poses[3:])

        try:
            poses = [float(n) for n in pose_mq]
            result = pose_action_client.cartesian_pose_client(poses[:3], poses[3:])
        except rospy.ROSInterruptException:
            print ("program interrupted before completion")

    def pick_spoon(self):
        if self.listen.frameExists("/root") and self.listen.frameExists("/spoon_position"):
            print ("we have the spoon frame")
            self.listen.waitForTransform('/root','/spoon_position',rospy.Time(),rospy.Duration(100.0))
            t = self.listen.getLatestCommonTime("/root", "/spoon_position")
            translation, quaternion = self.listen.lookupTransform("/root", "/spoon_position", t)

            translation =  list(translation)
            quaternion = list(quaternion)
            pose_value = translation + quaternion
            print (quaternion)
            orientation_XYZ = pose_action_client.Quaternion2EulerXYZ(quaternion)

            self.j.gripper.open()
            #second arg=0 (absolute movement), arg = '-r' (relative movement)
            self.move_cartcmmd(pose_value, 0)

            self.j.gripper.close()

        else:
            print ("we DONT have the frame")

    def goto_bowl(self):
        if self.listen.frameExists("/root") and self.listen.frameExists("/bowl_position"):
            self.listen.waitForTransform('/root','/bowl_position',rospy.Time(),rospy.Duration(100.0))
            # print ("we have the bowl frame")
            # t1 = self.listen.getLatestCommonTime("/root", "bowl_position")
            translation, quaternion = self.listen.lookupTransform("/root", "/bowl_position", rospy.Time(0))

            translation =  list(translation)
            quaternion = list(quaternion)
            pose_value = translation + quaternion
            #second arg=0 (absolute movement), arg = '-r' (relative movement)
            self.move_cartcmmd(pose_value, 0)
        else:
            print ("we DONT have the bowl frame")

    def stir_cup(self):
        for i in range(3):
            self.move_cartcmmd([0.05, 0, 0, 0, 0, 0, 1], '-r')
            self.move_cartcmmd([0.025, -0.025, 0, 0, 0, 0, 1], '-r')
            self.move_cartcmmd([0, -0.05, 0, 0, 0, 0, 1], '-r')
            self.move_cartcmmd([-0.025, -0.025, 0, 0, 0, 0, 1], '-r')
            self.move_cartcmmd([-0.05, 0, 0, 0, 0, 0, 1], '-r')
            self.move_cartcmmd([-0.025, 0.025, 0, 0, 0, 0, 1], '-r')
            self.move_cartcmmd([0, 0.05, 0, 0, 0, 0, 1], '-r')
            self.move_cartcmmd([0.025, 0.025, 0, 0, 0, 0, 1], '-r')


    def move_joints(self):
        #print (self.joint_angles)
            jointangles = [self.joint_angles[0], self.joint_angles[1], self.joint_angles[2], self.joint_angles[3], self.joint_angles[4], self.joint_angles[5]+20]
            try:
                self.j.move_joints(jointangles)
            except rospy.ROSInterruptException:
                print('program interrupted before completion')

    def cmmd_cart_velo(self,cart_velo):
        msg = PoseVelocity(
            twist_linear_x=cart_velo[0],
            twist_linear_y=cart_velo[1],
            twist_linear_z=cart_velo[2],
            twist_angular_x=cart_velo[3],
            twist_angular_y=cart_velo[4],
            twist_angular_z=cart_velo[5])

        self.j.kinematic_control(msg)

if __name__ == '__main__':
    rospy.init_node("task_1")
    # n = PickAndPlaceNode(Jaco)
    s = stirCup()
    s.readJointAngles()

    while not (s.listen.frameExists("/root") and s.listen.frameExists("/spoon_position") and s.listen.frameExists("/bowl_position")):
        pass

    print ("Starting task...\n")
    s.pick_spoon()

    s.move_cartcmmd([0, 0, 0.1, 0, 0, 0, 1], '-r')
    s.goto_bowl()
    # s.move_cartcmmd([0, 0, -0.08, 0, 0, 0, 1], '-r')
    s.stir_cup()
