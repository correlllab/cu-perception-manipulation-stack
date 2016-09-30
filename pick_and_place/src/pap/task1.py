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

import tf

class pick_peas_class(object):
    def __init__(self):
        self.j = Jaco()
        self.listen = tf.TransformListener()
        self.current_joint_angles = [0]*6


        # self.sub3 = rospy.Subscriber('/sensor_values', Int32MultiArray,
        #                              self.callback_1, queue_size=1)

        self.cart_vel_pub = rospy.Publisher('/j2n6a300_driver/in/cartesian_velocity',
                                                            PoseVelocity, queue_size=1)

        self.obj_det_sub = rospy.Subscriber('/finger_sensor/obj_detected',
                                            Bool,
                                            self.set_obj_det)
        self.obj_det = False

        # self.touch_r_sub = rospy.Subscriber("/finger_sensor_right/touch",
        #                                 Bool,
        #                                 queue_size=1)
        #
        # self.touch_l_sub = rospy.Subscriber("/finger_sensor_left/touch",
        #                                 Bool,
        #                                 queue_size=1)

    def readJointAngles(self):
        self.joint_angles_sub = rospy.Subscriber("/j2n6a300_driver/out/joint_angles",
                                                                JointAngles, self.callback)

    def set_obj_det(self,msg):
        self.obj_det = msg.data

    def callback(self,data):
        self.current_joint_angles[0] = data.joint1
        self.current_joint_angles[1] = data.joint2
        self.current_joint_angles[2] = data.joint3
        self.current_joint_angles[3] = data.joint4
        self.current_joint_angles[4] = data.joint5
        self.current_joint_angles[5] = data.joint6
        print (self.current_joint_angles)

    def callback_1(self, msg):
        # print (msg.data)
        pass


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
            # print (quaternion)
            orientation_XYZ = pose_action_client.Quaternion2EulerXYZ(quaternion)

            # self.j.gripper.open()
            #second arg=0 (absolute movement), arg = '-r' (relative movement)
            self.move_cartcmmd(pose_value, 0)

            # self.j.gripper.close()

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


    def goto_plate(self):
        if self.listen.frameExists("/root") and self.listen.frameExists("/plate_position"):
            self.listen.waitForTransform('/root','/plate_position',rospy.Time(),rospy.Duration(100.0))
            print ("we have the bowl frame")
            # t1 = self.listen.getLatestCommonTime("/root", "bowl_position")
            translation, quaternion = self.listen.lookupTransform("/root", "/plate_position", rospy.Time(0))

            translation =  list(translation)
            quaternion = [0.8678189045198146, 0.0003956789257977804, -0.4968799802988633, 0.0006910675928639343]
            pose_value = translation + quaternion
            #second arg=0 (absolute movement), arg = '-r' (relative movement)
            self.move_cartcmmd(pose_value, 0)

        else:
            print ("we DONT have the bowl frame")

    def move_joints(self):
        #print (self.joint_angles)
            # jointangles=[0]*6
            self.readJointAngles()
            print (self.current_joint_angles)
            # jointangles = [self.current_joint_angles[0], self.current_joint_angles[1], self.current_joint_angles[2], self.current_joint_angles[3], self.current_joint_angles[4], self.current_joint_angles[5]+20]
            # for i in range(6):
            #     jointangles[i] = self.current_joint_angles[i] + joints[i]
            #
            # print (jointangles)
            # try:
            #     self.j.move_joints(jointangles)
            # except rospy.ROSInterruptException:
            #     print('program interrupted before completion')

    def cmmd_cart_velo(self,cart_velo):
        msg = PoseVelocity(
            twist_linear_x=cart_velo[0],
            twist_linear_y=cart_velo[1],
            twist_linear_z=cart_velo[2],
            twist_angular_x=cart_velo[3],
            twist_angular_y=cart_velo[4],
            twist_angular_z=cart_velo[5])

        self.j.kinematic_control(msg)


    def searchSpoon(self):
        if self.listen.frameExists("/j2n6a300_end_effector") and self.listen.frameExists("/root"):
            # print ("we are in the search spoon fucntion")
            self.listen.waitForTransform('/j2n6a300_end_effector','/root',rospy.Time(),rospy.Duration(100.0))
            t = self.listen.getLatestCommonTime("/j2n6a300_end_effector","/root")
            translation, quaternion = self.listen.lookupTransform("/j2n6a300_end_effector","/root",t)
            matrix1=self.listen.fromTranslationRotation(translation,quaternion)
            counter=0
            rate=rospy.Rate(100)
            while not self.obj_det:
                #   print ("we are in the search spoon fucntion")
                  counter = counter + 1
                  if(counter < 200):
                    print('forward')
                    cart_velocities = np.dot(matrix1[:3,:3],np.array([-0.05,0,0])[np.newaxis].T)
                    cart_velocities = cart_velocities.T[0].tolist()
                    self.cmmd_cart_velo(cart_velocities + [0,0,0,1])
                  else:
                    print('backwards')
                    cart_velocities = np.dot(matrix1[:3,:3],np.array([0.05,0,0])[np.newaxis].T)
                    cart_velocities = cart_velocities.T[0].tolist()
                    self.cmmd_cart_velo(cart_velocities + [0,0,0,1])
                  rate.sleep()
                  if(counter >400):
                     counter=0


if __name__ == '__main__':
    rospy.init_node("task_1")
    # n = PickAndPlaceNode(Jaco)
    p = pick_peas_class()
    # p.j.gripper.set_position([0,100,100])

    while not (p.listen.frameExists("/root") and p.listen.frameExists("bowl_position") and p.listen.frameExists("/spoon_position")):
        pass

    print ("Starting task. . .\n")
    # p.pick_spoon()

    print ("Searching spoon. . .\n")
    # p.searchSpoon()

    # p.j.gripper.close()
    print ("Spoon found yay!!\n")

    print ("Lifitng the spoon. . .\n")
    # p.move_cartcmmd([0,0,0.1,0,0,0,1],'-r')

    print ("Going to bowl. . .\n")
    # p.goto_bowl()
    print ("Bowl reached. . .\n")

    print ("Scooping the peas. . .")
    # p.move_joints()
    # p.goto_plate()
    p.move_joints()
    # rospy.spin()
