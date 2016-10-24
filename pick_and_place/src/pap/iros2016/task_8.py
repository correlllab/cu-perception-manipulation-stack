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

    def set_calibrated(self,msg):
        self.calibrated = msg.data

    def pick_driver(self):
        self.calibrate_obj_det_pub.publish(True)

        while self.calibrated == False:
            pass

        self.calibrate_obj_det_pub.publish(False)

        print("Finger Sensors calibrated")

        if self.listen.frameExists("/root") and self.listen.frameExists("/driver_position"):
            print ("we have the spoon frame")
            self.listen.waitForTransform('/root','/driver_position',rospy.Time(),rospy.Duration(100.0))
            t = self.listen.getLatestCommonTime("/root", "/driver_position")
            translation, quaternion = self.listen.lookupTransform("/root", "/driver_position", t)

            translation =  list(translation)
            quaternion = list(quaternion)
            pose_value = translation + quaternion
            # print (quaternion)
            orientation_XYZ = pose_action_client.Quaternion2EulerXYZ(quaternion)

            # self.j.gripper.open()
            #second arg=0 (absolute movement), arg = '-r' (relative movement)
            self.cmmnd_CartesianPosition(pose_value, 0)

            # self.j.gripper.close()

    def goto_nut(self):
        if self.listen.frameExists("/root") and self.listen.frameExists("/hangTowel"):
            self.listen.waitForTransform('/root','/hangTowel',rospy.Time(),rospy.Duration(100.0))
            # print ("we have the bowl frame")
            # t1 = self.listen.getLatestCommonTime("/root", "bowl_position")
            translation, quaternion = self.listen.lookupTransform("/root", "/hangTowel", rospy.Time(0))

            translation =  list(translation)
            quaternion = list(quaternion)
            pose_value = translation + quaternion
            #second arg=0 (absolute movement), arg = '-r' (relative movement)
            self.cmmnd_CartesianPosition(pose_value, 0)
        else:
            print ("we DONT have the bowl frame")

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


    def searchdriver(self):
        if self.listen.frameExists("/j2n6a300_end_effector") and self.listen.frameExists("/root"):
            # print ("we are in the search spoon fucntion")
            self.listen.waitForTransform('/j2n6a300_end_effector','/root',rospy.Time(),rospy.Duration(100.0))
            t = self.listen.getLatestCommonTime("/j2n6a300_end_effector","/root")
            translation, quaternion = self.listen.lookupTransform("/j2n6a300_end_effector","/root",t)
            matrix1=self.listen.fromTranslationRotation(translation,quaternion)
            counter=0
            rate=rospy.Rate(100)
            while not self.obj_det:
                  counter = counter + 1
                  if(counter < 200):
                    cart_velocities = np.dot(matrix1[:3,:3],np.array([0,0,0.05])[np.newaxis].T) #change in y->x, z->y, x->z
                    cart_velocities = cart_velocities.T[0].tolist()
                    self.cmmnd_CartesianVelocity(cart_velocities + [0,0,0,1])
                  else:
                    cart_velocities = np.dot(matrix1[:3,:3],np.array([0,0,-0.05])[np.newaxis].T)
                    cart_velocities = cart_velocities.T[0].tolist()
                    self.cmmnd_CartesianVelocity(cart_velocities + [0,0,0,1])
                  rate.sleep()
                  if(counter >400):
                     counter=0

    def lift_driver(self):
        rate = rospy.Rate(100) # NOTE to publish cmmds to velocity_pub at 100Hz
        # self.move_fingercmmd([0, 0, 0])
        while self.m_touch != True:
            self.cmmnd_CartesianVelocity([0.025,0,0,0,0,0,1])
            rate.sleep()
        self.r_touch = False
        # while not(self.m_touch and self.r_touch):
        #     self.cmmnd_CartesianVelocity([0.02,0,0,0,0,0,1])
            # self.move_joints([0,0,0,0,0,-5])
            # rate.sleep()

        self.cmmnd_FingerPosition([100, 00, 100])
        self.cmmnd_CartesianPosition([0, 0, 0.13, 0, 0, 0, 1],'-r')
        self.cmmnd_FingerPosition([100, 100, 100])

if __name__ == '__main__':
    rospy.init_node("task_1")
    rate = rospy.Rate(100)
    p = pick_peas_class()
    # p.j.home()
    p.cmmnd_FingerPosition([0, 100, 100])
    #
    while not (p.listen.frameExists("/root") and p.listen.frameExists("/driver_position")): # p.listen.frameExists("bowl_position"):
        pass
    # print ("Starting task. . .\n")
    p.pick_driver()
    # # p.cmmnd_JointAngles([0,0,0,0,0,-9],'-r')
    p.cmmnd_CartesianPosition([0,0,-0.18,0,0,0,1], '-r')
    p.cmmnd_FingerPosition([100, 100, 100])
    p.cmmnd_CartesianPosition([0,0,0.1,0,0,0,1], '-r')


    # p.cmmnd_FingerPosition([90,100,100])
    # # p.cmmnd_FingerPosition([100,0,100])
    # p.cmmnd_CartesianPosition([0,0,0.15,0,0,0,1], '-r')
    # # p.cmmnd_FingerPosition([100,100,100])
    # p.cmmnd_CartesianPosition([0,-0.075,0,0,0,0,1], '-r')
    # p.cmmnd_CartesianPosition([0.07,0,0,0,0,0,1], '-r')

    # print ("Going to hang the towel. . .\n")
    # p.cmmnd_CartesianPosition([-0.15,-0.4,0,0,0,0,1],'-r')
    # p.cmmnd_FingerPosition([0, 0, 0])
