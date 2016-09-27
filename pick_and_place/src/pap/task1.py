#! /usr/bin/env python
from __future__ import division, print_function, absolute_import
import rospy
from pap.jaco import Jaco
from pap.manager import PickAndPlaceNode
from kinova_msgs.msg import JointAngles, PoseVelocity

from std_msgs.msg import Header, Int32MultiArray
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion

import pose_action_client

import tf

currentCartesianCommand = [0.212322831154, -0.257197618484, 0.509646713734, 1.63771402836, 1.11316478252, 0.134094119072] # default home in unit mq

class pick_peas_class(object):
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
            Pose(Point(0.3424,-0.1766,0.0381),
                 Quaternion(0.1802, 0.6434, 0.7075, 0.2299)))
        self.j.move_ik(self.calib_pose)

    def pick_spoon(self):
        if self.listen.frameExists("/root") and self.listen.frameExists("/spoon_position"):
            print ("we have the spoon frame")
            self.listen.waitForTransform('/root','/spoon_position',rospy.Time(),rospy.Duration(100.0))
            t = self.listen.getLatestCommonTime("/root", "/spoon_position")
            translation, quaternion = self.listen.lookupTransform("/root", "/spoon_position", t)

            translation =  list(translation)
            quaternion = list(quaternion)
            pose_value = translation + quaternion
            # print ('this is the coordinate of the spoon \n')
            orientation_XYZ = pose_action_client.Quaternion2EulerXYZ(quaternion)
            print ('Translation',translation)
            print ('Orientation: ' + str(orientation_XYZ) +'\n')
            # self.spoon_pose = PoseStamped(
            #     Header(0, rospy.Time(0), 'root'),
            #     Pose(Point(float(translation[0]),float(translation[1]),float(translation[2])),
            #         Quaternion(float(quaternion[0]),float(quaternion[1]),float(quaternion[2]),float(quaternion[3]))))

            self.j.gripper.open()
            # self.j.move_ik(self.spoon_pose)

            pose_action_client.getcurrentCartesianCommand('j2n6a300_')
            pose_mq, pose_mdeg, pose_mrad = pose_action_client.unitParser('mq', pose_value, 0)
            # print ('this is pose wrt home .. !? maybe \n')

            # # Print spoon and final position
            # if self.listen.frameExists("/root") and self.listen.frameExists("/spoon_position"):
            #     self.listen.waitForTransform('/root','/spoon_position',rospy.Time(),rospy.Duration(100.0))
            #     t = self.listen.getLatestCommonTime("/root", "/spoon_position")
            #     translation, quaternion = self.listen.lookupTransform("/root", "/spoon_position", t)
            #     orientation_XYZ = pose_action_client.Quaternion2EulerXYZ(quaternion)
            #     print ('\nSpoon Translation wrt /root',translation)
            #     print ('Spoon Orientation wrt /root: ' + str(orientation_XYZ) +'\n')

            poses = [float(n) for n in pose_mq]
            orientation_XYZ = pose_action_client.Quaternion2EulerXYZ(poses[3:])
            print ('Cmd Pose Translation',poses[:3])
            print ('Cmd Pose Orientation: ' + str(orientation_XYZ) +'\n')


            #print (pose_mdeg)
            try:
                poses = [float(n) for n in pose_mq]
                result = pose_action_client.cartesian_pose_client(poses[:3], poses[3:])
                print('Cartesian pose sent!')

            except rospy.ROSInterruptException:
                print ("program interrupted before completion")

            # pose_action_client.cartesian_pose_client(self.spoon_pose.pose.position, self.spoon_pose.pose.orientation)
            self.j.gripper.close()

        else:
            print ("we DONT have the frame")



    def goto_bowl(self):
        if self.listen.frameExists("/root") and self.listen.frameExists("/bowl_position"):
            self.listen.waitForTransform('/root','/bowl_position',rospy.Time(),rospy.Duration(100.0))
            print ("we have the bowl frame")
            # t1 = self.listen.getLatestCommonTime("/root", "bowl_position")
            translation, quaternion = self.listen.lookupTransform("/root", "/bowl_position", rospy.Time(0))

            # print (translation)
            # self.bowl_pose = PoseStamped(
            #     Header(0, rospy.Time(0), 1),
            #     Pose(Point(float(translation[0]),float(translation[1]),float(translation[2])),
            #          Quaternion(float(quaternion[0]),float(quaternion[1]),float(quaternion[2]),float(quaternion[3]))))

            translation =  list(translation)
            quaternion = list(quaternion)
            pose_value = translation + quaternion
            print ('this is the coordinate of the spoon \n')
            print (pose_value)

            pose_action_client.getcurrentCartesianCommand('j2n6a300_')
            pose_mq, pose_mdeg, pose_mrad = pose_action_client.unitParser('mq', pose_value, 0)
            print ('this is pose wrt home .. !? maybe \n')
            print (pose_mq)
            try:
                poses = [float(n) for n in pose_mq]
                result = pose_action_client.cartesian_pose_client(poses[:3], poses[3:])
                print('Cartesian pose sent!')

            except rospy.ROSInterruptException:
                print ("program interrupted before completion")


        else:
            print ("we DONT have the bowl frame")


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
    p = pick_peas_class()
    p.readJointAngles()

    while not (p.listen.frameExists("/j2n6a300_end_effector") and p.listen.frameExists("bowl_position") and p.listen.frameExists("/spoon_position")):
        pass

    print ("Starting task...\n")
    p.pick_spoon()

    # print ("Searching spoon...\n")
    # for i in range(10):
    #     if i<50:
    #         cart_velocities = [0.05,0,0,0,0,0] # linear[0:2], angular[3:5]
    #         p.cmmd_cart_velo(cart_velocities)
    #         # i+=1
    #         print (i)
    #     else:
    #         cart_velocities = [-0.05,0,0,0,0,0]
    #         p.cmmd_cart_velo(cart_velocities)
    #         # i+=1
    #         print (i)
    # p.j.home()

    print ("Spoon reached\n")
    p.goto_bowl()
    print ("Bowl reached\n")
    # p.move_joints()
