#! /usr/bin/env python
from __future__ import division, print_function, absolute_import
import rospy
from pap.jaco import Jaco
from pap.manager import PickAndPlaceNode
from kinova_msgs.msg import JointAngles, PoseVelocity

from std_msgs.msg import Header, Int32MultiArray
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion

import tf

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

            self.spoon_pose = PoseStamped(
                Header(0, rospy.Time(0), 'root'),
                Pose(Point(float(translation[0]),float(translation[1]),float(translation[2])),
                    Quaternion(float(quaternion[0]),float(quaternion[1]),float(quaternion[2]),float(quaternion[3]))))
            self.j.gripper.open()
            self.j.move_ik(self.spoon_pose)
            self.j.gripper.close()

        else:
            print ("we DONT have the frame")



    def goto_bowl(self):
        if self.listen.frameExists("/root") and self.listen.frameExists("/bowl_position"):
            self.listen.waitForTransform('/root','/bowl_position',rospy.Time(),rospy.Duration(100.0))
            print ("we have the bowl frame")
            # t1 = self.listen.getLatestCommonTime("/root", "bowl_position")
            translation, quaternion = self.listen.lookupTransform("/root", "/bowl_position", rospy.Time(0))

            print (translation)
            self.bowl_pose = PoseStamped(
                Header(0, rospy.Time(0), 1),
                Pose(Point(float(translation[0]),float(translation[1]),float(translation[2])),
                     Quaternion(float(quaternion[0]),float(quaternion[1]),float(quaternion[2]),float(quaternion[3]))))

            # print (self.bowl_pose.pose)
            # self.j.gripper.open()
            # print ("waiting...")
            #self.j.client.cancel_all_goals()
            # print ()
            #rospy.sleep(10)
            self.j.move_ik(self.bowl_pose)
            # self.j.gripper.close()
        else:
            print ("we DONT have the bowl frame")


    def move_joints(self):
        #print (self.joint_angles)
            jointangles = [self.joint_angles[0], self.joint_angles[1], self.joint_angles[2], self.joint_angles[3], self.joint_angles[4], self.joint_angles[5]+20]
            try:
                self.j.move_joints(jointangles)
            except rospy.ROSInterruptException:
                print('program interrupted before completion')

    def cartesian_velocity_cmmnd(self, joint_velo):
        self.dicta = joint_velo
        print type(self.dicta)
        rate = rospy.Rate(100) #100Hz
        while not rospy.is_shutdown():
            self.cart_vel_pub.publish(self.dicta)
            rate.sleep()




if __name__ == '__main__':
    rospy.init_node("task_1")
    # n = PickAndPlaceNode(Jaco)
    p = pick_peas_class()
    p.readJointAngles()
    # p.move_calib_position()
    # p.j.home()
    #while not rospy.is_shutdown():
    #    if p.listen.frameExists("/root") and p.listen.frameExists("bowl_position") and p.listen.frameExists("/spoon_position"):
    # print ("Waiting for frame...")
    # while not (p.listen.frameExists("/root") and p.listen.frameExists("bowl_position") and p.listen.frameExists("/spoon_position")):
        # pass
    # print ("Starting task...\n")
    # p.pick_spoon()
    # print ("Spoon reached\n")
    # p.goto_bowl()
    # print ("Bowl reached\n")
    # p.move_joints()
    vel = PoseVelocity
    vel.twist_linear_x = 0
    vel.twist_linear_y = 0
    vel.twist_linear_z = 0
    vel.twist_angular_x = 0
    vel.twist_angular_y = 0
    vel.twist_angular_z = 10

    # print (vel.twist_angular_z)

    try:
        p.cartesian_velocity_cmmnd(vel)
    except rospy.ROSInterruptException:
        pass
