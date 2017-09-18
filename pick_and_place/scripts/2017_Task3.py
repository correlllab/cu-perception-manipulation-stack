#! /usr/bin/env python
from __future__ import division, print_function, absolute_import
import sys
import rospy
from jaco import Jaco
from manager import PickAndPlaceNode
from kinova_msgs.msg import JointAngles, PoseVelocity
from finger_sensor_msgs.msg import FingerDetect, FingerTouch
from std_msgs.msg import Header, Int32MultiArray, Bool
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
import numpy as np

import pose_action_client
import fingers_action_client
import joints_action_client

import tf
import tf2_ros
import commands

class pick_peas_class(object):
    def __init__(self):
        self.j = Jaco()
        self.listener = tf.TransformListener()
        self.current_joint_angles = [0]*7

        self.tfBuffer = tf2_ros.Buffer()
        self.listen = tf2_ros.TransformListener(self.tfBuffer)

        self.velocity_pub = rospy.Publisher('/j2n6s300_driver/in/cartesian_velocity',
                                            PoseVelocity, queue_size=1)

        self.joint_angles_sub = rospy.Subscriber("/j2n6s300_driver/out/joint_angles",
                                                JointAngles, self.callback)

        self.obj_det_sub = rospy.Subscriber('/finger_sensor/obj_detected',
                                            FingerDetect, self.set_obj_det)

        self.fingetouch_finger_2_sub = rospy.Subscriber('/finger_sensor/touch',
                                                 FingerTouch, self.set_touch)

        self.calibrate_obj_det_pub = rospy.Publisher("/finger_sensor/calibrate_obj_det",
                                                    Bool,
                                                    queue_size=1)

        self.calibrate_obj_det_sub = rospy.Subscriber("/finger_sensor/obj_det_calibrated",
                                                    Bool,
                                                    self.set_calibrated)

        self.obj_det_finger_1 = False
        self.obj_det_finger_2 = False
        self.touch_finger_1 = False
        self.touch_finger_3 = False
        self.calibrated = False



    def set_obj_det(self,msg):
        # self.obj_det = np.any(np.array([msg.finger1, msg.finger2, msg.finger3]))
        self.obj_det_finger_1 = msg.finger1
        self.obj_det_finger_2 = msg.finger2
        # print(self.obj_det)


    def set_touch(self, msg):
        self.touch_finger_1 = msg.finger1
        self.touch_finger_3 = msg.finger2

    def callback(self,data):
        self.current_joint_angles[0] = data.joint1
        self.current_joint_angles[1] = data.joint2
        self.current_joint_angles[2] = data.joint3
        self.current_joint_angles[3] = data.joint4
        self.current_joint_angles[4] = data.joint5
        self.current_joint_angles[5] = data.joint6
        self.current_joint_angles[6] = data.joint7
        # print (self.current_joint_angles)


    def cmmnd_CartesianPosition(self, pose_value, relative):
        pose_action_client.getcurrentCartesianCommand('j2n6s300_')
        pose_mq, pose_mdeg, pose_mrad = pose_action_client.unitParser('mq', pose_value, relative)
        poses = [float(n) for n in pose_mq]
        orientation_XYZ = pose_action_client.Quaternion2EulerXYZ(poses[3:])

        try:
            poses = [float(n) for n in pose_mq]
            result = pose_action_client.cartesian_pose_client(poses[:3], poses[3:])
        except rospy.ROSInterruptException:
            print ("program interrupted before completion")

    def cmmnd_FingerPosition(self, finger_value):
        commands.getoutput('rosrun kinova_demo fingers_action_client.py j2n6s300 percent -- {0} {1} {2}'.format(finger_value[0],finger_value[1],finger_value[2]))

    def cmmnd_CartesianVelocity(self,cart_velo):
        msg = PoseVelocity(
            twist_linear_x=cart_velo[0],
            twist_linear_y=cart_velo[1],
            twist_linear_z=cart_velo[2],
            twist_angular_x=cart_velo[3],
            twist_angular_y=cart_velo[4],
            twist_angular_z=cart_velo[5])
        self.velocity_pub.publish(msg)

    def cmmnd_JointAngles(self,joints_cmd, relative):
        joints_action_client.getcurrentJointCommand('j2n6s300_')
        joint_degree, joint_radian = joints_action_client.unitParser('degree', joints_cmd, relative)
        try:
            positions = [float(n) for n in joint_degree]
            result = joints_action_client.joint_angle_client(positions)
        except rospy.ROSInterruptException:
            print('program interrupted before completion')

    def set_calibrated(self,msg):
        self.calibrated = msg.data

    def goto_spoon(self):

        try:
            trans = self.tfBuffer.lookup_transform('root', 'spoon_position', rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            # continue

        translation  = [trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z]
        rotation = [trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w]
        pose_value = translation + rotation
        #second arg=0 (absolute movement), arg = '-r' (relative movement)
        self.cmmnd_CartesianPosition(pose_value, 0)


    def lift_spoon(self):

        rate = rospy.Rate(100) # NOTE to publish cmmds to velocity_pub at 100Hz
        while self.touch_finger_1 != True:
            self.cmmnd_CartesianVelocity([0,0.025,0,0,0,0,1])
            rate.sleep()
        self.touch_finger_1 = False

        # p.cmmnd_CartesianPosition([0.02,0,0,0,0,0,1],'-r')

        self.cmmnd_FingerPosition([0, 0, 60])
        self.cmmnd_FingerPosition([100, 0, 100])
        self.cmmnd_CartesianPosition([0, 0, 0.13, 0, 0, 0, 1],'-r')
        # self.cmmnd_FingerPosition([100, 100, 100])


    def searchSpoon(self):
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            try:
                trans = self.tfBuffer.lookup_transform('root', 'j2n6s300_end_effector', rospy.Time())
                break
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rate.sleep()

        translation  = [trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z]
        rotation = [trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w]

        matrix1 = self.listener.fromTranslationRotation(translation,rotation)


        counter = 0
        rate = rospy.Rate(100)
        while not self.obj_det_finger_1 or self.obj_det_finger_2 and not rospy.is_shutdown():
            counter = counter + 1
            if(counter < 300):
                cart_velocities = np.dot(matrix1[:3,:3],np.array([0,0,0.05])[np.newaxis].T) #change in y->x, z->y, x->z
                cart_velocities = cart_velocities.T[0].tolist()
                self.cmmnd_CartesianVelocity(cart_velocities + [0,0,0,1])
                print("forward")
            else:
                cart_velocities = np.dot(matrix1[:3,:3],np.array([0,0,-0.05])[np.newaxis].T)
                cart_velocities = cart_velocities.T[0].tolist()
                self.cmmnd_CartesianVelocity(cart_velocities + [0,0,0,1])
                print("backwards")
            if(counter > 600):
                p.cmmnd_CartesianPosition([0,0.01,0,0,0,0,1],'r')
                counter = 0
            rate.sleep()

        p.cmmnd_CartesianPosition([0.01,0,0,0,0,0,1],'r')


    def goto_stirCup(self):

        try:
            trans = self.tfBuffer.lookup_transform('root', 'stir_position', rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            # continue

        translation  = [trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z]
        rotation = [trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w]
        pose_value = translation + rotation
        #second arg=0 (absolute movement), arg = '-r' (relative movement)
        self.cmmnd_CartesianPosition(pose_value, 0)


    def stir_cup(self):
        for i in range(3):
            self.cmmnd_CartesianPosition([0.02, 0, 0, 0, 0, 0, 1], '-r')
            self.cmmnd_CartesianPosition([0.01, -0.01, 0, 0, 0, 0, 1], '-r')
            self.cmmnd_CartesianPosition([0, -0.02, 0, 0, 0, 0, 1], '-r')
            self.cmmnd_CartesianPosition([-0.01, -0.01, 0, 0, 0, 0, 1], '-r')
            self.cmmnd_CartesianPosition([-0.02, 0, 0, 0, 0, 0, 1], '-r')
            self.cmmnd_CartesianPosition([-0.01, 0.01, 0, 0, 0, 0, 1], '-r')
            self.cmmnd_CartesianPosition([0, 0.02, 0, 0, 0, 0, 1], '-r')
            self.cmmnd_CartesianPosition([0.01, 0.01, 0, 0, 0, 0, 1], '-r')


if __name__ == '__main__':
    rospy.init_node("task_3")
    rate = rospy.Rate(100)
    p = pick_peas_class()

    p.j.home()
    p.cmmnd_FingerPosition([0, 0, 60])

    p.calibrate_obj_det_pub.publish(True)
    while p.calibrated == False:
        pass

    print ("Starting task. . .\n")
    p.goto_spoon()
    p.cmmnd_JointAngles([0,0,0,0,0,-8,0],'-r')

    print ("Searching spoon. . .\n")
    # p.searchSpoon()
    p.searchSpoon()
    p.lift_spoon()

    print ("Going to stir bowl. . .\n")
    p.goto_stirCup()
    p.cmmnd_CartesianPosition([0,0,-0.085,0,0,0,1],'-r')
    p.stir_cup()

    # p.j.home()
