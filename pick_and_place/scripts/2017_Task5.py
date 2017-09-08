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

from finger_sensor_msgs.msg import FingerSAI, FingerFAI, FingerTouch, FingerDetect

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

        self.bump_det_sub = rospy.Subscriber("/finger_sensor/fai",
                                                    FingerFAI,
                                                    self.detect_Bump)

        self.obj_det = False
        self.touch_finger_1 = False
        self.touch_finger_3 = False
        self.calibrated = False
        self.bump_finger_1 = 0

    def set_calibrated(self,msg):
        self.calibrated = msg.data

    def detect_Bump(self,msg):
        self.bump_finger_1 = msg.finger1
        # print (self.bump_finger_1)

    def set_obj_det(self,msg):
        self.obj_det = np.any(np.array([msg.finger1, msg.finger2, msg.finger3]))
        # print(self.obj_det)


    def set_touch(self, msg):
        '''
        touch messages used for control
        '''

        self.touch_finger_1 = msg.finger1
        self.touch_finger_3 = msg.finger3

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
        '''
        Commands the arm in cartesian position mode. Server client
        interface from kinova api.
        '''
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
        '''
        Commands the finger joints. Server client
        interface from kinova api.
        '''
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
        '''
        Commands the arm in joint position mode. Server client
        interface from kinova api.
        '''
        joints_action_client.getcurrentJointCommand('j2n6s300_')
        joint_degree, joint_radian = joints_action_client.unitParser('degree', joints_cmd, relative)
        try:
            positions = [float(n) for n in joint_degree]
            result = joints_action_client.joint_angle_client(positions)
        except rospy.ROSInterruptException:
            print('program interrupted before completion')


    def goto(self, from_frame, to_frame):
        '''
        Calculates the transfrom from from_frame to to_frame
        and commands the arm in cartesian mode.
        '''
        try:
            trans = self.tfBuffer.lookup_transform(from_frame, to_frame, rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            # continue

        translation  = [trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z]
        rotation = [trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w]
        pose_value = translation + rotation
        #second arg=0 (absolute movement), arg = '-r' (relative movement)
        self.cmmnd_CartesianPosition(pose_value, 0)

    def cmmnd_makeContact_ground(self, sensitivity):
        rate = rospy.Rate(100)
        while (self.bump_finger_1<sensitivity)and not rospy.is_shutdown():
            print (self.bump_finger_1)
            self.cmmnd_CartesianVelocity([0,0,-0.03,0,0,0,1])
            rate.sleep()
        print ("contact made with the ground")

    def pick_USBlight_1(self, current_finger_position):
        ii = 0
        rate = rospy.Rate(100)
        while self.touch_finger_1 != True and not rospy.is_shutdown():
            current_finger_position[0] += 5 # slowly close finger_1 until contact is made
            print (current_finger_position[0])
            self.cmmnd_FingerPosition([current_finger_position[0], current_finger_position[1], current_finger_position[2]])
            rate.sleep()
        self.touch_finger_1 = False
        return current_finger_position

    def pick_USBlight_2(self, current_finger_position):
        ii = 0
        rate = rospy.Rate(1000)
        while self.touch_finger_3 != True and not rospy.is_shutdown():
            current_finger_position[2] += 2 # slowly close finger_1 until contact is made
            print (current_finger_position[0])
            self.cmmnd_FingerPosition([current_finger_position[0], current_finger_position[1], current_finger_position[2]])
            rate.sleep()





if __name__ == '__main__':
    rospy.init_node("task_5")

    p = pick_peas_class()

    # p.j.home()

    p.cmmnd_FingerPosition([50,40,0])
    # pick position (hand generated) for normal light
    p.cmmnd_CartesianPosition([0.5, -0.19, 0.0726234141737, 0.765976846218, 0.641924321651, -0.00027509778738, -0.00027509778738], 0)
    p.cmmnd_CartesianPosition([0,0,-0.065,0,0,0,1],'r')
    p.cmmnd_JointAngles([0,0,0,0,0,8,0], 'r')

    # current_finger_position = p.pick_USBlight_1([50,40,0]) # close fingers with feedback
    # p.cmmnd_FingerPosition([current_finger_position[0],100,current_finger_position[2]]) # close fingers with feedback
    # p.cmmnd_FingerPosition([100,100,current_finger_position[2]]) # close fingers with feedback

    p.cmmnd_FingerPosition([100,100,0]) # close fingers without feedback

    p.cmmnd_CartesianPosition([0,0,0.06,0,0,0,1],'r')
    ## plug it back in
    # p.cmmnd_makeContact_ground(40) # argument #1 is sensitivity
    # p.cmmnd_CartesianPosition([0,0,-0.015,0,0,0,1],'r')
    # p.cmmnd_FingerPosition([0,0,0])
    p.cmmnd_CartesianPosition([0,0,-0.06,0,0,0,1],'r')
    p.cmmnd_FingerPosition([0,0,0])

    ## pick position (hand generated) for USB light
    # p.cmmnd_CartesianPosition([0.55, -0.2, 0.0373686514795, -0.55370759964, -0.470494896173, -0.533289194107, -0.433180242777], 0)
    # p.cmmnd_FingerPosition([50,50,50])
    # # p.cmmnd_FingerPosition([100,100,50])
    # p.cmmnd_CartesianPosition([0.06,0,0,0,0,0,1],'r')
    # current_finger_position = p.pick_USBlight_1([50,50,50]) # start from the current finger position
    # p.cmmnd_FingerPosition([current_finger_position[0],100,50])
    # p.cmmnd_FingerPosition([100,100,50])
    # p.cmmnd_CartesianPosition([0,0,0.08,0,0,0,1],'r')
    # ## plug it back in
    # p.cmmnd_makeContact_ground(100)
    # p.cmmnd_FingerPosition([0,0,0])
    #
    # p.j.home()
