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

        self.obj_det = False
        self.touch_finger_1 = False
        self.touch_finger_3 = False
        self.calibrated = False



    def set_obj_det(self,msg):
        self.obj_det = np.any(np.array([msg.finger1, msg.finger2, msg.finger3]))
        print(self.obj_det)


    def set_touch(self, msg):
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



if __name__ == '__main__':
    rospy.init_node("task_2")
    rate = rospy.Rate(100)
    p = pick_peas_class()
    p.j.home()
    p.cmmnd_FingerPosition([75,75,75])

    print ("Starting task. . .\n")
    p.cmmnd_CartesianPosition([0.539246678352, -0.342241108418, 0.08, 0.994541704655, 0.095476359129, 0.0416482128203, 0.0060525611043],0)
    p.cmmnd_CartesianPosition([0,0,-0.11,0,0,0,1],'r')
    p.cmmnd_FingerPosition([100,100,100])
    p.cmmnd_CartesianPosition([0,0,0.2,0,0,0,1],'r')
