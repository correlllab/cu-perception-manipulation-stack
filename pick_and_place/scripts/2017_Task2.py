#! /usr/bin/env python
from __future__ import division, print_function, absolute_import
import sys
import rospy
from jaco import Jaco
from manager import PickAndPlaceNode
from kinova_msgs.msg import JointAngles, PoseVelocity
from finger_sensor_msgs.msg import FingerDetect, FingerTouch
from std_msgs.msg import Header, Int32MultiArray, Bool
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion, WrenchStamped
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

        self.tool_wrench_sub = rospy.Subscriber("/j2n6s300_driver/out/tool_wrench",
                                                    WrenchStamped,
                                                    self.tool_wrench)

        self.obj_det = False
        self.touch_finger_1 = False
        self.touch_finger_3 = False
        self.calibrated = False
        self.tool_wrench_x = 0
        self.tool_wrench_y = 0
        self.tool_wrench_z = 0


    def tool_wrench(self,msg):
        self.tool_wrench_x = msg.wrench.force.x
        self.tool_wrench_y = msg.wrench.force.y
        self.tool_wrench_z = msg.wrench.force.z


    def set_obj_det(self,msg):
        self.obj_det = np.any(np.array([msg.finger1, msg.finger2, msg.finger3]))
        # print(self.obj_det)


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

    def cmmnd_makeContact_ground(self, sensitivity):
        rate = rospy.Rate(100)
        while (self.tool_wrench_z > sensitivity)and not rospy.is_shutdown():
            print (self.tool_wrench_z)
            self.cmmnd_CartesianVelocity([0,0,-0.05,0,0,0,1])
            rate.sleep()
        print ("contact made with the ground")



if __name__ == '__main__':
    rospy.init_node("task_2")
    rate = rospy.Rate(100)
    p = pick_peas_class()
    p.j.home()
    p.cmmnd_FingerPosition([80,80,75])

    print ("Starting task. . .\n")
    # ====================================
    ## picking utensils from tray
    # ====================================
    ## picking spoon
    p.cmmnd_CartesianPosition([0.476334422827,-0.345300972462,0.0714216381311,-0.998056650162,0.0282405298203,-0.0554221756756,0.00370769621804],0)
    p.cmmnd_CartesianPosition([0,0,-0.075,0,0,0,1],'r')
    p.cmmnd_FingerPosition([100,100,75])
    p.cmmnd_CartesianPosition([0,0,0.2,0,0,0,1],'r')
    p.cmmnd_CartesianPosition([0.64, 0, 0, -0.725623369217, -0.685643792152, -0.0564622879028, -0.0132434144616],0)
    p.cmmnd_FingerPosition([80,80,75])

    ## picking fork
    p.cmmnd_CartesianPosition([0.539246678352, -0.342241108418, 0.08, -0.999147117138, 0.02394787594689, -0.0333280749619, 0.00455530406907],0)
    p.cmmnd_CartesianPosition([0,0,-0.1,0,0,0,1],'r')
    p.cmmnd_FingerPosition([100,100,75])
    p.cmmnd_CartesianPosition([0,0,0.2,0,0,0,1],'r')
    p.cmmnd_CartesianPosition([0.60404330492, 0.3, 0, -0.725623369217, -0.685643792152, -0.0564622879028, -0.0132434144616],0)
    p.cmmnd_FingerPosition([80,80,75])

    ## picking knife
    p.cmmnd_CartesianPosition([0.590910434723, -0.331213414669, 0.111737504601, -0.999147117138, 0.02394787594689, -0.0333280749619, 0.00455530406907],0)
    p.cmmnd_CartesianPosition([0,0,-0.13,0,0,0,1],'r')
    p.cmmnd_FingerPosition([100,100,75])
    p.cmmnd_CartesianPosition([0,0,0.2,0,0,0,1],'r')
    p.cmmnd_CartesianPosition([0.60404330492, 0.05, 0, -0.725623369217, -0.685643792152, -0.0564622879028, -0.0132434144616],0)
    p.cmmnd_FingerPosition([80,80,75])

    # ====================================
    ## placing utensils in tray
    # ====================================
    ## picking spoon
    p.cmmnd_CartesianPosition([0.64, 0, 0, -0.725623369217, -0.685643792152, -0.0564622879028, -0.0132434144616],0)
    p.cmmnd_CartesianPosition([0,0,-0.02,0,0,0,1],'-r')
    p.cmmnd_FingerPosition([100,100,100])
    p.cmmnd_CartesianPosition([0.476334422827,-0.345300972462,0.1,-0.998056650162,0.0282405298203,-0.0554221756756,0.00370769621804],0)
    p.cmmnd_FingerPosition([80,80,75])

    ## picking fork
    p.cmmnd_CartesianPosition([0.60404330492, 0.3, 0, -0.725623369217, -0.685643792152, -0.0564622879028, -0.0132434144616],0)
    p.cmmnd_CartesianPosition([0,0,-0.02,0,0,0,1],'-r')
    p.cmmnd_FingerPosition([100,100,100])
    p.cmmnd_CartesianPosition([0.539246678352, -0.342241108418, 0.08, -0.999147117138, 0.02394787594689, -0.0333280749619, 0.00455530406907],0)
    p.cmmnd_FingerPosition([80,80,75])

    ## picking knife
    p.cmmnd_CartesianPosition([0.60404330492, 0.05, 0, -0.725623369217, -0.685643792152, -0.0564622879028, -0.0132434144616],0)
    p.cmmnd_CartesianPosition([0,0,-0.02,0,0,0,1],'-r')
    p.cmmnd_FingerPosition([100,100,100])
    p.cmmnd_CartesianPosition([0.590910434723, -0.331213414669, 0.111737504601, -0.999147117138, 0.02394787594689, -0.0333280749619, 0.00455530406907],0)
    p.cmmnd_FingerPosition([80,80,75])
