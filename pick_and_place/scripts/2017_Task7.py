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
from finger_sensor_msgs.msg import FingerSAI, FingerFAI, FingerTouch, FingerDetect

import tf
import tf2_ros
import commands
import random

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

        self.tool_wrench_sub = rospy.Subscriber("/j2n6s300_driver/out/tool_wrench",
                                                    WrenchStamped,
                                                    self.tool_wrench)

        self.tool_pose_sub = rospy.Subscriber("/j2n6s300_driver/out/tool_pose",
                                                    PoseStamped,
                                                    self.tool_pose)

        self.sensor_sub = rospy.Subscriber('/sensor_values',Int32MultiArray,self.handle_sensor)

        self.obj_det = False
        self.sensor_values = []
        self.touch_finger_1 = False
        self.touch_finger_3 = False
        self.calibrated = False
        self.bump_finger_1 = 0
        self.bump_finger_2 = 0
        self.tool_wrench_x = 0
        self.tool_wrench_y = 0
        self.tool_wrench_z = 0
        self.angles = []
        self.sensorValues_1 = []
        self.sensorValues_2 = []
        self.tool_pose_ = [0]*7

    def tool_wrench(self,msg):
        self.tool_wrench_x = msg.wrench.force.x
        self.tool_wrench_y = msg.wrench.force.y
        self.tool_wrench_z = msg.wrench.force.z
        # print((self.tool_wrench_x/(self.tool_wrench_x+self.tool_wrench_y)), (self.tool_wrench_y/(self.tool_wrench_x+self.tool_wrench_y)))

    def tool_pose(self,msg):
        self.tool_pose_[0] = msg.pose.position.x
        self.tool_pose_[1] = msg.pose.position.y
        self.tool_pose_[2] = msg.pose.position.z
        self.tool_pose_[3] = msg.pose.orientation.x
        self.tool_pose_[4] = msg.pose.orientation.y
        self.tool_pose_[5] = msg.pose.orientation.z
        self.tool_pose_[6] = msg.pose.orientation.w
        # print (self.tool_pose_)


    def detect_Bump(self,msg):
        self.bump_finger_1 = msg.finger1

    def handle_sensor(self, msg):
        self.sensor_values = msg.data
        # print (self.sensor_values)
        # self.sensorValues_1.append(self.sensor_values[0])
        # self.sensorValues_2.append(self.sensor_values[1])

    def set_obj_det(self,msg):
        self.obj_det = np.any(np.array([msg.finger1, msg.finger2, msg.finger3]))
        # print(self.obj_det)


    def set_touch(self, msg):
        self.touch_finger_1 = msg.finger1
        self.touch_finger_2 = msg.finger2

    def callback(self,data):
        self.current_joint_angles[0] = data.joint1
        self.current_joint_angles[1] = data.joint2
        self.current_joint_angles[2] = data.joint3
        self.current_joint_angles[3] = data.joint4
        self.current_joint_angles[4] = data.joint5
        self.current_joint_angles[5] = data.joint6
        self.current_joint_angles[6] = data.joint7
        # print (self.current_joint_angles[5])


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

    def cmmnd_makeContact_ground(self, sensitivity, pose):
        rate = rospy.Rate(100)
        while (self.bump_finger_1 < sensitivity)and not rospy.is_shutdown():
            # print (self.tool_wrench_x, self.tool_wrench_y)
            self.cmmnd_CartesianPosition([pose[0]+((random.random()*2-1)/100), pose[1]+((random.random()*2-1)/100),pose[2],pose[3],pose[4],pose[5],pose[6]],0)
            pose = [pose[0]+((random.random()*2-1)/100), pose[1]+((random.random()*2-1)/100),pose[2]-0.009,pose[3],pose[4],pose[5],pose[6]]
            rate.sleep()
        print ("contact made with the ground")

    def cmmnd_makeContact_ground1(self, sensitivity):
        rate = rospy.Rate(100)
        while (self.bump_finger_1 < sensitivity)and not rospy.is_shutdown():
            # print (self.tool_wrench_x, self.tool_wrench_y)
            self.cmmnd_CartesianVelocity([0,0,-0.03,0,0,0,1])
            rate.sleep()
        print ("contact made with the ground")

    def cmmd_touchBlock(self,current_finger_position):
        ii = 0
        rate = rospy.Rate(100)
        while self.touch_finger_1 != True and not rospy.is_shutdown():
            current_finger_position[0] += 5 # slowly close finger_1 until contact is made
            # print (current_finger_position[0])
            self.cmmnd_FingerPosition([current_finger_position[0], current_finger_position[1], current_finger_position[2]])
            rate.sleep()
        self.touch_finger_1 = False

        while self.touch_finger_2 != True and not rospy.is_shutdown():
            current_finger_position[1] += 5 # slowly close finger_1 until contact is made
            # print (current_finger_position[0])
            self.cmmnd_FingerPosition([current_finger_position[0], current_finger_position[1], current_finger_position[2]])
            rate.sleep()
        self.touch_finger_2 = False

        return current_finger_position




if __name__ == '__main__':
    rospy.init_node("task_7")
    rate = rospy.Rate(49)
    p = pick_peas_class()


    # p.j.home()
    # p.cmmnd_FingerPosition([75,75,75])

    ## print ("Starting task. . .\n")
    # =========================
    # circle placing position
    # =========================
    p.cmmnd_CartesianPosition([0.546411573887, -0.3385501503942, 0.0928448587656, 0.620167553425, 0.783816814423, -0.0133556500077, 0.0290686544031],0)
    p.cmmnd_makeContact_ground1(25)
    p.cmmnd_makeContact_ground(25, p.tool_pose_)
    p.cmmnd_FingerPosition([60,60,70])

    # ===========================
    ## rectangle placing position
    # ===========================
    # p.cmmnd_CartesianPosition([0.562245368958, -0.294906884432, 0.081909827888, 0.0850020498037, 0.995069265366, -0.0124650411308, 0.0495618768036],0)
    # p.cmmnd_makeContact_ground1(25)
    # p.cmmnd_makeContact_ground(35, p.tool_pose_)
    # p.cmmnd_FingerPosition([60,60,70])

    # =========================
    ## square placing position
    # =========================
    # p.cmmnd_CartesianPosition([0.545260071754, -0.132728561759, 0.0699792727828, 0.779677569866, 0.625241935253, -0.00358248315752, 0.034096993506],0)
    # p.cmmnd_makeContact_ground1(25)
    # p.cmmnd_makeContact_ground(100, p.tool_pose_)
    # p.cmmnd_FingerPosition([60,60,70])


    ## object scanning
    # p.cmmnd_JointAngles([0,0,0,0,0,-360,0],'-r')
    # with open('values_1.txt', 'w') as f:
    #     f.writelines(["%s\n" % item  for item in p.sensorValues_1])
    # with open('values_2.txt', 'w') as f:
    #     f.writelines(["%s\n" % item  for item in p.sensorValues_2])

    # p.cmmnd_FingerPosition([60,0,0])
    # for i in [0,45,90,135,180,225,270,305,360]:
    #     p.cmmnd_JointAngles([0,0,0,0,0,-i,0],'-r')
    #     p.sensorValues_1.append(p.sensor_values)
    #
    # with open('values_2.txt', 'w') as f:
    #     f.writelines(["%s\n" % item  for item in p.sensorValues_1])
