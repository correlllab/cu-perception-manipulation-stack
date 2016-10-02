#! /usr/bin/env python
from collections import deque
import rospy
import numpy as np
from std_msgs.msg import Int32MultiArray, Float64, Bool

class SignalDetector():
    def __init__(self):
        self.objectDet = False


        self.fail_sub = rospy.Subscriber("/finger_sensor/fail",
                                        Float64,
                                        self.fail_detect_change)

        self.fair_sub = rospy.Subscriber("/finger_sensor/fair",
                                        Float64,
                                        self.fair_detect_change)

        self.faim_sub = rospy.Subscriber("/finger_sensor/faim",
                                        Float64,
                                        self.faim_detect_change)

        self.sail_sub = rospy.Subscriber("/finger_sensor/sair",
                                        Float64,
                                        self.sair_detect_change)

        self.object_det_pub = rospy.Publisher("/finger_sensor/obj_detected",
                                        Bool,
                                        queue_size=1)

        self.touch_r_pub = rospy.Publisher("/finger_sensor_right/touch",
                                        Bool,
                                        queue_size=1)

        self.touch_l_pub = rospy.Publisher("/finger_sensor_left/touch",
                                        Bool,
                                        queue_size=1)

        self.touch_m_pub = rospy.Publisher("/finger_sensor_middle/touch",
                                        Bool,
                                        queue_size=1)

        self.prev_val_l = False
        self.prev_val_r = False
        self.prev_val_m = False

    def fail_detect_change(self,msg):
        if msg.data > 1000.0 and self.prev_val_l == False:
             self.touch_l_pub.publish(True)
             self.prev_val_l = True
        elif msg.data < -1000.0 and self.prev_val_l == True:
             self.touch_l_pub.publish(False)
             self.prev_val_l = False

    def fair_detect_change(self,msg):
        if msg.data > 1000.0 and self.prev_val_r == False:
             self.touch_r_pub.publish(True)
             self.prev_val_r = True
        elif msg.data < -1000.0 and self.prev_val_r == True:
             self.touch_r_pub.publish(False)
             self.prev_val_r = False

    def faim_detect_change(self,msg):
        if msg.data > 1000.0 and self.prev_val_m == False:
             self.touch_m_pub.publish(True)
             self.prev_val_m = True
        elif msg.data < -1000.0 and self.prev_val_m == True:
             self.touch_m_pub.publish(False)
             self.prev_val_m = False

    def sair_detect_change(self,msg):
        # print(msg.data)
        if msg.data > 32300 and self.objectDet == False:
            self.object_det_pub.publish(True)
            self.objectDet = True
        elif msg.data < 32300 and self.objectDet == True:
            self.object_det_pub.publish(False)
            self.objectDet = False






if __name__=='__main__':
    rospy.init_node('signal_detector')
    sd = SignalDetector()
    rospy.spin()
