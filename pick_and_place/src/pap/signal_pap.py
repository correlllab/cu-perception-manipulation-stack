#! /usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Int32MultiArray, Float64, Bool

class SignalDetector():
    def __init__(self):
        self.fail_sub = rospy.Subscriber("/finger_sensor/fail",
                                        Float64,
                                        self.fail_detect_change)

        self.fair_sub = rospy.Subscriber("/finger_sensor/fair",
                                        Float64,
                                        self.fair_detect_change)

        self.touch_r_pub = rospy.Publisher("/finger_sensor_right/touch",
                                        Bool,
                                        queue_size=1)

        self.touch_l_pub = rospy.Publisher("/finger_sensor_left/touch",
                                        Bool,
                                        queue_size=1)
        self.prev_val_l = False
        self.prev_val_r = False

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


if __name__=='__main__':
    rospy.init_node('signal_detector')
    sd = SignalDetector()
    rospy.spin()
