#! /usr/bin/env python
from collections import deque
import rospy
import numpy as np
from std_msgs.msg import Int32MultiArray, Float64, Bool

class SignalDetector():
    def __init__(self):
        self.objectDet = False
        self.saim_calibration = None
        self.prev_val_l = False
        self.prev_val_r = False
        self.prev_val_m = False
        self.current_saim_val = None
        self.calibrate = False
        self.calibrate_vals = deque(maxlen=100)

        self.object_det_calibrated_pub = rospy.Publisher("/finger_sensor/obj_det_calibrated",
                                            Bool,
                                            queue_size=1)

        self.calibrate_obj_det_sub = rospy.Subscriber("/finger_sensor/calibrate_obj_det",
                                            Bool,
                                            self.set_calibrate)

        self.fail_sub = rospy.Subscriber("/finger_sensor/fail",
                                        Float64,
                                        self.fail_detect_change)

        self.fair_sub = rospy.Subscriber("/finger_sensor/fair",
                                        Float64,
                                        self.fair_detect_change)

        self.faim_sub = rospy.Subscriber("/finger_sensor/faim",
                                        Float64,
                                        self.faim_detect_change)

        self.saim_sub = rospy.Subscriber("/finger_sensor/saim",
                                        Float64,
                                        self.saim_detect_change)

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
        self.object_det_calibrated_pub.publish(False)



    def set_calibrate(self,msg):
        self.calibrate = msg.data

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

    def saim_detect_change(self,msg):
        if self.calibrate == False:
            if self.saim_calibration != None:
                if msg.data > self.saim_calibration and self.objectDet == False:
                    self.object_det_pub.publish(True)
                    self.objectDet = True
                elif msg.data < self.saim_calibration and self.objectDet == True:
                    self.object_det_pub.publish(False)
                    self.objectDet = False
        else:
            if self.calibrate_vals.maxlen == len(self.calibrate_vals):
                self.object_det_calibrated_pub
                self.saim_calibration = max(self.calibrate_vals) + 340 #340 for spoon
                print (self.saim_calibration)
                self.calibrate_vals.clear()
                self.object_det_calibrated_pub
                self.object_det_calibrated_pub.publish(True)
            else:
                self.calibrate_vals.append(msg.data)





if __name__=='__main__':
    rospy.init_node('signal_detector')
    sd = SignalDetector()
    rospy.spin()
