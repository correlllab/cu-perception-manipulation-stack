#! /usr/bin/env python
from collections import deque
import rospy
import numpy as np
from std_msgs.msg import Int32MultiArray, Float64, Bool
from finger_sensor_msgs.msg import FingerSAI, FingerFAI, FingerTouch, FingerDetect

class SignalDetector():
    def __init__(self):
        self.objectDet = False
        self.saim_calibration = None
        self.current_fingers_touch = [False]*3
        self.current_saim_val = None
        self.calibrate = False
        self.calibrate_vals = deque(maxlen=100)

        self.object_det_calibrated_pub = rospy.Publisher("/finger_sensor/obj_det_calibrated",
                                            Bool,
                                            queue_size=1)

        self.calibrate_obj_det_sub = rospy.Subscriber("/finger_sensor/calibrate_obj_det",
                                            Bool,
                                            self.set_calibrate)

        self.fai_sub = rospy.Subscriber("/finger_sensor/fai",
                                        FingerFAI,
                                        self.detect_touch)

        #
        # self.sai_sub = rospy.Subscriber("/finger_sensor/sai",
        #                                 FingerSAI,
        #                                 self.saim_detect_change)

        self.object_det_pub = rospy.Publisher("/finger_sensor/obj_detected",
                                        FingerDetect,
                                        queue_size=1)

        self.touch_pub = rospy.Publisher("/finger_sensor/touch",
                                        FingerTouch,
                                        queue_size=1)



    def set_calibrate(self,msg):
        self.calibrate = msg.data

    def detect_touch(self,msg):
        touch_msg = FingerTouch()
        touch = self.current_fingers_touch[:]
        fai = [msg.finger1, msg.finger2, msg.finger3]

        for finger in range(3):
            if fai[finger] > 2000.0 and self.current_fingers_touch[finger] == False:
                touch[finger] = True

            if fai[finger] < -2000.0 and self.current_fingers_touch[finger] == True:
                touch[finger] = False

        if np.any(np.array(self.current_fingers_touch) != np.array(touch)):
            self.current_fingers_touch = touch
            touch_msg.finger1 = touch[0]
            touch_msg.finger2 = touch[1]
            touch_msg.finger3 = touch[2]
            self.touch_pub.publish(touch_msg)


    # def saim_detect_change(self,msg):
    #     if self.calibrate == False:
    #         if self.saim_calibration != None:
    #             if msg.data > self.saim_calibration and self.objectDet == False:
    #                 self.object_det_pub.publish(True)
    #                 self.objectDet = True
    #             elif msg.data < self.saim_calibration and self.objectDet == True:
    #                 self.object_det_pub.publish(False)
    #                 self.objectDet = False
    #     else:
    #         if self.calibrate_vals.maxlen == len(self.calibrate_vals):
    #             self.object_det_calibrated_pub
    #             self.saim_calibration = max(self.calibrate_vals) + 340 #340 for spoon
    #             print (self.saim_calibration)
    #             self.calibrate_vals.clear()
    #             self.object_det_calibrated_pub
    #             self.object_det_calibrated_pub.publish(True)
    #         else:
    #             self.calibrate_vals.append(msg.data)
    #
    #



if __name__=='__main__':
    rospy.init_node('signal_detector')
    sd = SignalDetector()
    rospy.spin()
