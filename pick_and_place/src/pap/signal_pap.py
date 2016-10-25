#! /usr/bin/env python
from collections import deque
import rospy
import numpy as np
from std_msgs.msg import Int32MultiArray, Float64, Bool
from finger_sensor_msgs.msg import FingerSAI, FingerFAI, FingerTouch, FingerDetect

class SignalDetector():
    def __init__(self):
        self.objectDet = [False] * 3
        self.sai_calibration = [None] * 3
        self.current_fingers_touch = [False]*3
        self.calibrate = False
        self.calibrate_vals = [deque(maxlen=100),deque(maxlen=100),deque(maxlen=100)]

        self.object_det_calibrated_pub = rospy.Publisher("/finger_sensor/obj_det_calibrated",
                                            Bool,
                                            queue_size=1)

        self.calibrate_obj_det_sub = rospy.Subscriber("/finger_sensor/calibrate_obj_det",
                                            Bool,
                                            self.set_calibrate)

        self.fai_sub = rospy.Subscriber("/finger_sensor/fai",
                                        FingerFAI,
                                        self.detect_touch)

        self.sai_sub = rospy.Subscriber("/finger_sensor/sai",
                                        FingerSAI,
                                        self.detect_object)

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


    def detect_object(self,msg):
        detect_msg = FingerDetect()
        detected = self.objectDet[:]
        sai = [msg.finger1, msg.finger2, msg.finger3]
        for finger in range(3):
            if self.calibrate == False:
                if self.sai_calibration[finger] != [None]*3:
                    if sai[finger] > self.sai_calibration[finger] and self.objectDet[finger] == False:
                        detected[finger] = True
                        self.objectDet[finger] = True
                    elif sai[finger] < self.sai_calibration[finger] and self.objectDet[finger] == True:
                        detected[finger] = False
                        self.objectDet[finger] = False
            else:
                if np.all(np.array(self.sai_calibration) != np.array([None]*3)):
                    self.calibrate=False
                elif self.calibrate_vals[finger].maxlen == len(self.calibrate_vals[finger]):
                    self.sai_calibration[finger] = max(self.calibrate_vals[finger]) + 300 #340 for spoon
                    self.calibrate_vals[finger].clear()
                    self.object_det_calibrated_pub.publish(True)
                else:
                    self.object_det_calibrated_pub.publish(False)
                    self.calibrate_vals[finger].append(sai[finger])

        # if np.any(np.array(self.objectDet) != np.array(detected)):
        self.objectDet = detected
        detect_msg.finger1 = detected[0]
        detect_msg.finger2 = detected[1]
        detect_msg.finger3 = detected[2]
        self.object_det_pub.publish(detect_msg)

if __name__=='__main__':
    rospy.init_node('signal_detector')
    sd = SignalDetector()
    rospy.spin()
