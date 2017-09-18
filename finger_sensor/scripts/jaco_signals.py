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

        self.object_det_calibrated_pub = rospy.Publisher("/finger_sensor/obj_det_calibrated", Bool, queue_size=1)
        self.calibrate_obj_det_sub = rospy.Subscriber("/finger_sensor/calibrate_obj_det", Bool, self.set_calibrate)
        self.fai_sub = rospy.Subscriber("/finger_sensor/fai", FingerFAI, self.detect_touch)
        self.sai_sub = rospy.Subscriber("/finger_sensor/sai", FingerSAI, self.detect_object)
        self.object_det_pub = rospy.Publisher("/finger_sensor/obj_detected", FingerDetect, queue_size=1)
        self.touch_pub = rospy.Publisher("/finger_sensor/touch", FingerTouch, queue_size=1)

    def set_calibrate(self,msg):
        self.calibrate = msg.data
        self.sai_calibration = [None] * 3
        print('Calibrating Fingers...')

    def detect_touch(self,msg):
        touch = self.current_fingers_touch[:]
        fai = [msg.finger1, msg.finger2, msg.finger3]
        tol = 300

        for finger in range(3):
            if fai[finger] > tol and self.current_fingers_touch[finger] == False:
                touch[finger] = True
            elif fai[finger] < -tol and self.current_fingers_touch[finger] == True:
                touch[finger] = False

        self.current_fingers_touch = self.send_finger_msg(self.touch_pub,FingerTouch,touch,self.current_fingers_touch)

    def detect_object(self,msg):
        detected = self.objectDet[:]
        sai = [msg.finger1, msg.finger2, msg.finger3]

        for finger in range(3):
            if self.calibrate == False and self.sai_calibration[finger] != [None]*3:
                    if sai[finger] > self.sai_calibration[finger] and self.objectDet[finger] == False:
                        detected[finger] = True
                    elif sai[finger] < self.sai_calibration[finger] and self.objectDet[finger] == True:
                        detected[finger] = False
            else:
                self.calibrate_fingers(sai,finger,50)

        self.objectDet = self.send_finger_msg(self.object_det_pub,FingerDetect,detected,self.objectDet)

    def calibrate_fingers(self,data,finger,tolerance):
        if np.all(np.array(self.sai_calibration) != np.array([None]*3)):
            self.calibrate=False
        elif self.calibrate_vals[finger].maxlen == len(self.calibrate_vals[finger]):
            self.sai_calibration[finger] = max(self.calibrate_vals[finger]) + tolerance #340 for spoon
            self.calibrate_vals[finger].clear()
            self.object_det_calibrated_pub.publish(True)
        else:
            self.calibrate_vals[finger].append(data[finger])
            self.object_det_calibrated_pub.publish(False)

    def send_finger_msg(self,publisher,msg_type,data,prev_data):
        msg = msg_type()
        if np.any(np.array(prev_data) != np.array(data)):
            prev_data = data
            msg.finger1 = data[0]
            msg.finger2 = data[1]
            msg.finger3 = data[2]
            publisher.publish(msg)
        return data

if __name__=='__main__':
    rospy.init_node('signal_detector')
    sd = SignalDetector()
    rospy.spin()
