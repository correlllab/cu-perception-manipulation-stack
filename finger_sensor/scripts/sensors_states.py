#!/usr/bin/env python

#rebecca's new addition 1/5/2017
#states: no object, distance to object, contact
import numpy as np
import rospy
from std_msgs.msg import Int32MultiArray, Header

class FingerSensorsStates(object):
    def __init__(self, topic='/sensor_values'):
        rospy.init_node("finger_sensors_states")
        self.inside_left = np.zeros(7)
        self.inside_right = np.zeros(7)
        self.inside = np.zeros(14)
        self.tip = np.zeros(2)
        self.inside_offset = np.zeros_like(self.inside)
        self.tip_offset = np.zeros_like(self.tip)
        self.sensor_sub = rospy.Subscriber(topic,
                                           Int32MultiArray,
                                           self.update_sensor_values,
                                           queue_size=1)

    def update_sensor_values(self, msg):
        values = np.array(msg.data)
        self.inside_left = values[:7]
        self.inside_right = values[8:15]
        self.inside = np.concatenate((values[:7],
                                      values[8:15])) - self.inside_offset
        self.tip = values[[7, 15]] - self.tip_offset
        error = np.mean(self.inside[7:] - self.inside[:7])
        # Experimentally, we need a deadband of about
        # 314-315. Otherwise it moves left or right with nothing in
        # between fingers.
        self.determine_contact();
        if abs(error) < 350:
            self.error = 0
        else:
            self.error = error
            
    def determine_contact(self):
        for value in self.inside_left:
            if(value > 20000):
                print("left gripper contact!")
                break
        if self.tip[0] > 20000:
            print("left tip contact!")
            
        for value in self.inside_right:
            if(value > 20000):
                print("right gripper contact!")
                break
        if self.tip[1] > 20000:
            print("right tip contact!")
            
    def calculate_distance(self, sensor_reading):
        #from the paper:
        #x = 1/(((y-c)/a)^(1/b))
        return
            
            
if __name__ == '__main__':
    n = FingerSensorsStates()
    rospy.spin()
