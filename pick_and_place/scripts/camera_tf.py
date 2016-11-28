#!/usr/bin/env python
import rospy
import tf
from std_msgs.msg import Header, Int64, Bool, String
import numpy as np

class CameraFrame(object):
    def __init__(self):
        self.listen = tf.TransformListener()

        self.obj_pose_sub = rospy.Subscriber("/num_objects", Int64,
                              self.get_frame,
                              queue_size=1)

    def get_frame(self,msg):
        frame1 = '/root'
        frame2 = 'camera_link'
        if self.listen.frameExists(frame1) and self.listen.frameExists(frame2):
            translation, quaternion = self.listen.lookupTransform(frame1, frame2, rospy.Time.now())
            print(translation)
            print(quaternion)

if __name__ == '__main__':
    rospy.init_node("camera_frame")
    cf = CameraFrame()
    rospy.spin()
