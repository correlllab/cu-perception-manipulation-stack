#!/usr/bin/env python
import rospy
import tf

if __name__ == '__main__':
    rospy.init_node("camera_link_transform")
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    translation = (0.0709, 0.5315, 0.3215) #1.0763, 1.0109, 0.89861
    quaternion = (-0.0724, -0.2635, 0.2591, -0.9264)
    while not rospy.is_shutdown():
        br.sendTransform(translation,
                        quaternion,
                        rospy.Time.now(),
                        "/camera_link",
                        "/root")
    rospy.spin()
