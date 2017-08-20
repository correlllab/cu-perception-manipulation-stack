#!/usr/bin/env python
import rospy
import tf

if __name__ == '__main__':
    rospy.init_node("camera_link_transform")
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    translation = (-0.0071, 0.4955, 0.27) #1.0763, 1.0109, 0.89861
    quaternion = (0.0527, 0.2445, -0.2311, 0.9402)
    while not rospy.is_shutdown():
        br.sendTransform(translation,
                        quaternion,
                        rospy.Time.now(),
                        "/camera_link",
                        "/root")
    rospy.spin()
