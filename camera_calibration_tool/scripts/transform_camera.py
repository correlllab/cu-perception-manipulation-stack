#!/usr/bin/env python
import rospy
import tf

if __name__ == '__main__':
    rospy.init_node("camera_link_transform")
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    translation = (-0.05184876843938464, 0.39596451763428525, 0.8525322633373289)
    quaternion = (0.1500047082842217, 0.36815132294890407, -0.2298717566510456, 0.8883254844918611)
    while not rospy.is_shutdown():
        br.sendTransform(translation,
                        quaternion,
                        rospy.Time.now(),
                        "/camera_link",
                        "/root")
    rospy.spin()
