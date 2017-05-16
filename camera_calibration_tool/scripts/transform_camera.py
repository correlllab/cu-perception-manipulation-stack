#!/usr/bin/env python
import rospy
import tf
from geometry_msgs.msg import Pose, Point, Quaternion

#4/2/2017, p good
#translation = ( 1.2323, -0.24536, 0.66794)
#quaternion = (-0.45097, 0.0087127, 0.89084, -0.054341)

translation = (1.1163, 1.0209, 0.95861) #1.0763, 1.0109, 0.89861
quaternion = (0.27218, 0.32564, -0.72603, 0.54107)

def update_transform(pose):
    global translation
    global quanternion
    print("Updating transformation...")
    translation = (pose.position.x, pose.position.y, pose.position.z)
    quaternion = (pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w)
    print(translation)
    print(quaternion)


if __name__ == '__main__':
    rospy.init_node("camera_link_transform")

    obj_pose_sub = rospy.Subscriber("/update_camera_alignment", Pose,
                                          update_transform,
                                          queue_size=1)
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    #manual translation: 1.2496, -0.26179, 0.67349
     #1.2396, -0.25679, 0.65349
    #quaternion = (-0.46737, -0.0041255, 0.88405, -0.0024514)
    #modifications: -0.44737, 0.0141255, 0.88405, -0.0024514
    
    while not rospy.is_shutdown():
        br.sendTransform(translation,
                        quaternion,
                        rospy.Time.now(),
                        "/camera_link",
                        "/root")

    
    rospy.spin()
