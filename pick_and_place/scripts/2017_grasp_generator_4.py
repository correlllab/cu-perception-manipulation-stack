#!/usr/bin/env python
import rospy
import tf2_ros
import tf

from std_msgs.msg import Header, Int64, Bool
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from keyboard.msg import Key
import numpy as np

class grasp_generator(object):
    def __init__(self):
        self.tfBuffer = tf2_ros.Buffer()
        self.listen = tf2_ros.TransformListener(self.tfBuffer)

        self.listener = tf.TransformListener()
        self.broadcast = tf.TransformBroadcaster()


    def getOffsetPoses(self, translation, quaternion, requrd_rot, requrd_trans):
        matrix1 = self.listener.fromTranslationRotation(translation, quaternion)
        requrd_quat = tf.transformations.quaternion_from_euler(requrd_rot[0], requrd_rot[1], requrd_rot[2])
        # print (requrd_quat)
        matrix2 = self.listener.fromTranslationRotation(requrd_trans, requrd_quat) #identity matrix
        # print (matrix2)
        matrix3 =  np.zeros((4,4))
        matrix3 = np.dot(matrix1, matrix2)
        #get the euler angles from 4X4 T martix
        scale, shear, rpy_angles, trans_1, perps = tf.transformations.decompose_matrix(matrix3)
        # covert quaternion to euler
        quat_1 = tf.transformations.quaternion_from_euler(rpy_angles[0], rpy_angles[1], rpy_angles[2])

        #converting numpy.ndarray into tuple
        trans_1 = list(trans_1.tolist())
        quat_1 = list(quat_1)
        pose = trans_1 + quat_1
        return pose

    def broadcast_frame(self, from_frame, to_frame, requrd_rot, requrd_trans):
            trans = self.tfBuffer.lookup_transform('root', from_frame, rospy.Time())
            translation  = [trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z]
            rotation = [trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w]
            requrd_trans = tuple(x for x in requrd_trans)
            # calculate and get- an offset frame w/o ref to objct frame
            pose = self.getOffsetPoses(translation, rotation, requrd_rot, requrd_trans)
            trans_1= tuple(pose[:3])
            quat_1= tuple(pose[3:])
            self.broadcast.sendTransform(trans_1, quat_1,rospy.Time.now(),to_frame,"root")


if __name__ == '__main__':
    rospy.init_node("grasp_generator")
    gg = grasp_generator()
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        try:
            gg.broadcast_frame("green_cup_with_spoon_position", 'pour_3', (1.5, 3, -3), (0.15, 0.0, 0.2546))
            gg.broadcast_frame("unknown_2", 'pour_2', (1.5, 3, -3), (0.15, 0.0, 0.2546))
            gg.broadcast_frame("unknown_1", 'pour_1', (1.5, 3, -3), (0.15, 0.0, 0.2546))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue
    rospy.spin()
