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
        matrix3 = np.dot(matrix1,matrix2)
        #get the euler angles from 4X4 T martix
        scale, shear, rpy_angles, trans_1, perps = tf.transformations.decompose_matrix(matrix3)
        # covert quaternion to euler
        quat_1 = tf.transformations.quaternion_from_euler(rpy_angles[0], rpy_angles[1], rpy_angles[2])

        #converting numpy.ndarray into tuple
        trans_1 = list(trans_1.tolist())
        quat_1 = list(quat_1)
        pose = trans_1 + quat_1
        return pose

    def broadcast_frame(self):
        # self.num_objects = msg.data
        # print ('we do not have the frame')
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            try:
                trans = self.tfBuffer.lookup_transform('root', 'white_cup_position', rospy.Time())
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rate.sleep()
                continue
            # print (trans.transform.translation.x)
            translation  = [trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z]
            rotation = [trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w]

            # Identity matrix. Set the requ rot n trans wrt obj frame
            requrd_rot = (3.14,0,0) # in radians
            requrd_trans = (-0.02,0,0.07)
            requrd_trans = tuple(x for x in requrd_trans)
            # calculate and get- an offset frame w/o ref to objct frame
            pose = self.getOffsetPoses(translation, rotation, requrd_rot, requrd_trans)
            trans_1= tuple(pose[:3])
            quat_1= tuple(pose[3:])

            self.broadcast.sendTransform(trans_1, quat_1,
                                    rospy.Time.now(),
                                    "teaCup_position",
                                    "root")

            try:
                trans = self.tfBuffer.lookup_transform('root', 'saucer_position', rospy.Time())
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rate.sleep()
                continue

            translation  = [trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z]
            rotation = [trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w]
            # Identity matrix. Set the requ rot n trans wrt obj frame
            requrd_rot = (3.14,0,0) # in radians
            requrd_trans = (0,0.02,0.13)
            # calculate and get an offset frame w/o ref to objct frame
            pose = self.getOffsetPoses(translation, rotation, requrd_rot, requrd_trans)
            trans_1= tuple(pose[:3])
            quat_1= tuple(pose[3:])

            self.broadcast.sendTransform(trans_1, quat_1,
                                    rospy.Time.now(),
                                    "cupPlace_position",
                                    "root")

            rate.sleep()



if __name__ == '__main__':
    rospy.init_node("grasp_generator")
    gg = grasp_generator()
    # rate = rospy.Rate(100)
    gg.broadcast_frame()
    rospy.spin()
