#!/usr/bin/env python
import rospy
import tf

from std_msgs.msg import Header, Int64, Bool
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from keyboard.msg import Key
import numpy as np

class grasp_generator(object):
    def __init__(self):
        self.listen = tf.TransformListener()
        self.broadcast = tf.TransformBroadcaster()

    # keyboard_sub = rospy.Subscriber("/keyboard/keyup",
    #                                          Key,
    #                                          handle_keyboard,
    #                                          queue_size=1)
        self.obj_pose_sub = rospy.Subscriber("/num_objects", Int64,
                                          self.broadcast_frame,
                                          queue_size=1)
      # def handle_keyboard(self, key):
      #    character = chr(key.code)

    def broadcast_frame(self,msg):
        self.num_objects = msg.data

        if self.listen.frameExists("/root") and self.listen.frameExists("/object_pose_4"):
            # print ('we have the frame')
            t = self.listen.getLatestCommonTime("/root", "/object_pose_4")
            translation, quaternion = self.listen.lookupTransform("/root", "/object_pose_4", rospy.Time(0))
            # print '------ \n'
            # print 'Translation BEFORE'
            # print translation,quaternion
            matrix1 = self.listen.fromTranslationRotation(translation, quaternion)
            # print '\n'
            # print 'this is the matrix BEFORE'
            # print (matrix1)

            # Identity matrix
            requrd_rot = (3.14,0,0) # in radians
            requrd_trans = (0,0.05,0.09)
            #euler to quaternion
            requrd_quat = tf.transformations.quaternion_from_euler(requrd_rot[0], requrd_rot[1], requrd_rot[2])
            # print (requrd_quat)
            matrix2 = self.listen.fromTranslationRotation(requrd_trans, requrd_quat) #identity matrix
            # print (matrix2)
            matrix3 =  np.zeros((4,4))
            matrix3 = np.dot(matrix1,matrix2)
            # print '\n'
            # print 'this is the matrix AFTER'
            # print (matrix3)

            #get the euler angles from 4X4 T martix
            scale, shear, rpy_angles, trans_1, perps = tf.transformations.decompose_matrix(matrix3)

            # covert quaternion to euler
            quat_1 = tf.transformations.quaternion_from_euler(rpy_angles[0], rpy_angles[1], rpy_angles[2])
            # print '\n'
            # print 'Decomposing the AFTER matrix'
            # print trans, quaternion2
            # print '------ \n'

            #converting numpy.ndarray into tuple
            trans_1 = tuple(trans_1.tolist())
            quat_1 = tuple(quat_1)
            # print 'Transltaion AFTER'
            # print trans, quaternion2

            self.broadcast.sendTransform(trans_1,
                                    quat_1,
                                    rospy.Time.now(),
                                    "spoon_position",
                                    "root")

        if self.listen.frameExists("/root") and self.listen.frameExists("/object_pose_2"):
            t = self.listen.getLatestCommonTime("/root", "/object_pose_2")
            translation, quaternion = self.listen.lookupTransform("/root", "/object_pose_2", rospy.Time(0))

            matrix1 = self.listen.fromTranslationRotation(translation, quaternion)

            # Identity matrix
            requrd_rot = (-3.14,0,1.5) # in radians
            requrd_trans = (0,0.05,0.09)
            #euler to quaternion
            requrd_quat = tf.transformations.quaternion_from_euler(requrd_rot[0], requrd_rot[1], requrd_rot[2])
            # print (requrd_quat)
            matrix2 = self.listen.fromTranslationRotation(requrd_trans, requrd_quat) #identity matrix

            matrix3 =  np.zeros((4,4))
            matrix3 = np.dot(matrix1,matrix2)

            #get the euler angles from 4X4 T martix
            scale, shear, rpy_angles, trans_1, perps = tf.transformations.decompose_matrix(matrix3)

            # covert quaternion to euler
            quat_1 = tf.transformations.quaternion_from_euler(rpy_angles[0], rpy_angles[1], rpy_angles[2])

            #converting numpy.ndarray into tuple
            trans_1 = tuple(trans_1.tolist())
            quat_1 = tuple(quat_1)

            self.broadcast.sendTransform(trans_1,
                                    quat_1,
                                    rospy.Time.now(),
                                    "bowl_position",
                                    "/root")



if __name__ == '__main__':
    rospy.init_node("grasp_generator")
    gg = grasp_generator()
    rospy.spin()
