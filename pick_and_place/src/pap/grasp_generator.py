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

        if self.listen.frameExists("/root") and self.listen.frameExists("/object_pose_1"):
            # print ('we have the frame')
            t = self.listen.getLatestCommonTime("/root", "/object_pose_1")
            translation, quaternion = self.listen.lookupTransform("/root", "/object_pose_1", rospy.Time(0))
            print 'Translation BEFORE'
            print translation,quaternion
            matrix1 = self.listen.fromTranslationRotation(translation, quaternion)
            print 'this is the matrix BEFORE'
            print (matrix1)
            # Identity matrix
            requrd_rot = (0,0,0)
            requrd_trans = (0,0,0)
            #euler to quaternion
            requrd_quat = tf.transformations.quaternion_from_euler(requrd_rot[0], requrd_rot[1], requrd_rot[2])
            # print (requrd_quat)
            matrix2 = self.listen.fromTranslationRotation(requrd_trans, requrd_quat) #identity matrix
            # print (matrix2)
            matrix3 =  np.zeros((4,4))
            matrix3 = np.dot(matrix2,matrix1)
            print 'this is the matrix AFTER'
            print (matrix3)
            #get the euler angles from 4X4 T martix
            scale, shear, rpy_angles, trans, perps = tf.transformations.decompose_matrix(matrix3)

            # covert quaternion to euler
            quaternion2 = tf.transformations.quaternion_from_euler(rpy_angles[0], rpy_angles[1], rpy_angles[2])
            print 'Decomposing the AFTER matrix'
            print trans, quaternion2
            #converting numpy.ndarray into tuple
            trans = tuple(trans.tolist())
            quaternion2 = tuple(quaternion2)
            # print 'Transltaion AFTER'
            # print trans, quaternion2

            self.broadcast.sendTransform(trans,
                                    quaternion2,
                                    rospy.Time.now(),
                                    "spoon_position",
                                    "object_pose_1")

        if self.listen.frameExists("/root") and self.listen.frameExists("/object_pose_4"):
            t = self.listen.getLatestCommonTime("/root", "/object_pose_4")
            translation, quaternion = self.listen.lookupTransform("/root", "/object_pose_4", rospy.Time(0))

            translation_list = list(translation)
            translation_list[0] -= 0.03
            translation_list[1] -= 0.05
            translation_list[2] += 0.19
            translation = tuple(translation_list)

            quaternion_list = list(quaternion)
            quaternion_list[0] += 0
            quaternion_list[1] += 0
            quaternion_list[2] += 0
            quaternion_list[3] += 0
            quaternion = tuple(quaternion_list)

            # print translation, quaternion

            self.broadcast.sendTransform(translation,
                                    quaternion,
                                    rospy.Time.now(),
                                    "bowl_position",
                                    "/root")



if __name__ == '__main__':
    rospy.init_node("grasp_generator")
    gg = grasp_generator()
    rospy.spin()
