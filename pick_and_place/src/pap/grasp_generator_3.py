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

        self.obj_pose_sub = rospy.Subscriber("/num_objects", Int64,
                                          self.broadcast_frame,
                                          queue_size=1)

    def getOffsetPoses(self, translation, quaternion, requrd_rot, requrd_trans):
        # print ('we are in the fucntion')
        # return 0
        matrix1 = self.listen.fromTranslationRotation(translation, quaternion)
        requrd_quat = tf.transformations.quaternion_from_euler(requrd_rot[0], requrd_rot[1], requrd_rot[2])
        # print (requrd_quat)
        matrix2 = self.listen.fromTranslationRotation(requrd_trans, requrd_quat) #identity matrix
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

    def broadcast_frame(self,msg):
        self.num_objects = msg.data
        # print ('we have the frame')
        if self.listen.frameExists("/root") and self.listen.frameExists("/unknown_2"):
            # print ('we have the frame')
            t = self.listen.getLatestCommonTime("/root", "/unknown_2")
            translation, quaternion = self.listen.lookupTransform("/root", "/unknown_2", rospy.Time(0))

            # Identity matrix. Set the requ rot n trans wrt obj frame
            requrd_rot = (1.57,0,1.4) # in radians
            requrd_trans = (-0.06,-0.08,0.1)
            # calculate and get an offset frame w/o ref to objct frame
            pose = self.getOffsetPoses(translation, quaternion, requrd_rot, requrd_trans)
            trans_1= tuple(pose[:3])
            quat_1= tuple(pose[3:])

            self.broadcast.sendTransform(trans_1, quat_1,
                                    rospy.Time.now(),
                                    "spoon_position",
                                    "root")

        if self.listen.frameExists("/root") and self.listen.frameExists("/unknown_1"):
            t = self.listen.getLatestCommonTime("/root", "/unknown_1")
            translation, quaternion = self.listen.lookupTransform("/root", "/unknown_1", rospy.Time(0))

            # Identity matrix. Set the requ rot n trans wrt obj frame
            requrd_rot = (1.57,0,1.4) # in radians
            requrd_trans = (-0.05,-0.04,0.23)
            # calculate and get an offset frame w/o ref to objct frame
            pose = self.getOffsetPoses(translation, quaternion, requrd_rot, requrd_trans)
            trans_1= tuple(pose[:3])
            quat_1= tuple(pose[3:])

            self.broadcast.sendTransform(trans_1, quat_1,
                                    rospy.Time.now(),
                                    "stir_position",
                                    "/root")




if __name__ == '__main__':
    rospy.init_node("grasp_generator")
    gg = grasp_generator()
    rospy.spin()
