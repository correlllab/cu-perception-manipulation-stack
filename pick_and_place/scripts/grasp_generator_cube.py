#!/usr/bin/env python
import rospy
import tf

from std_msgs.msg import Header, Int64, Bool, String
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from keyboard.msg import Key
import numpy as np

class GraspGeneratorCube(object):
    def __init__(self):
        self.listen = tf.TransformListener()
        self.broadcast = tf.TransformBroadcaster()

        self.obj_pose_sub = rospy.Subscriber("/pick_frame_name", String,
                                          self.set_pick_frame,
                                          queue_size=1)

        self.obj_pose_sub = rospy.Subscriber("/place_frame_name", String,
                                          self.set_place_frame,
                                          queue_size=1)

        self.obj_pose_sub = rospy.Subscriber("/num_objects", Int64,
                                  self.broadcast_frames,
                                  queue_size=1)

        self.place_frame = ''
        self.pick_frame = ''
        self.tower_size = 1
        self.tower_size_sub = rospy.Subscriber("/tower_size",Int64,self.set_tower_size)

    def set_tower_size(self,msg):
        self.tower_size = msg.data

    def set_place_frame(self,msg):
        self.place_frame = msg.data

    def set_pick_frame(self,msg):
        self.pick_frame = msg.data

    def getOffsetPoses(self, translation, quaternion, requrd_rot, requrd_trans):
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

    def generate_place_frame(self):
        place_obj_name = self.place_frame
        if self.listen.frameExists("/root") and self.listen.frameExists(place_obj_name):
            translation, quaternion = self.listen.lookupTransform("/root", place_obj_name, rospy.Time(0))
            # Identity matrix. Set the requ rot n trans wrt obj frame
            requrd_rot = (np.pi,0,0) # in radians
            requrd_trans = (0,.035,0.1+translation[2]+self.tower_size*0.05)
            # calculate and get an offset frame w/o ref to objct frame
            pose = self.getOffsetPoses(translation, quaternion, requrd_rot, requrd_trans)
            trans_1= tuple(pose[:3])
            quat_1= tuple(pose[3:])
            self.broadcast.sendTransform(trans_1, quat_1,
                                    rospy.Time.now(),
                                    "/place_frame",
                                    "/root")

    def get_camera_frame(self):
        frame = "/camera_link"
        if self.listen.frameExists("/root") and self.listen.frameExists(frame):
            translation, quaternion = self.listen.lookupTransform("/root", frame, rospy.Time(0))
            print(translation)
            print(quaternion)

    def generate_pick_frame(self):
        pick_obj_name = self.pick_frame
        if self.listen.frameExists("/root") and self.listen.frameExists(pick_obj_name):
            translation, quaternion = self.listen.lookupTransform("/root", pick_obj_name, rospy.Time(0))
            # Identity matrix. Set the requ rot n trans wrt obj frame
            requrd_rot = (np.pi,0,0) # in radians
            requrd_trans = (0,.035,0.1+translation[2])
            # calculate and get an offset frame w/o ref to objct frame
            pose = self.getOffsetPoses(translation, quaternion, requrd_rot, requrd_trans)
            trans_1= tuple(pose[:3])
            quat_1= tuple(pose[3:])
            self.broadcast.sendTransform(trans_1, quat_1,
                                    rospy.Time.now(),
                                    "/pick_frame",
                                    "/root")

    def broadcast_frames(self,msg):
        self.generate_place_frame()
        self.generate_pick_frame()
        self.get_camera_frame()


    def getOffsetPoses(self, translation, quaternion, requrd_rot, requrd_trans):
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






if __name__ == '__main__':
    rospy.init_node("grasp_generator")
    gg = GraspGeneratorCube()
    rospy.spin()
