import rospy
import tf

from std_msgs.msg import Header, Int64, Bool
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from keyboard.msg import Key

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
    # try:
    #     obj_to_get = int(character)
    # except ValueError:
    #     rospy.logerr("Please provide a number in picking mode")
    #     return
    #
    # frame_name = "object_pose_{}".format(obj_to_get)
    #
    # rospy.loginfo("Picking object {}...".format(obj_to_get))
        if self.listen.frameExists("/root") and self.listen.frameExists("/object_pose_4"):
            t = self.listen.getLatestCommonTime("/root", "/object_pose_4")
            rdhn_translation, rdhn_quaternion = self.listen.lookupTransform("/root", "/object_pose_4", t)

            self.broadcast.sendTransform(rdhn_translation,
                                    rdhn_quaternion,
                                    rospy.Time.now(),
                                    "cup_position",
                                    "/root")

            rdhn_translation_list = list(rdhn_translation)
            rdhn_translation_list[0] -= 0.03
            rdhn_translation_list[1] -= 0.05
            rdhn_translation_list[2] += 0.09
            rdhn_translation = tuple(rdhn_translation_list)

            rdhn_quaternion_list = list(rdhn_quaternion)
            rdhn_quaternion_list[0] += 0
            rdhn_quaternion_list[1] += 6
            rdhn_quaternion_list[2] += 0
            rdhn_quaternion_list[3] += 0
            rdhn_quaternion = tuple(rdhn_quaternion_list)

            print rdhn_translation, rdhn_quaternion
            # cup_frame = tf.TransformerROS().fromTranslationRotation(rdhn_translation,   #convert trans and quat in 4X4matrix
            #                                                         rdhn_quaternion)
            # # print my_frame
            # spoon_offset = tf.TransformerROS().fromTranslationRotation((0, 0, 0),        #convert trans and quat in 4X4matrix
            #                                                             (0, 1, 0, 0))
            # # print spoon_offset
            # spoon_location = spoon_offset*cup_frame
            # # print tf.transformations.translation_from_matrix(spoon_location)
            # self.broadcast.sendTransform(tf.transformations.translation_from_matrix(spoon_location),  #quaternion calculation is not accurate from the 4X4 MATRIX
            #                         tf.transformations.quaternion_from_matrix(spoon_location),
            #                         rospy.Time.now(),
            #                         "spoon_location",
            #                         "/root")

            self.broadcast.sendTransform(rdhn_translation,
                                    rdhn_quaternion,
                                    rospy.Time.now(),
                                    "spoon_position",
                                    "/root")

            # spoon_offset = PoseStamped(
            #     Header(0, rospy.Time(0), "/root"),
            #     Pose(Point(0.99, 0.85, 1),
            #          Quaternion(1, 0, 0, 0)))



if __name__ == '__main__':
    rospy.init_node("grasp_generator")
    gg = grasp_generator()
    rospy.spin()
