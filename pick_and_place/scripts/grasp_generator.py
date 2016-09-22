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

        if self.listen.frameExists("/root") and self.listen.frameExists("/object_pose_4"):
            t = self.listen.getLatestCommonTime("/root", "/object_pose_4")
            translation, quaternion = self.listen.lookupTransform("/root", "/object_pose_4", t)

            # self.broadcast.sendTransform(rdhn_translation,
            #                         rdhn_quaternion,
            #                         rospy.Time.now(),
            #                         "cup_position",
            #                         "/root")

            translation_list = list(translation)
            translation_list[0] -= 0.03
            translation_list[1] -= 0.05
            translation_list[2] += 0.09
            translation = tuple(translation_list)

            quaternion_list = list(quaternion)
            quaternion_list[0] += 0.7071
            quaternion_list[1] += -0.4071
            quaternion_list[2] += -0.97
            quaternion_list[3] += 0
            quaternion = tuple(quaternion_list)

            print translation, quaternion

            self.broadcast.sendTransform(translation,
                                    quaternion,
                                    rospy.Time.now(),
                                    "spoon_position",
                                    "/root")

        if self.listen.frameExists("/root") and self.listen.frameExists("/object_pose_1"):
            t = self.listen.getLatestCommonTime("/root", "/object_pose_1")
            translation, quaternion = self.listen.lookupTransform("/root", "/object_pose_1", t)

            # self.broadcast.sendTransform(rdhn_translation,
            #                         rdhn_quaternion,
            #                         rospy.Time.now(),
            #                         "cup_position",
            #                         "/root")

            translation_list = list(translation)
            translation_list[0] -= 0.03
            translation_list[1] -= 0.05
            translation_list[2] += 0.09
            translation = tuple(translation_list)

            quaternion_list = list(quaternion)
            quaternion_list[0] += 0
            quaternion_list[1] += 0
            quaternion_list[2] += 0
            quaternion_list[3] += 0
            quaternion = tuple(quaternion_list)

            print translation, quaternion

            self.broadcast.sendTransform(translation,
                                    quaternion,
                                    rospy.Time.now(),
                                    "bowl_position",
                                    "/root")



if __name__ == '__main__':
    rospy.init_node("grasp_generator")
    gg = grasp_generator()
    rospy.spin()
