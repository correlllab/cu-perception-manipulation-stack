#! /usr/bin/env python
from __future__ import division, print_function, absolute_import
import rospy
from pap.robot import Jaco
from pap.manager import PickAndPlaceNode

from std_msgs.msg import Header, Int64, Bool
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion

import tf

class pick_peas_class(object):
    def __init__(self):
        self.j = Jaco()
        self.listen = tf.TransformListener()

    def pick_spoon(self):
        if self.listen.frameExists("/root") and self.listen.frameExists("/spoon_position"):
            # print ("we have the frame")
            t = self.listen.getLatestCommonTime("/root", "/spoon_position")
            translation, quaternion = self.listen.lookupTransform("/root", "/spoon_position", t)

            self.spoon_pose = PoseStamped(
                Header(0, rospy.Time(0), 'spoon_position'),
                Pose(Point(float(translation[0]),float(translation[1]),float(translation[2])),
                     Quaternion(float(quaternion[0]),float(quaternion[1]),float(quaternion[2]),float(quaternion[3]))))
            self.j.move_ik(self.spoon_pose)
        else:
            print ("we dont have the frame")

    def move_calib_position(self):
        # move arm to the calibration position
        self.calib_pose = PoseStamped(
            Header(0, rospy.Time(0), self.j.base),
            Pose(Point(0.3424,-0.1766,0.0381),
                 Quaternion(0.1802, 0.6434, 0.7075, 0.2299)))
        self.j.move_ik(self.calib_pose)



if __name__ == '__main__':
    rospy.init_node("task_1")
    # n = PickAndPlaceNode(Jaco)
    p = pick_peas_class()
    # p.move_calib_position()
    # p.j.home()
    while not rospy.is_shutdown():
        p.pick_spoon()
    # rospy.spin()
