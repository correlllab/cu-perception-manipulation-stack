#!/usr/bin/env python
from __future__ import division, print_function

import numpy as np
import rospy
import actionlib
from std_msgs.msg import Header
from geometry_msgs.msg import Point, Quaternion, Pose, PoseStamped

# Not used currently, but interesting module!
# from moveit_commander import conversions

# Apparently tf was deprecated a long time ago? And we should use tf2?
import tf
# import tf2_ros

from .utils import Point2list

class Robot(object):
    def __init__(self, base):
        """base : str
            Robot base/root tf"""
        self.base = base
        self.tl = tf.TransformListener()
        # self.home()

    def pick(self, pose, direction=(0, 0, 1), distance=0.1):
        """Go to pose + pick_direction * pick_distance, open, go to pose,
        close, go to pose + pick_direction * pick_distance.

        """
        pregrasp_pose = self.translate(pose, direction, distance)

        self.move_ik(pregrasp_pose)
        rospy.sleep(0.5)
        # We want to block end effector opening so that the next
        # movement happens with the gripper fully opened. In Baxter,
        # that involves sublcassing the gripper class
        self.gripper.open()
        self.move_ik(pose)
        rospy.sleep(0.5)
        self.gripper.close()
        rospy.sleep(0.5)
        self.move_ik(pregrasp_pose)

    def place(self, pose, direction=(0, 0, 1), distance=0.1):
        """Go to pose + place_direction * place_distance, go to pose,
        open, go to pose + pick_direction * pick_distance.

        """
        preplace_pose = self.translate(pose, direction, distance)
        self.move_ik(preplace_pose)
        rospy.sleep(0.5)
        self.move_ik(pose)
        rospy.sleep(0.5)
        self.gripper.open()
        rospy.sleep(0.5)
        self.move_ik(preplace_pose)

    def translate(self, pose, direction, distance):
        """Get an PoseStamped msg and apply a translation direction * distance

        """
        # Let's get the pose, move it to the tf frame, and then
        # translate it
        base_pose = self.tl.transformPose(self.base, pose)
        base_pose.pose.position = Point(*np.array(direction) * distance +
                                        Point2list(base_pose.pose.position))
        return base_pose


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(
        description='Use baxter to pick and place objects')
    parser.add_argument(
        '-l', '--limb', choices=('left', 'right'),
        help='Choose which limb to use for picking and placing')
    parser.add_argument(
        '-r', '--reset', action='store_true', default=False)

    args = parser.parse_args(rospy.myargv()[1:])

    # main(args.limb, args.reset)
    # c = PickAndPlaceNode('left')
    rospy.spin()
