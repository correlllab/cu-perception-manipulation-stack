#!/usr/bin/env python
from __future__ import division, print_function, absolute_import

from collections import defaultdict

import rospy

import tf

from std_msgs.msg import Header, Int64, Bool
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from keyboard.msg import Key

class Manager(object):
    def __init__(self, name):
        rospy.init_node(name)

        self.transition_table = None
        self.state = 'initial'
	rospy.loginfo("manager.py")
        self.keyboard_sub = rospy.Subscriber("/keyboard/keyup",
                                             Key,
                                             self.handle_keyboard,
                                             queue_size=1)

    def handle_keyboard(self, key):
        self.character = chr(key.code)
        try:
            next_state = self.transition_table[self.state][self.character]
        except KeyError:
            rospy.logerr(
                "Make sure input ({}) given"
                " state ({}) is valid!".format(input, self.state)
            )
            return

        next_state()
        self.show_options()

    def show_options(self):
        rospy.loginfo("Possible options are: {}".format(
            ', '.join(self.transition_table[self.state].keys())))


class PerceptionNode(Manager):
    def __init__(self):
        super(PerceptionNode, self).__init__('p_node')

        self.transition_table = {
            # If calibration has already happened, allow skipping it
            'initial': {'q': self._perceive},
            'perceive': {'q': self._post_perceive},
            }

        # Hardcoded place for now
        self.tl = tf.TransformListener()
        self.num_objects = 0
        # Would this work too? Both tf and tf2 have (c) 2008...
        # self.tf2 = tf2_ros.TransformListener()
        self.n_objects_sub = rospy.Subscriber("/num_objects", Int64,
                                              self.update_num_objects,
                                              queue_size=1)
        self.perception_pub = rospy.Publisher("/perception/enabled",
                                              Bool,
                                              queue_size=1)

        self.br = tf.TransformBroadcaster()

        # Dict to map imarker names and their updated poses
        self.int_markers = {}

        # Ideally this call would be in a Factory/Metaclass/Parent
        self.show_options()

    def update_num_objects(self, msg):
        self.num_objects = msg.data

    def _perceive(self):
        self.state = "perceive"
        rospy.loginfo("Asking for perception...")
        self.perception_pub.publish(Bool(True))

    def _post_perceive(self):
        self.state = "post_perceive"
        rospy.loginfo("Asking to stop perception...")
        self.perception_pub.publish(Bool(False))

    def _level(self):
        self.robot.level = int(self.character)
        self.state = 'level'

if __name__ == '__main__':
    n = PerceptionNode()
    rospy.spin()
