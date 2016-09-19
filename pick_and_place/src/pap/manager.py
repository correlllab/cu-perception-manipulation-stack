from __future__ import division, print_function, absolute_import

from collections import defaultdict

import rospy

import tf

from std_msgs.msg import Header, Int64, Bool
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from keyboard.msg import Key

from interactive_markers.interactive_marker_server import (
    InteractiveMarkerServer
    )

from .interactive_marker import make_interactive_marker
from .utils import Point2list, Quaternion2list


class Manager(object):
    def __init__(self, name):
        rospy.init_node(name)

        self.transition_table = None
        self.state = 'initial'

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


class PickAndPlaceNode(Manager):
    def __init__(self, robot, *robotargs):
        super(PickAndPlaceNode, self).__init__('pp_node')

        _post_perceive_trans = defaultdict(lambda: self._pick)
        _post_perceive_trans.update({'c': self._calibrate})
        _preplace = defaultdict(lambda: self._preplace)
        self.transition_table = {
            # If calibration has already happened, allow skipping it
            'initial': {'c': self._calibrate, 'q': self._perceive,
                        's': self._preplace},
            'calibrate': {'q': self._perceive, 'c': self._calibrate},
            'perceive': {'q': self._post_perceive},
            'post_perceive': _post_perceive_trans,
            'postpick': {'1': self._level, '2': self._level, '9': self._level},
            'level': _preplace,
            'preplace': {'s': self._place},
            'place': {'q': self._perceive, 'c': self._calibrate}
            }

        if callable(robot):
            self.robot = robot(*robotargs)
        else:
            self.robot = robot
        self.robot.level = 1

        # Hardcoded place for now
        self.place_pose = PoseStamped(
            Header(0, rospy.Time(0), self.robot.base),
            Pose(Point(0.526025806, 0.4780144, -0.161326153),
                 Quaternion(1, 0, 0, 0)))
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
        self.alignment_pub = rospy.Publisher("/alignment/doit",
                                             Bool,
                                             queue_size=1)
        self.br = tf.TransformBroadcaster()

        self.int_marker_server = InteractiveMarkerServer('int_markers')
        # Dict to map imarker names and their updated poses
        self.int_markers = {}

        # Ideally this call would be in a Factory/Metaclass/Parent
        self.show_options()

    def update_num_objects(self, msg):
        self.num_objects = msg.data

    def _calibrate(self):
        self.state = "calibrate"
        self.alignment_pub.publish(Bool(True))

    def _perceive(self):
        self.state = "perceive"
        rospy.loginfo("Asking for perception...")
        self.perception_pub.publish(Bool(True))

    def _post_perceive(self):
        self.state = "post_perceive"
        rospy.loginfo("Asking to stop perception...")
        self.perception_pub.publish(Bool(False))

    def _preplace(self):
        # State not modified until placing succeeds
        try:
            obj_to_get = int(self.character)
        except ValueError:
            rospy.logerr("Please provide a number in placing mode")
            return
        self.state = "preplace"
        frame_name = "object_pose_{}".format(obj_to_get)

        rospy.loginfo(
            "Placing object on top of object {}...".format(obj_to_get))
        if self.tl.frameExists(self.robot.base) and self.tl.frameExists(frame_name):
            t = self.tl.getLatestCommonTime(self.robot.base, frame_name)
            position, quaternion = self.tl.lookupTransform(self.robot.base,
                                                           frame_name,
                                                           t)
            position = list(position)
            # Height of cubelet
            position[2] += self.robot.level * 0.042
            # YCB
            # position[2] += self.robot.level * 0.026
            # Update pose position from perception
            self.place_pose.pose.position = Point(*position)

        rospy.loginfo("Adjusting place position...")
        imarker_name = 'place_pose'
        self.int_markers[imarker_name] = self.place_pose
        imarker = make_interactive_marker(imarker_name,
                                          self.place_pose.pose)
        # TODO delete imarker at post place
        self.int_marker_server.insert(imarker, self.imarker_callback)
        self.int_marker_server.setCallback(imarker_name, self.imarker_callback)
        self.int_marker_server.applyChanges()
        rospy.loginfo("imarker stuff done")

    def imarker_callback(self, msg):
        # http://docs.ros.org/jade/api/visualization_msgs/html/msg/InteractiveMarkerFeedback.html # noqa
        rospy.loginfo('imarker_callback called')
        name = msg.marker_name
        new_pose = msg.pose
        self.int_markers[name] = PoseStamped(msg.header, new_pose)

    def _place(self):
        self.state = "place"
        rospy.loginfo("Placing...")

        place_pose = self.int_markers['place_pose']
        # It seems positions and orientations are randomly required to
        # be actual Point and Quaternion objects or lists/tuples. The
        # least coherent API ever seen.
        self.br.sendTransform(Point2list(place_pose.pose.position),
                              Quaternion2list(place_pose.pose.orientation),
                              rospy.Time.now(),
                              "place_pose",
                              self.robot.base)
        self.robot.place(place_pose)

        # For cubelets:
        place_pose.pose.position.z += 0.042
        # For YCB:
        # place_pose.pose.position.z += 0.026
        self.place_pose = place_pose

    def _pick(self):
        # State not modified until picking succeeds
        try:
            obj_to_get = int(self.character)
        except ValueError:
            rospy.logerr("Please provide a number in picking mode")
            return

        frame_name = "object_pose_{}".format(obj_to_get)

        rospy.loginfo("Picking object {}...".format(obj_to_get))
        if self.tl.frameExists(self.robot.base) and self.tl.frameExists(frame_name):
            t = self.tl.getLatestCommonTime(self.robot.base, frame_name)
            position, quaternion = self.tl.lookupTransform(self.robot.base,
                                                           frame_name,
                                                           t)
            print("position", position)
            print("quaternion", quaternion)

            position = list(position)
            # Vertical orientation
            self.br.sendTransform(position,
                                  [1, 0, 0, 0],
                                  rospy.Time.now(),
                                  "pick_pose",
                                  self.robot.base)
            # Ignore orientation from perception
            pose = Pose(Point(*position),
                        Quaternion(1, 0, 0, 0))
            h = Header()
            h.stamp = t
            h.frame_id = self.robot.base
            stamped_pose = PoseStamped(h, pose)
            self.robot.pick(stamped_pose)
            self.state = 'postpick'

    def _level(self):
        self.robot.level = int(self.character)
        self.state = 'level'
