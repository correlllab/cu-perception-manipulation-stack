#! /usr/bin/env python
from __future__ import division, print_function, absolute_import

import numpy as np

import rospy
import tf
from std_msgs.msg import Header, Int32MultiArray
from geometry_msgs.msg import (Point, Quaternion, Pose, PoseStamped,
                               Vector3, Vector3Stamped)

from pap.robot import Baxter


class SmartBaxter(Baxter):
    def __init__(self, limb_name, topic='/sensor_values'):
        super(SmartBaxter, self).__init__(limb_name)
        self.inside = np.zeros(14)
        self.tip = np.zeros(2)
        self.inside_offset = np.zeros_like(self.inside)
        self.tip_offset = np.zeros_like(self.tip)

        self.sensor_sub = rospy.Subscriber(topic,
                                           Int32MultiArray,
                                           self.update_sensor_values,
                                           queue_size=1)
        self.zero_sensor()

    def update_sensor_values(self, msg):
        values = np.array(msg.data)
        self.inside = np.concatenate((values[:7],
                                      values[8:15])) - self.inside_offset
        self.tip = values[[7, 15]] - self.tip_offset
        error = np.mean(self.inside[7:] - self.inside[:7])
        # Experimentally, we need a deadband of about
        # 314-315. Otherwise it moves left or right with nothing in
        # between fingers.
        if abs(error) < 350:
            self.error = 0
        else:
            self.error = error

    def zero_sensor(self):
        rospy.loginfo("Zeroing sensor...")
        # Wait a little bit until we get a message from the sensor
        rospy.sleep(1)
        self.tip_offset, self.inside_offset = (np.zeros_like(self.tip),
                                               np.zeros_like(self.inside))
        inside_vals, tip_vals = [], []
        r = rospy.Rate(10)
        while not rospy.is_shutdown() and len(inside_vals) < 10:
            inside, tip = self.inside, self.tip
            # If there are zero values (most likely becase a message
            # has not yet been received), skip that. We could also
            # initialize them with nans to find out if there's a
            # problem
            if all(inside) and all(tip):
                inside_vals.append(inside)
                tip_vals.append(tip)
            r.sleep()
        # Center around 5000, so ranges are similar to when not centering
        self.inside_offset = np.min(inside_vals, axis=0) - 5000
        self.tip_offset = np.min(tip_vals, axis=0) - 5000
        rospy.loginfo("Zeroing finished")


class SafetyStop(object):
    """In theory, if this node is running, no matter what happens, if
    something gets too close to the sensor tip, the robot will stop
    moving until the obstacle is cleared.

    """
    def __init__(self, topic='/sensor_values'):
        self.bx = Baxter('left')
        self.joint_v = {name: 0.0 for name in self.bx.limb.joint_names()}
        self.stop = False
        self.nh = rospy.init_node('SafetyStop')
        self.sensor_sub = rospy.Subscriber(topic,
                                           Int32MultiArray,
                                           self.udpate_joint_v,
                                           queue_size=1)

    def _cartesian_v_from_sensor_values(self, values):
        tip = values[[7, 15]]
        if np.any(tip < 10**4):
            self.stop = True
        else:
            self.stop = False
        return [0] * 6

    def update_joint_v(self, msg):
        arr = np.array(msg.data)
        cartesian_v = self._cartesian_v_from_sensor_values(arr)
        self.joint_v = self.bx.compute_joint_velocities(cartesian_v)


class ControlArmThroughHand(object):
    def __init__(self, topic='/sensor_values'):
        self.bx = SmartBaxter('left')

        self.br = tf.TransformBroadcaster()
        self.tl = tf.TransformListener()

    def control_from_sensor_values(self):
        log_values = np.log(self.bx.inside)
        # Match which side is which. Ideally, if the sign of the diff
        # matches whether the gripper needs to move towards the
        # positive or negative part of the y axis in left_gripper.
        # That is, we need left side - right side (using left/right
        # like in l_gripper_{l, r}_finger_tip tfs)
        # We want to take log(values) for a better behaved controller
        inside_diff = log_values[7:] - log_values[:7]
        scalar_diff = sum(inside_diff) / len(inside_diff)

        # Take negative for one of the sides, so that angles should
        # match for a parallel object in the gripper
        l_angle, _ = np.polyfit(np.arange(7), log_values[:7], 1)
        r_angle, _ = np.polyfit(np.arange(7), -log_values[7:], 1)
        rospy.loginfo('Angle computed from l: {}'.format(np.rad2deg(l_angle)))
        rospy.loginfo('Angle computed from r: {}'.format(np.rad2deg(r_angle)))
        avg_angle = np.arctan((l_angle + r_angle) / 2.0)

        # Let's get a frame by the middle of the end effector
        # p = Point(0, 0, -0.05)
        p = (0, 0, -0.05)
        # Of course, tf uses (w, x, y, z), Quaternion uses x, y, z,
        # w. However, tf.TransformBroadcaster().sendTransform uses the
        # tf order.
        # q = Quaternion(q[1], q[2], q[3], q[0])
        q = tf.transformations.quaternion_about_axis(avg_angle, (1, 0, 0))
        # We had a lot of trouble sending a transform (p, q) from
        # left_gripper, and then asking for a point in the last frame
        # in the tf coordinate. Timing issues that I couldn't
        # solve. Instead, do it manually here:
        m1 = tf.transformations.quaternion_matrix(q)
        m1[:3, 3] = p
        p = (0, scalar_diff / 100, 0.05)
        m2 = np.eye(4)
        m2[:3, 3] = p
        m = m2.dot(m1)
        # Extract pose now
        p = Point(*m[:3, 3])
        q = Quaternion(*tf.transformations.quaternion_from_matrix(m))
        time = rospy.Time(0)
        h = Header()
        h.frame_id = 'left_gripper'
        h.stamp = time
        pose = Pose(p, q)
        new_endpose = self.tl.transformPose('base', PoseStamped(h, pose))
        self.bx.move_ik(new_endpose)


class VelocityControlArmThroughHand(object):
    def __init__(self, topic='/sensor_values'):
        self.bx = SmartBaxter('left')

        self.br = tf.TransformBroadcaster()
        self.tl = tf.TransformListener()

    def _transform_vector(self, v, mode='speed', to='base'):
        h = Header()
        h.stamp = rospy.Time(0)
        h.frame_id = '{}_gripper'.format(self.bx.limb_name)
        v = Vector3(*v)
        v_base = self.tl.transformVector3(to,
                                          Vector3Stamped(h, v)).vector
        if mode == 'speed':
            v_cartesian = np.array([0, 0, 0, v_base.x, v_base.y, v_base.z])
        elif mode == 'direction':
            v_cartesian = np.array([v_base.x, v_base.y, v_base.z, 0, 0, 0])
        return v_cartesian

    def control_from_sensor_values(self):
        log_values = np.log(self.bx.inside)
        # Match which side is which. Ideally, if the sign of the diff
        # matches whether the gripper needs to move towards the
        # positive or negative part of the y axis in left_gripper.
        # That is, we need left side - right side (using left/right
        # like in l_gripper_{l, r}_finger_tip tfs)
        inside_diff = log_values[7:] - log_values[:7]
        scalar_diff = sum(inside_diff) / len(inside_diff)

        rospy.loginfo('scalar diff is {}'.format(scalar_diff))
        scalar_diff = deadband(scalar_diff, 0.05)

        # Take negative for one of the sides, so that angles should
        # match for a parallel object in the gripper
        l_angle, _ = np.polyfit(np.arange(7), log_values[:7], 1)
        r_angle, _ = np.polyfit(np.arange(7), -log_values[7:], 1)
        rospy.loginfo('Angle computed from l: {}'.format(np.rad2deg(l_angle)))
        rospy.loginfo('Angle computed from r: {}'.format(np.rad2deg(r_angle)))
        avg_angle = np.arctan((l_angle + r_angle) / 2.0)
        avg_angle = deadband(avg_angle, 0.1)

        # Essentially, first align, then center
        if avg_angle == 0.0:
            # Unstable: 1. Better, but too brusque: 0.1. Not good either: 0.01.
            P = 0.1
        else:
            P = 0.0
        y_v = np.clip(P * scalar_diff, -0.08, 0.08)
        rospy.loginfo("y_v = {}".format(y_v))
        v_cartesian_1 = self._transform_vector((0, y_v, 0), mode='direction')
        P = 1
        rotation_speed = (P * avg_angle, 0, 0)
        v_cartesian_2 = self._transform_vector(rotation_speed, mode='speed')
        v_cartesian = v_cartesian_1 + v_cartesian_2
        v_joint = self.bx.compute_joint_velocities(v_cartesian)
        self.bx.limb.set_joint_velocities(v_joint)


def deadband(value, a, b=None):
    if abs(value) < a:
        value = 0.0
    if b is not None:
        value = np.clip(value, -b, b)
    return value

if __name__ == '__main__':
    mode = 2
    if mode == 0:
        rospy.init_node('SafetyStop')
        s = SafetyStop()
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            if s.stop:
                s.bx.limb.set_joint_velocities(s.joint_v)
            r.sleep()
    else:
        nh = rospy.init_node('ArmController')
        if mode == 1:
            s = ControlArmThroughHand()
        elif mode == 2:
            s = VelocityControlArmThroughHand()
        rospy.sleep(1)
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            s.control_from_sensor_values()
            r.sleep()
