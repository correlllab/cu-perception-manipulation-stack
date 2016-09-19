#! /usr/bin/env python
from __future__ import division, print_function, absolute_import

from math import copysign, pi, cos

import numpy as np

import rospy
from std_msgs.msg import Int32MultiArray, Header
from geometry_msgs.msg import Vector3, Vector3Stamped
from keyboard.msg import Key
from teleop_interface.msg import object as ob

from pap.robot import Baxter, Jaco
from pap.manager import PickAndPlaceNode


class SmartBaxter(Baxter):
    def __init__(self, limb_name, topic='/sensor_values'):
        super(SmartBaxter, self).__init__(limb_name)
        self.inside = np.zeros(14)
        self.tip = np.zeros(2)
        self.inside_offset = np.zeros_like(self.inside)
        self.tip_offset = np.zeros_like(self.tip)
        # Picking level
        self.level = None

        self.sensor_sub = rospy.Subscriber(topic,
                                           Int32MultiArray,
                                           self.update_sensor_values,
                                           queue_size=1)
        #self.zero_sensor()
        #import ipdb;ipdb.set_trace()

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

    def _vector_to(self, vector, to='base'):
        h = Header()
        h.stamp = rospy.Time(0)
        h.frame_id = '{}_gripper'.format(self.limb_name)
        v = Vector3(*vector)
        v_base = self.tl.transformVector3(to,
                                          Vector3Stamped(h, v)).vector
        v_cartesian = [v_base.x, v_base.y, v_base.z, 0, 0, 0]
        return v_cartesian

    def pick(self, pose, direction=(0, 0, 1), distance=0.1):
        pregrasp_pose = self.translate(pose, direction, distance)

        self.move_ik(pregrasp_pose)
        rospy.sleep(0.5)
        # We want to block end effector opening so that the next
        # movement happens with the gripper fully opened.
        self.gripper.open()
        while True:
            rospy.loginfo("Going down to pick (at {})".format(self.tip.max()))
            if self.tip.max() > 10000:
                break
            else:
                scaled_direction = (di / 100 for di in direction)
                v_cartesian = self._vector_to(scaled_direction)
                v_joint = self.compute_joint_velocities(v_cartesian)
                self.limb.set_joint_velocities(v_joint)
        rospy.loginfo('Went down!')
        rospy.sleep(0.5)
        rospy.loginfo('Centering')
        # Let's stop centering when we've seen enough zero errors so
        # far (so that a single spike doesn't stop the process)
        so_far = 0
        while True:
            err = self.error
            rospy.loginfo("err is {}".format(err))
            # This needs to be 0, given the deadband
            if abs(err) == 0:
                so_far += 1
                if so_far == 4:
                    # Not sure if necessary, but command 0 velocity seems
                    # a good idea
                    self.limb.set_joint_velocities(
                        self.compute_joint_velocities([0, 0, 0, 0, 0, 0])
                    )
                    break
            # Controlling the arm: At 1.0 / 30000.0 * err, it's unstable
            # 40000^{-1} -> unstable
            # 50000^{-1} -> unstable
            # 80000^{-1} -> almost stable
            # 10^{-5} -> decent given the system, etc
            y_v = np.clip(1.0 / 100000.0 * err, -0.08, 0.08)
            rospy.loginfo("y_v = {}".format(y_v))
            v_cartesian = self._vector_to((0, y_v, 0))
            v_joint = self.compute_joint_velocities(v_cartesian)
            self.limb.set_joint_velocities(v_joint)
        rospy.loginfo('Centered')
        # self.move_ik(pose)

        rospy.sleep(0.5)
        self.gripper.close()
        rospy.sleep(0.5)
        self.move_ik(pregrasp_pose)

    def place(self, pose, direction=(0, 0, 1), distance=0.1):
        preplace_pose = self.translate(pose, direction, distance)
        self.move_ik(preplace_pose)
        rospy.sleep(0.5)
        # Edit this. Idea:
        # 1) Move down (direction) until maybe 1cm above place pose.
        # 2) Figure out whether one tip is closer or the other
        # 3) Move left-right to fix that, until values are similar from both tip sensors
        # 4) When kind of close, stop. Move down the missing cm.
        # 5) Open gripper
        # 6) Go to pregrasp
        while True:
            rospy.loginfo("Going down to pick (at {})".format(self.tip.max()))
            # Comment refers to cubelets:
            # 5900 as max tip value works well to stack two levels
            # high. Value experimentally found. Each block is 42 mm
            # high. Believe 5700 is the right value for two blocks
            # high.
            # Ideally, we'd have a fully calibrated sensor and these
            # would become distances
            # Cubelets limits:
            limit = {1: 5700, 2: 5500}
            # YCB limits:
            # limit = {1: 7450, 2: 7100, 9: 5520 + 50 - 20}  # 9: side stacking (11000 for bottom)
            if self.tip.max() > limit[self.level]:
                break
            else:
                scaled_direction = (di / 100 for di in direction)
                v_cartesian = self._vector_to(scaled_direction)
                v_joint = self.compute_joint_velocities(v_cartesian)
                self.limb.set_joint_velocities(v_joint)
        self.limb.set_joint_velocities(self.compute_joint_velocities([0] * 6))
        rospy.loginfo('Went down!')
        rospy.sleep(0.5)
        # If side stacking, do that
        if self.level == 9:
            done = False
            while not done:
                try:
                    cond = False
                    if cond:
                        done = True
                        break
                    # Hard code going backwards
                    scaled_direction = (di / 100 for di in (-1, 0, 0))
                    v_cartesian = self._vector_to(scaled_direction)
                    v_joint = self.compute_joint_velocities(v_cartesian)
                    self.limb.set_joint_velocities(v_joint)
                except KeyboardInterrupt:
                    break
        else:
            A = 0.0  # Centering amplitude (SI)
            theta = pi  # rad/s, so the centering wave takes 2s
            t_0 = rospy.Time.now().to_sec()
            r = rospy.Rate(50)
            tip = []
            while not rospy.is_shutdown():
                # We could do something more clever and command the
                # midpoint velocity from now until the next command, but
                # not really that important
                t = rospy.Time.now().to_sec()
                if theta * (t - t_0) > (2 * pi):
                    break
                y_v = A * theta * cos(theta * (t - t_0))
                v_cartesian = self._vector_to((0, y_v, 0))
                v_joint = self.compute_joint_velocities(v_cartesian)
                self.limb.set_joint_velocities(v_joint)
                tip.append(sum(self.tip))
            r = rospy.Rate(20)
            done = False
            #import pdb; pdb.set_trace()
            while not done:
                rospy.loginfo("centering")
                delta = - (self.tip[0] - self.tip[1])
                rospy.loginfo("Delta is {} ({})".format(delta, 'positive' if delta >= 0 else 'negative'))
                if abs(delta) < 400:
                    done = True
                else:
                    v_cartesian = self._vector_to((0, copysign(0.005, delta), 0))
                    v_joint = self.compute_joint_velocities(v_cartesian)
                    self.limb.set_joint_velocities(v_joint)
                    r.sleep()
            # We're usually over by now, so let's go back a little bit
            r = rospy.Rate(20)
            for i in range(2):
                v_cartesian = self._vector_to((0, -copysign(0.005, delta), 0))
                v_joint = self.compute_joint_velocities(v_cartesian)
                self.limb.set_joint_velocities(v_joint)
                r.sleep()
            self.limb.set_joint_velocities(self.compute_joint_velocities([0] * 6))
            rospy.loginfo("centered")
        rospy.sleep(0.5)
        #self.move_ik(pose)
        #rospy.sleep(0.5)
        self.gripper.open()
        rospy.sleep(0.5)
        self.move_ik(preplace_pose)


class Placement(object):
    def __init__(self):
        self.nh = rospy.init_node('placing')
        self.character = 'k'
        self.bx = SmartBaxter('left')
        self.kb_sub = rospy.Subscriber('/keyboard/keyup',
                                       Key,
                                       self.keyboard_cb, queue_size=1)

    def keyboard_cb(self, msg):
        character = chr(msg.code)
        if character in {'j', 'k', 'l'}:
            self.character = character

    def run_manual(self):
        r = rospy.Rate(20)
        while not rospy.is_shutdown():
            v = {'k': (0, 0, 0),
                 'j': (0, 0, 0.02),
                 'l': (0, 0, -0.02)}[self.character]
            v_cartesian = self.bx._vector_to(v)
            v_joint = self.bx.compute_joint_velocities(v_cartesian)
            self.bx.limb.set_joint_velocities(v_joint)
            r.sleep()

    def run_auto(self):
        r = rospy.Rate(20)
        while not rospy.is_shutdown():
            for i in range(20):
                v = (0, 0, 0.02)
                v_cartesian = self.bx._vector_to(v)
                v_joint = self.bx.compute_joint_velocities(v_cartesian)
                self.bx.limb.set_joint_velocities(v_joint)
                r.sleep()
            for i in range(20):
                v = (0, 0, -0.02)
                v_cartesian = self.bx._vector_to(v)
                v_joint = self.bx.compute_joint_velocities(v_cartesian)
                self.bx.limb.set_joint_velocities(v_joint)
                r.sleep()


if __name__ == '__main__':
    if False:
        p = Placement()
        # p.run_manual()
        p.run_auto()
    smart = True
    #1/0
    if smart:
        n = PickAndPlaceNode(SmartBaxter, 'left')
    else:
        n = PickAndPlaceNode(Jaco)
        # n = PickAndPlaceNode(Baxter('left'))
    rospy.spin()
