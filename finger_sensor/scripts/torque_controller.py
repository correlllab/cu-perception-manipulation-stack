#!/usr/bin/env python

"""
Baxter joint torque controller. Uses recoreded timestamped joint positions.
"""

import argparse
import operator
import sys
import rospy

from bisect import bisect
from copy import copy
from os import path
import actionlib
from IPython.core.debugger import Pdb
from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
)
from trajectory_msgs.msg import (JointTrajectoryPoint,)

from dynamic_reconfigure.server import (Server,)
from std_msgs.msg import (Empty,)
import baxter_interface

from baxter_examples.cfg import (JointSpringsExampleConfig,)
from baxter_interface import CHECK_VERSION


class JointSprings(object):
    """
    Virtual Joint Springs class for torque example.

    @param limb: limb on which to run joint springs example
    @param reconfig_server: dynamic reconfigure server

    JointSprings class contains methods for the joint torque example allowing
    moving the limb to a neutral location, entering torque mode, and attaching
    virtual springs.
    """
    def __init__(self, limb, reconfig_server):
        self._dyn = reconfig_server
        self.movearm = []
        # control parameters
        self._rate = 1000.0  # Hz
        self._missed_cmds = 20.0  # Missed cycles before triggering timeout

        # create our limb instance
        self._limb = baxter_interface.Limb(limb)

        # initialize parameters
        self._springs = dict()
        self._damping = dict()
        self._start_angles = dict()
        self.left_joint_names = []
        self.i = 1
        self.joint_names = []
        self.sum_sqr_error = 0

        # create cuff disable publisher
        cuff_ns = 'robot/limb/' + limb + '/suppress_cuff_interaction'
        self._pub_cuff_disable = rospy.Publisher(cuff_ns, Empty, queue_size=1)

        # verify robot is enabled
        print("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable(CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()
        print("Running. Ctrl-c to quit")

    def _update_parameters(self):
        for joint in self._limb.joint_names():
            self._springs[joint] = self._dyn.config[joint[-2:] +
                                                    '_spring_stiffness']
            self._damping[joint] = self._dyn.config[joint[-2:] +
                                                    '_damping_coefficient']

    def move_to_neutral(self):
        """
        Moves the limb to neutral location.
        """
        self._limb.move_to_neutral()

    def try_float(self,x):
        try:
            return float(x)
        except ValueError:
            return None

    def _get_data(self):
        with open('jointrecorderBax.txt', 'r') as f:
            self.data = f.readlines()

        self.joint_names = self.data[0].rstrip().split(',')
        #parse joint names for the left and right limbs
        for name in self.joint_names:
            if 'left' == name[:-3]:
                self.left_joint_names.append(name)

    def _get_cmd_positions(self):
        #convt str to float
        self.i += 1
        self._start_angles_1 = [self.try_float(x) for x in self.data[self.i].rstrip().split(',')] #str to float
        cmd = dict(zip(self.joint_names, self._start_angles_1)) #formatted dict({str:float})
        #selecting left arm values
        left_arm_cmd = [cmd[jnts] for jnts in self.left_joint_names]
        self._start_angles_1 = dict(zip(self.left_joint_names, left_arm_cmd)) #formatted dict({str:float})
        return self._start_angles_1

    def attach_springs(self):
        self._start_angles = self._get_cmd_positions()
        control_rate = rospy.Rate(self._rate)

        # for safety purposes, set the control rate command timeout.
        # if the specified number of command cycles are missed, the robot
        # will timeout and disable
        self._limb.set_command_timeout((1.0 / self._rate) * self._missed_cmds)

        error = [self._limb.joint_angles()[key] - self._start_angles[key] for key in self._limb.joint_angles().keys()]
        self.sum_sqr_error = sum([error[i]**2 for i in range(len(error))])
        print ('-------new set of joint position---------')
        print (self.sum_sqr_error)
        tolerance = 0.020
        # loop at specified rate commanding new joint torques
        while self.sum_sqr_error>tolerance and not rospy.is_shutdown():

            if not self._rs.state().enabled:
                    rospy.logerr("Joint torque example failed to meet "
                            "specified control rate timeout.")
                    break
            self._update_forces()
            control_rate.sleep()


    def _update_forces(self):
        """
        Calculates the current angular difference between the start position
        and the current joint positions applying the joint torque spring forces
        as defined on the dynamic reconfigure server.
        """
        # get latest spring constants
        self._update_parameters()
        # disable cuff interaction
        self._pub_cuff_disable.publish()
        # create our command dict
        cmd = dict()
        # record current angles/velocities
        cur_pos = self._limb.joint_angles()
        cur_vel = self._limb.joint_velocities()


        error = [self._limb.joint_angles()[key] - self._start_angles[key] for key in self._limb.joint_angles().keys()]
        self.sum_sqr_error = sum([error[i]**2 for i in range(len(error))])
        #print (self.sum_sqr_error)

        # calculate current forces
        for joint in self._start_angles.keys():
            # spring portion
            cmd[joint] = self._springs[joint] * (self._start_angles[joint] -
                                                   cur_pos[joint])
            # damping portion
            cmd[joint] -= self._damping[joint] * cur_vel[joint]
        # command new joint torques
        self._limb.set_joint_torques(cmd)
        #print ('These are the torque values')
        #print (cmd)

    def clean_shutdown(self):
        """
        Switches out of joint torque mode to exit cleanly
        """
        print("\nExiting example...")
        self._limb.exit_control_mode()
        if not self._init_state and self._rs.state().enabled:
            print("Disabling robot...")
            self._rs.disable()


def main():
    """Joint Torque Example: Joint Springs

    Moves the specified limb in torque control mode,
    attaching virtual springs (Hooke's Law)
    to each joint maintaining the start position.
    """
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    parser.add_argument(
        '-l', '--limb', dest='limb', required=True, choices=['left', 'right'],
        help='limb on which to attach joint springs'
    )
    args = parser.parse_args(rospy.myargv()[1:])

    print("Initializing node... ")
    rospy.init_node("rsdk_joint_torque_springs_%s" % (args.limb,))
    dynamic_cfg_srv = Server(JointSpringsExampleConfig,
                             lambda config, level: config)
    js = JointSprings(args.limb, dynamic_cfg_srv)
    # register shutdown callback
    rospy.on_shutdown(js.clean_shutdown)
    js._get_data() # get data from file
    js.move_to_neutral()
    #while not rospy.is_shutdown() and js.i<len(js.data):
    while len(js.data) and not rospy.is_shutdown():
        js.attach_springs()



if __name__ == "__main__":
    main()
