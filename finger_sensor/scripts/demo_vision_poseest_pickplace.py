
'''
TYPICAL PICK AND PLACE DEMO
# Initalize with pick and place poses (options: initalize || choose previous ones)
Uses vision to identify block pose
'''

from __future__ import division, print_function
from itertools import chain, repeat
import rospy
import baxter_interface
import baxter_external_devices
from moveit_commander import conversions
from baxter_core_msgs.srv import SolvePositionIK, SolvePositionIKRequest
from baxter_core_msgs.msg import DigitalIOState
from geometry_msgs.msg import PoseArray
import tf
from baxter_pykdl import baxter_kinematics
from baxter_interface import CHECK_VERSION
import getAccEff
import time
import serial
from controller_1 import Controller_1
from getAccEff import FilterValues
import re
import matplotlib.pyplot as plt

#stopCollectingData = False
#dataCollectLock = threading.Lock()

class CuffOKButton(object):
    def __init__(self, limb_name):
        """Subscribe to the OK button on either cuff (selected using
        limb_name). action_function must be a function that accepts a data
        argument (DigitalIOState, where DigitalIOState.state is 1 if
        button is pressed, 0 otherwise).

        """
        self.limb_name = limb_name
        self.pressed = False
        node = '/robot/digital_io/%s_lower_button/state' % limb_name
        rospy.Subscriber(node, DigitalIOState, self.callback)

    def callback(self, data):
        self.pressed = data.state == 1


def limb_pose(limb_name):
    """Get limb pose at time of OK cuff button press."""
    button = CuffOKButton(limb_name)
    rate = rospy.Rate(20)  # rate at which we check whether button was
                           # pressed or not
    rospy.loginfo(
        'Waiting for %s OK cuff button press to save pose' % limb_name)
    while not button.pressed and not rospy.is_shutdown():
        rate.sleep()
    joint_pose = baxter_interface.Limb(limb_name).joint_angles()
    # Now convert joint coordinates to end effector cartesian
    # coordinates using forward kinematics.
    kinematics = baxter_kinematics(limb_name)
    endpoint_pose = kinematics.forward_position_kinematics(joint_pose)
    return endpoint_pose


class Baxter(object):
    """Add higher level operations to Baxter."""
    def __init__(self, limb_name, gripper=None):
        self.limb_name = limb_name
        self.limb = baxter_interface.Limb(limb_name)
        if gripper is None:
            self.gripper = baxter_interface.Gripper(limb_name)
        else:
            self.gripper = gripper(limb_name)

        # Enable actuators (doesn't seem to work?) I need to rosrun
        # robot_enable.py -e
        baxter_interface.RobotEnable().enable()
        # Calibrate gripper
        self.gripper.calibrate()

    def pick(self, pose, direction=(0, 0, 1), distance=0.1, controller=None):
        """Go to pose + pick_direction * pick_distance, open, go to pose,
        close, go to pose + pick_direction * pick_distance.

        """
        rospy.logdebug("pick pose: %s" % pose)
        pregrasp_pose = self.translate(pose, direction, distance)
        rospy.logdebug("pregrasp_pose: %s" % pregrasp_pose)
        rospy.sleep(1)
        self.move_ik(pregrasp_pose)
        # We want to block end effector opening so that the next
        # movement happens with the gripper fully opened.
        self.gripper.open(block=True)
        self.move_ik(pose)
        if controller is not None:
            print ('controller ON!!')
            controller.enable()
            rospy.sleep(5)
            controller.disable()
        self.gripper.close(block=True)
        self.move_ik(pregrasp_pose)

    def place(self, pose, direction=(0, 0, 1), distance=0.1):
        """Go to pose + place_direction * place_distance, go to pose,
        open, go to pose + pick_direction * pick_distance.

        """
        pregrasp_pose = self.translate(pose, direction, distance)
        self.move_ik(pregrasp_pose)
        self.move_ik(pose)
        self.gripper.open(block=True)
        self.move_ik(pregrasp_pose)
        self.gripper.close(block=True)

    def move(self, direction, distance):
        """Go to pose + place_direction * place_distance.

        """
        pose = limb_pose(self.limb_name)
        # rospy.loginfo("limb pose is %d" %pose)
        print(pose)
        pregrasp_pose = self.translate(pose, direction, distance)
        print(pregrasp_pose)
        self.move_ik(pregrasp_pose)

    def move_ik(self, pose):
        """Take a pose (either xyz rpy or xyz qxqyqzqw) and move the arm
        there.

        """
        rospy.loginfo("pose in move_ik: %s" % pose)
        stamped_pose = self.to_stamped_pose(pose)
        joint_pose = self.ik_quaternion(stamped_pose)
        return self.limb.move_to_joint_positions(joint_pose)

    @staticmethod
    def to_stamped_pose(pose):
        """Take a xyzrpy or xyz qxqyqzqw pose and convert it to a stamped
        pose (quaternion as orientation).

        """
        stamped_pose = conversions.list_to_pose_stamped(pose, "base")
        return stamped_pose

    @staticmethod
    def translate(pose, direction, distance):
        """Get an xyz rpy or xyz qxqyqzqw pose and add direction * distance to
        xyz. There's probably a better way to do this.

        """
        scaled_direction = [distance * di for di in direction]
        translated_pose = [pi + di for pi, di in
                           zip(pose,
                               chain(scaled_direction,
                                     repeat(0, len(pose) - 3)))]
        return translated_pose

    def ik_quaternion(self, quaternion_pose):
        """Take a xyz qxqyqzqw stamped pose and convert it to joint angles
        using IK. You can call self.limb.move_to_joint_positions to
        move to those angles

        """
        node = ("ExternalTools/{}/PositionKinematicsNode/"
                "IKService".format(self.limb_name))
        ik_service = rospy.ServiceProxy(node, SolvePositionIK)
        ik_request = SolvePositionIKRequest()
        ik_request.pose_stamp.append(quaternion_pose)
        try:
            rospy.loginfo('ik: waiting for IK service...')
            rospy.wait_for_service(node, 5.0)
            ik_response = ik_service(ik_request)
        except (rospy.ServiceException, rospy.ROSException) as err:
            rospy.logerr("ik_move: service request failed: %r" % err)
        else:
            if ik_response.isValid[0]:
                rospy.loginfo("ik_move: valid joint configuration found")
                # convert response to joint position control dictionary
                limb_joints = dict(zip(ik_response.joints[0].name,
                                       ik_response.joints[0].position))
                return limb_joints
            else:
                rospy.logerr('ik_move: no valid configuration found')
                return None


class Blocks(object):
    def __init__(self):
        self.sub = rospy.Subscriber('/finger_sensor_test/blockpose', PoseArray, self.update)
        self.pose = None

    def update(self, msg):
        pose = msg.poses[0]
        self.pose = [pose.position.x,
                     pose.position.y,
                     pose.position.z,
                     -0.99,0,0,0]

def main(limb_name, reset):
    """
    Parameters
    ----------
    limb : str
        Which limb to use. Choices are {'left', 'right'}
    reset : bool
        Whether to use previously saved picking and placing poses or
        to save new ones by using 0g mode and the OK cuff buttons.
    """
    # Initialise ros node
    rospy.init_node("pick_and_place", anonymous=False)


    b = Baxter(limb_name)
    place_pose = limb_pose(limb_name).tolist()
    print (place_pose)
    block = Blocks()
    rospy.sleep(4)
    pick_pose = block.pose
    rospy.loginfo('Block pose: %s' % pick_pose)
    #import ipdb; ipdb.set_trace()
    b.pick(pick_pose, controller=None)
    b.place(place_pose)



if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(
        description='Use baxter to pick and place objects')
    parser.add_argument(
        '-l', '--limb', choices=('left', 'right'), default='left',
        help='Choose which limb to use for picking and placing')
    parser.add_argument(
        '-r', '--reset', action='store_true', default=False)

    args = parser.parse_args(rospy.myargv()[1:])

    main(args.limb, args.reset)
