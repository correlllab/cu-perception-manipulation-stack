#!/usr/bin/env python
from __future__ import division, print_function

import numpy as np

import rospy

import actionlib

from std_msgs.msg import Header
from geometry_msgs.msg import Point, Quaternion, Pose, PoseStamped

import baxter_interface
# Not used currently, but interesting module!
# from moveit_commander import conversions
from baxter_core_msgs.srv import SolvePositionIK, SolvePositionIKRequest
from baxter_core_msgs.msg import DigitalIOState

import kinova_msgs.msg
from kinova_msgs.srv import HomeArm

# Apparently tf was deprecated a long time ago? And we should use tf2?
import tf
# import tf2_ros

from baxter_pykdl import baxter_kinematics

from .utils import Point2list


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
    """Get limb pose at time of OK cuff button press.

    The returned pose matches """
    button = CuffOKButton(limb_name)
    rate = rospy.Rate(20)  # rate at which we check whether button was
                           # pressed or not
    rospy.loginfo(
        'Waiting for %s OK cuff button press to save pose' % limb_name)
    while not button.pressed and not rospy.is_shutdown():
        rate.sleep()
    endpoint_pose = baxter_interface.Limb(limb_name).endpoint_pose()
    pose_list = ([getattr(endpoint_pose['position'], i) for i in
                  ('x', 'y', 'z')] +
                 [getattr(endpoint_pose['orientation'], i) for i in
                  ('x', 'y', 'z', 'w')])
    return pose_list
    # How is
    # baxter_kinematics(limb_name).forward_position_kinematics(
    #     baxter_interface.Limb(limb_name).joint_angles())
    # different from
    # baxter_interface.Limb(limb_name).endpoint_pose()
    # After some testing, we find that the differences are
    # negligible. In rviz, pretty much the same. In numerical terms,
    # they agree to at least two significant figures.
    # Eg:
    """
    baxter_interface endpoint pose:
    {'position': Point(x=0.8175678653720638,
                       y=0.11419185934519176,
                       z=-0.18813767395654984),
     'orientation': Quaternion(x=0.9983899777382322,
                               y=-0.008582355126430147,
                               z=0.051452248110337225,
                               w=-0.022281420437849815)}
    pykdl forward kinematics endpoint pose:
    [ 0.81954073  0.11447536 -0.18811824
      0.9983032  -0.00843487  0.0532936  -0.02189443]"""


class Robot(object):
    def __init__(self, base):
        """base : str
            Robot base/root tf"""
        self.base = base
        self.tl = tf.TransformListener()

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


class BaxterGripper(baxter_interface.Gripper):
    """Make open/close not take any args, to play nicely with our
    superclass Robot"""
    def open(self):
        super(BaxterGripper, self).open(block=True)

    def close(self):
        super(BaxterGripper, self).close(block=True)


class Baxter(Robot):
    """Add higher level operations to Baxter."""
    def __init__(self, limb_name, gripper=None):
        super(Baxter, self).__init__(base='base')
        self.limb_name = limb_name
        self.other_limb_name = {'right': 'left',
                                'left': 'right'}[limb_name]

        self.limb = baxter_interface.Limb(limb_name)
        self.limb.set_joint_position_speed(0.1)

        if gripper is None:
            self.gripper = BaxterGripper(limb_name)
        else:
            self.gripper = gripper(limb_name)

        # Enable actuators (doesn't seem to work?) I need to rosrun
        # robot_enable.py -e
        baxter_interface.RobotEnable().enable()
        # Calibrate gripper
        self.gripper.calibrate()

        self.kinematics = baxter_kinematics(self.limb_name)

    def compute_joint_velocities(self, cartesian_velocities, jinv=None):
        """Given cartesian end effector velocities (in the base coordinate
        system), compute the necessary joint velocities to move it in
        that way.

        It can reuse a jacobian pseudo inverse, which is particularly
        useful to avoid excessive computations for small
        displacements.

        """
        if jinv is None:
            self.jinv = self.kinematics.jacobian_pseudo_inverse()
        else:
            self.jinv = jinv
        joint_v = np.squeeze(np.asarray(self.jinv.dot(cartesian_velocities)))
        joint_v_dict = {'{}_{}'.format(self.limb_name, joint_name): val for
                        joint_name, val in zip(['s0', 's1', 'e0', 'e1',
                                                'w0', 'w1', 'w2'],
                                               joint_v)}
        joint_v_dict.update(
            {'{}_{}'.format(self.other_limb_name, joint_name): 0.0 for
             joint_name in ['s0', 's1', 'e0', 'e1', 'w0', 'w1', 'w2']})
        joint_v_dict.update(
            {'head_nod': 0.0, 'head_pan': 0.0, 'torso_t0': 0.0})
        return joint_v_dict

    def ik_quaternion(self, stamped_pose):
        """Take a PoseStamped and convert it to joint angles using IK. You can
        call self.limb.move_to_joint_positions to move to those angles

        """
        node = ("ExternalTools/{}/PositionKinematicsNode/"
                "IKService".format(self.limb_name))
        ik_service = rospy.ServiceProxy(node, SolvePositionIK)
        ik_request = SolvePositionIKRequest()
        ik_request.pose_stamp.append(stamped_pose)
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

    def move_ik(self, stamped_pose):
        """Take a PoseStamped and move the arm there.

        """
        if not isinstance(stamped_pose, PoseStamped):
            raise TypeError("No duck typing here? :(")
        joint_pose = self.ik_quaternion(stamped_pose)
        return self.limb.move_to_joint_positions(joint_pose)

    def _compare_ik_fk(self):
        """All the logged values should match in theory. In practice they're
        fairly close.

        """
        kinematics = baxter_kinematics(self.limb_name)
        rospy.logerr(kinematics.forward_position_kinematics())
        rospy.logerr(kinematics.forward_position_kinematics(
            self.limb.joint_angles()))
        rospy.logerr(self.limb.endpoint_pose())

    def fk(self, joint_angles=None):
        """Compute end point pose (xyz qxqyqzqw) based on joint angles."""
        kinematics = baxter_kinematics(self.limb_name)
        return kinematics.forward_position_kinematics(joint_angles)

    def move_relative(self, pose):
        """Move endpoint according to the relative pose given.

        Examples
        --------
        # No movement
        baxter.move_relative(Pose(Point(0, 0, 0), Quaternion(0, 0, 0, 1)))
        # End effector translation
        baxter.move_relative(Pose(Point(x, y, z), Quaternion(0, 0, 0, 1)))
        """
        # Why use a geometry_msgs.msg.Pose for a pose when rethink can
        # invent its own?
        current_pose = self.limb.endpoint_pose()
        # Convert to actual Pose type
        current_pose = Pose(Point(*current_pose['position']),
                            Quaternion(*current_pose['orientation']))
        h = Header()
        h.stamp = rospy.Time.now()
        h.frame_id = '{}_gripper'.format(self.limb_name)
        stamped_rel_pose = PoseStamped(h,
                                       pose)
        final_pose = self.tl.transformPose("base", stamped_rel_pose)
        self.move_ik(final_pose)

    @staticmethod
    def _to_quaternion_pose(pose):
        if len(pose) == 6:
            q = tf.transformations.quaternion_from_euler(*pose[3:]).tolist()
            return list(pose[:3]) + q
        elif len(pose) == 7:
            return pose
        else:
            raise TypeError('Pose needs to be xyzrpy or xyzqxqyqzq')


class JacoGripper(object):
    def __init__(self, robot_type='j2n6a300'):
        self.robot_type = robot_type
        action_address = ('/{}_driver/fingers_action/'
                          'finger_positions'.format(robot_type))
        self.client = actionlib.SimpleActionClient(
            action_address,
            kinova_msgs.msg.SetFingersPositionAction)
        self.client.wait_for_server()

        self.finger_maxDist = 18.9/2/1000  # max distance for one finger
        self.finger_maxTurn = 6800  # max thread rotation for one finger

    def set_position(self, position):
        """Accept position from 0 (open) to 100 (closed)."""
        position_turns = np.clip(position, 0, 100) / 100 * self.finger_maxTurn
        goal = kinova_msgs.msg.SetFingersPositionGoal()
        goal.fingers.finger1 = float(position_turns[0])
        goal.fingers.finger2 = float(position_turns[1])
        # The MICO arm has only two fingers, but the same action
        # definition is used
        if len(position) < 3:
            goal.fingers.finger3 = 0.0
        else:
            goal.fingers.finger3 = float(position_turns[2])
        self.client.send_goal(goal)
        if self.client.wait_for_result(rospy.Duration(5.0)):
            return self.client.get_result()
        else:
            self.client.cancel_all_goals()
            rospy.logwarn('        the gripper action timed-out')

    def open(self):
        self.set_position([0, 0, 0])

    def close(self):
        self.set_position([100, 100, 100])


class Jaco(Robot):
    def __init__(self, robot_type='j2n6a300', *args, **kwargs):
        super(Jaco, self).__init__(base='root')
        self.robot_type = robot_type
        self.home()
        self.gripper = JacoGripper()
        self.velocity_pub = rospy.Publisher(
            '/{}_driver/in/cartesian_velocity'.format(self.robot_type),
            kinova_msgs.msg.PoseVelocity,
            queue_size=1)
        action_address = ('/' + self.robot_type +
                          '_driver/pose_action/tool_pose')
        self.client = actionlib.SimpleActionClient(
            action_address,
            kinova_msgs.msg.ArmPoseAction)

    def home(self):
        addr = '/{}_driver/in/home_arm'.format(self.robot_type)
        rospy.wait_for_service(addr)
        try:
            serv = rospy.ServiceProxy(addr, HomeArm)
            rep = serv()
            rospy.loginfo(rep)
        except rospy.ServiceException as e:
            rospy.loginfo("Service error {}".format(e))

    def move_ik(self, stamped_pose):
        """Take a PoseStamped and move the arm there.

        """
        if not isinstance(stamped_pose, PoseStamped):
            raise TypeError("No duck typing here? :(")
        pose = stamped_pose.pose
        position, orientation = pose.position, pose.orientation
        # Send a cartesian goal to the action server. Adapted from kinova_demo
        rospy.loginfo("Waiting for SimpleAction server")
        self.client.wait_for_server()

        goal = kinova_msgs.msg.ArmPoseGoal()
        goal.pose.header = Header(frame_id=(self.robot_type + '_link_base'))
        goal.pose.pose.position = position
        goal.pose.pose.orientation = orientation

        rospy.loginfo("Sending goal")
        self.client.send_goal(goal)

        t = 3.0
        rospy.loginfo("Waiting for up to {} s for result".format(t))
        if self.client.wait_for_result(rospy.Duration(t)):
            rospy.loginfo("Action succeeded")
            return self.client.get_result()
        else:
            self.client.cancel_all_goals()
            rospy.loginfo('        the cartesian action timed-out')

    def kinematic_control(self):
        r = rospy.Rate(100)
        return
        while not self.done:
            msg = kinova_msgs.msg.PoseVelocity(
                twist_linear_x=0.0,
                twist_linear_y=0.0,
                twist_linear_z=0.0,
                twist_angular_x=0.0,
                twist_angular_y=0.0,
                twist_angular_z=10.0)
            self.velocity_pub.publish(msg)
            r.sleep()


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
    rospy.init_node("pick_and_place", anonymous=True)

    # Either load picking and placing poses from the parameter server
    # or save them using the 0g mode and the circular buttons on
    # baxter's cuffs
    if reset or not rospy.has_param('~pick_and_place_poses'):
        rospy.loginfo(
            'Saving picking pose for %s limb' % limb_name)
        pick_pose = limb_pose(limb_name)
        rospy.sleep(1)
        place_pose = limb_pose(limb_name)
        # Parameter server can't store numpy arrays, so make sure
        # they're lists of Python floats (specifically not
        # numpy.float64's). I feel that's a bug in rospy.set_param.
        rospy.set_param('~pick_and_place_poses',
                        {'pick': pick_pose.tolist(),
                         'place': place_pose.tolist()})
        rospy.loginfo('pick_pose is %s' % pick_pose)
        rospy.loginfo('place_pose is %s' % place_pose)
    else:
        pick_pose = rospy.get_param('~pick_and_place_poses/pick')
        place_pose = rospy.get_param('~pick_and_place_poses/place')

    b = Baxter(limb_name)
    b.pick(pick_pose)
    b.place(place_pose)


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
