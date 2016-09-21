from robot import Robot
from baxter_pykdl import baxter_kinematics
import baxter_interface

from baxter_core_msgs.srv import SolvePositionIK, SolvePositionIKRequest
from baxter_core_msgs.msg import DigitalIOState


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
