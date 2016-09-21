from robot import Robot
import rospy
import kinova_msgs.msg
from kinova_msgs.srv import HomeArm

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

