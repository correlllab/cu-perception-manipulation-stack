#!/usr/bin/env python

import rospy
import numpy as np
import tf
from pap.jaco import Jaco, JacoGripper
import commands
import time
import actionlib
import kinova_msgs.msg
from std_msgs.msg import Bool


def gripper_client(finger_positions):
    """Send a gripper goal to the action server."""
    action_address = '/j2n6a300_driver/fingers_action/finger_positions'

    client = actionlib.SimpleActionClient(action_address,
                                          kinova_msgs.msg.SetFingersPositionAction)
    client.wait_for_server()

    goal = kinova_msgs.msg.SetFingersPositionGoal()
    goal.fingers.finger1 = float(finger_positions[0])
    goal.fingers.finger2 = float(finger_positions[1])
    # The MICO arm has only two fingers, but the same action definition is used
    if len(finger_positions) < 3:
        goal.fingers.finger3 = 0.0
    else:
        goal.fingers.finger3 = float(finger_positions[2])
    print('goal')
    print(goal)
    client.send_goal(goal)
    if client.wait_for_result(rospy.Duration(5.0)):
        return client.get_result()
    else:
        client.cancel_all_goals()
        rospy.WARN('        the gripper action timed-out')
        return None

def cfp(finger_value):
    commands.getoutput('rosrun kinova_demo fingers_action_client.py j2n6a300 percent -- {0} {1} {2}'.format(finger_value[0],finger_value[1],finger_value[2]))


if __name__ == '__main__':
    rospy.init_node("test")
    jg = JacoGripper()
    jg.set_position([50,50,50])
    #jg.close_with_feedback([0,0,0],three_fingers=False)
    calibrate_obj_det_pub = rospy.Publisher("/finger_sensor/calibrate_obj_det",
                                        Bool,
                                        queue_size=1)

    calibrate_obj_det_pub.publish(True)


    # jg.open()
    # cfp([0,0,0])
    # cfp([50,50,50])
    # # cfp([100,100,100])
    # # print(jg.set_position([0,0,0]))
    # for i in xrange(0,20):
    #     # pp = [i*20.0] * 3
    #     # pt = np.clip(pp, 0, 100) / 100 * 6800
    #     # pt = np.clip(pt,0,6800)
    #     # print(gripper_client(pt))
    #
    #     #cfp([i*20,i*20,i*20])
    #     jg.set_position(np.array([i*5]*5))
    # jg.open()
    # print(jg.currentFingerPercent)
    # jg.close_with_feedback()
    # print(jg.currentFingerPercent)
    rospy.spin()
