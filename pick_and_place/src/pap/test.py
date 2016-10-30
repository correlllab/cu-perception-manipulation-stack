#!/usr/bin/env python

import rospy
import numpy as np
import tf
import action_database
import smach

def main():
    rospy.init_node("test")
    sm = smach.StateMachine(outcomes=['Done','hi'])
    with sm:
        smach.StateMachine.add('GotoObject',
                                action_database.GotoObject('/cube_1_grasp'),
                                transitions={'there':'Done',
                                            'no_tf_found':'GotoObject'})

    outcome = sm.execute()

if __name__ == '__main__':
    main()
