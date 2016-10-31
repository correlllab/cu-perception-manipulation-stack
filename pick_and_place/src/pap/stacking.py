#!/usr/bin/env python

import rospy
import numpy as np
import tf
import action_database
import smach
from pap.jaco import JacoGripper

def main():
    rospy.init_node("test")
    jg = JacoGripper()
    jg.set_position([50.0,50.0,0.0])
    sm = smach.StateMachine(outcomes=['Done','hi'])
    with sm:
        smach.StateMachine.add('GotoObject1',
                                action_database.GotoObject('/cube_0_grasp'),
                                transitions={'there':'SearchObject1',
                                            'no_tf_found':'GotoObject1'})

        smach.StateMachine.add('SearchObject1',
                                action_database.SearchObject([True,True,False]),
                                transitions={'found':'Done',
                                            'not_found': 'GotoObject1'})

    outcome = sm.execute()

if __name__ == '__main__':
    main()
