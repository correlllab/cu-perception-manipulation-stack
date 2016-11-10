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
    gripper_pos = [50.0,50.0,0.0]
    jg.set_position(gripper_pos)
    fingers_to_use = [0,1]
    sm = smach.StateMachine(outcomes=['Success','Fail'])
    sm.userdata.tower_size = 0
    with sm:
        smach.StateMachine.add('StackingStart',
                                action_database.StackingStart(),
                                transitions={'Start':'PerceiveObjects1',
                                                'Ready':'GenerateCubeGrasp1'},
                                remapping={'tower_size_in':'tower_size',
                                            'tower_size_out':'tower_size'})

        smach.StateMachine.add('PerceiveObjects1',
                                action_database.PerceiveObjects(['unknown_1','unknown_0']),
                                transitions={'objects_found':'GenerateCubeGrasp1',
                                            'objects_not_found':'Success'},
                                remapping={'pick_obj_name_out':'pick_object_name',
                                            'place_obj_name_out':'place_object_name'})

        smach.StateMachine.add('GenerateCubeGrasp1',
                                action_database.GenerateCubeGrasp(),
                                transitions={'grasp_generated':'CalibrateFingers1',
                                            'grasp_not_generated':'GenerateCubeGrasp1'},
                                remapping={'pick_obj_name_in':'pick_object_name',
                                            'place_obj_name_in':'place_object_name',
                                            'tower_size_in':'tower_size'})

        smach.StateMachine.add('CalibrateFingers1',
                                action_database.CalibrateFingers(pos=gripper_pos),
                                transitions={'calibrated':'GotoObject1',
                                            'not_calibrated':'CalibrateFingers1'})

        smach.StateMachine.add('GotoObject1',
                                action_database.GotoObject('/pick_frame'),
                                transitions={'there':'CalibrateFingers2',
                                            'no_tf_found':'GotoObject1'})

        smach.StateMachine.add('SearchObject3',
                                action_database.SearchObject(fingers_to_use,search='down_z'),
                                transitions={'found':'GraspObject1',
                                            'not_found': 'SearchObject1'})

        smach.StateMachine.add('SearchObject1',
                                action_database.SearchObject(fingers_to_use),
                                transitions={'found':'GraspObject1',
                                            'not_found': 'GraspObject1'})

        smach.StateMachine.add('GraspObject1',
                        action_database.GraspObject(gripper_pos,third_finger=False),
                        transitions={'grasped':'GotoObject2',
                                    'not_grasped': 'CalibrateFingers2'})

        smach.StateMachine.add('GotoObject2',
                                action_database.GotoObject('/place_frame'),
                                transitions={'there':'CalibrateFingers3',
                                            'no_tf_found':'GotoObject2'})

        smach.StateMachine.add('CalibrateFingers2',
                                action_database.CalibrateFingers(pos=gripper_pos),
                                transitions={'calibrated':'SearchObject3',
                                            'not_calibrated':'CalibrateFingers2'})

        smach.StateMachine.add('CalibrateFingers3',
                                action_database.CalibrateFingers(),
                                transitions={'calibrated':'SearchObject2',
                                            'not_calibrated':'CalibrateFingers3'})


        smach.StateMachine.add('SearchObject2',
                                action_database.SearchObject(fingers_to_use,search='down_z'),
                                transitions={'found':'StackingStart',
                                            'not_found': 'StackingStart'})


    outcome = sm.execute()

    jg.set_position([50.0,50.0,0.0])


if __name__ == '__main__':
    main()
