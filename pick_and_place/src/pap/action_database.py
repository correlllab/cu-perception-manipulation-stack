#! /usr/bin/env python
from __future__ import division, print_function, absolute_import
import sys
import rospy
from pap.jaco import Jaco, JacoGripper
from pap.manager import PickAndPlaceNode
from kinova_msgs.msg import JointAngles, PoseVelocity
from finger_sensor_msgs.msg import FingerDetect, FingerTouch

from std_msgs.msg import Header, Int32MultiArray, Bool, String, Int64
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
import numpy as np

import commands
import tf
import smach
import time

class StackingStart(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Ready','Start'],
                            input_keys=['tower_size_in'],
                            output_keys=['tower_size_out'])
        self.jn = Jaco()
        self.jgn = JacoGripper()

    def execute(self,userdata):
        if userdata.tower_size_in == 0:
            self.jn.home()
            userdata.tower_size_out = userdata.tower_size_in + 1
            return 'Start'
        else:
            self.jgn.set_position([50.0,50.0,0.0])
            self.jn.home()
            userdata.tower_size_out = userdata.tower_size_in + 1
            return 'Ready'

class PerceiveObjects(smach.State):
    def __init__(self,object_list,perception_time=2.5):
        smach.State.__init__(self, outcomes=['objects_found', 'objects_not_found'],
                            output_keys=['pick_obj_name_out','place_obj_name_out'])

        self.perception_pub = rospy.Publisher("/perception/enabled",
                                              Bool,
                                              queue_size=1)

        self.listen = tf.TransformListener()
        self.perception_time = perception_time
        self.object_list = object_list

    def execute(self,userdata):
        time.sleep(1)
        self.perception_pub.publish(Bool(True))
        time.sleep(self.perception_time)
        self.perception_pub.publish(Bool(False))
        frames = self.listen.getFrameStrings()
        object_frames = [of for of in frames if of in self.object_list]
        if object_frames[0] and object_frames[1]:
            translation1, quaternion1 = self.listen.lookupTransform("/root", object_frames[0], rospy.Time(0))
            translation2, quaternion2 = self.listen.lookupTransform("/root", object_frames[1], rospy.Time(0))
            if translation1[2] > translation2[2]:
                userdata.place_obj_name_out = object_frames[0]
                userdata.pick_obj_name_out = object_frames[1]
            else:
                userdata.place_obj_name_out = object_frames[1]
                userdata.pick_obj_name_out = object_frames[0]
            return 'objects_found'
        else:
            return 'objects_not_found'

class GenerateCubeGrasp(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['grasp_generated', 'grasp_not_generated'],
                            input_keys=['pick_obj_name_in','place_obj_name_in','tower_size_in'])
        self.listen = tf.TransformListener()
        self.pick_obj_name_pub = rospy.Publisher('/place_frame_name',String,queue_size=1)
        self.place_obj_name_pub = rospy.Publisher('/pick_frame_name',String,queue_size=1)
        self.tower_size_pub = rospy.Publisher('/tower_size',Int64,queue_size=1)


    def execute(self,userdata):
        self.tower_size_pub.publish(userdata.tower_size_in)
        time.sleep(1)
        self.place_obj_name_pub.publish("/unknown_{}".format(userdata.tower_size_in))
        self.pick_obj_name_pub.publish("/unknown_0")
        if self.listen.frameExists("/unknown_{}".format(userdata.tower_size_in)) and self.listen.frameExists("/unknown_0"):
            return 'grasp_generated'
        else:
            return 'grasp_not_generated'

class CalibrateFingers(smach.State):
    def __init__(self,pos=None):
        smach.State.__init__(self, outcomes=['calibrated','not_calibrated'])
        self.calibrate_pub = rospy.Publisher('/finger_sensor/calibrate_obj_det',
                                            Bool,
                                            queue_size=1)
        self.calibrated_sub = rospy.Subscriber('/finger_sensor/obj_det_calibrated',
                                                Bool,
                                                self.set_calibrated)
        self.calibrated = None
        self.finger_pos = pos
        self.jgn = JacoGripper()

    def set_calibrated(self,msg):
        self.calibrated = msg.data

    def execute(self,userdata):
        if self.finger_pos:
            self.jgn.set_position(self.finger_pos)
        self.calibrate_pub.publish(True)
        time.sleep(5)
        while self.calibrated == False:
            pass
        return 'calibrated'

class GotoObject(smach.State):
    def __init__(self,frame):
        smach.State.__init__(self, outcomes=['there','no_tf_found'])
        self.frame = frame
        self.jn = Jaco()
        self.listen = tf.TransformListener()

    def execute(self,userdata):
        if self.listen.frameExists("/root") and self.listen.frameExists(self.frame):
            self.listen.waitForTransform('/root',self.frame,rospy.Time(),rospy.Duration(100.0))
            t = self.listen.getLatestCommonTime("/root", self.frame)
            translation, quaternion = self.listen.lookupTransform("/root", self.frame, t)

            translation =  list(translation)
            quaternion = list(quaternion)
            msg = self.make_pose_stamped_msg(translation,quaternion)
            self.jn.move_ik(msg)
            return 'there'

        else:
            return 'no_tf_found'

    def make_pose_stamped_msg(self,translation,orientation,frame='/j2n6a300_link_base'):
        msg = PoseStamped()
        msg.header.frame_id = frame
        msg.header.stamp = rospy.Time.now()
        msg.pose.position.x = translation[0]
        msg.pose.position.y = translation[1]
        msg.pose.position.z = translation[2]
        msg.pose.orientation.x = orientation[0]
        msg.pose.orientation.y = orientation[1]
        msg.pose.orientation.z = orientation[2]
        return msg

class GraspObject(smach.State):
    def __init__(self,start,third_finger=True):
        smach.State.__init__(self, outcomes=['grasped', 'not_grasped'])
        self.finger_touch_sub = rospy.Subscriber('/finger_sensor/touch',
                                                FingerTouch,
                                                self.set_finger_touch)
        self.finger_touch = [None, None, None]
        self.jgn = JacoGripper()
        self.start = start
        self.third_finger = third_finger

    def set_finger_touch(self,msg):
        self.finger_touch[0] = msg.finger1
        self.finger_touch[1] = msg.finger2
        self.finger_touch[2] = msg.finger3

    def execute(self,userdata):
        status = self.jgn.close_with_feedback(self.start,self.third_finger)
        if np.any(np.array(self.finger_touch) == True) and status == 'done':
            return 'grasped'
        else:
            return 'not_grasped'

class SearchObject(smach.State):
    def __init__(self,fingers,search='back_forth_search_xy'):
        smach.State.__init__(self, outcomes=['found', 'not_found'])
        self.finger_detect_sub = rospy.Subscriber('/finger_sensor/obj_detected',
                                                FingerDetect,
                                                self.set_finger_detect)
        self.finger_detect = [None, None, None]
        self.jn = Jaco()
        self.detect_goal = [True] * len(fingers)
        self.fingers = fingers
        self.listen = tf.TransformListener()
        self.transformer = tf.TransformerROS()
        searches = {'back_forth_search_xy':self.back_forth_search_xy,
                    'down_z': self.down_z}
        self.search_motion = searches[search]

    def set_finger_detect(self,msg):
        self.finger_detect[0] = msg.finger1
        self.finger_detect[1] = msg.finger2
        self.finger_detect[2] = msg.finger3

    def create_pose_velocity_msg(self,cart_velo,transform=False):
        if transform:
            if self.listen.frameExists("/root") and self.listen.frameExists("/j2n6a300_end_effector"):
                self.listen.waitForTransform('/root',"/j2n6a300_end_effector",rospy.Time(),rospy.Duration(100.0))
                t = self.listen.getLatestCommonTime("/root", "/j2n6a300_end_effector")
                translation, quaternion = self.listen.lookupTransform("/root", "/j2n6a300_end_effector", t)
            transform = self.transformer.fromTranslationRotation(translation, quaternion)
            transformed_vel = np.dot(transform,(cart_velo[0:3]+[1.0]))
            print(transformed_vel)
            cart_velo[0:3] = transformed_vel[0:3]
        msg = PoseVelocity(
        twist_linear_x=cart_velo[0],
        twist_linear_y=cart_velo[1],
        twist_linear_z=cart_velo[2],
        twist_angular_x=cart_velo[3],
        twist_angular_y=cart_velo[4],
        twist_angular_z=cart_velo[5])
        return msg

    def execute(self,userdata):
        if np.all(np.array(self.finger_detect)[[self.fingers]] == np.array(self.detect_goal)):
            return 'found'
        else:
            found = self.search_motion()
            return found

    def back_forth_search_xy(self):
        rate = rospy.Rate(100)
        for i in range(100):
            if np.all(np.array(self.finger_detect) == np.array(self.detect_goal)):
                return 'found'
            msg = self.create_pose_velocity_msg([0.0,-0.05,0.0,0.0,0.0,0.0])
            self.jn.kinematic_control(msg)
            rate.sleep()
        for i in range(200):
            if np.all(np.array(self.finger_detect)[[self.fingers]] == np.array(self.detect_goal)):
                return 'found'
            msg = self.create_pose_velocity_msg([0.0,0.05,0.0,0.0,0.0,0.0])
            self.jn.kinematic_control(msg)
            rate.sleep()
        for i in range(100):
            if np.all(np.array(self.finger_detect) == np.array(self.detect_goal)):
                return 'found'
            msg = self.create_pose_velocity_msg([0.0,-0.05,0.0,0.0,0.0,0.0])
            self.jn.kinematic_control(msg)
            rate.sleep()
        return 'not_found'

    def down_z(self):
        rate = rospy.Rate(100)
        while True:
            if np.any(np.array(self.finger_detect)[[self.fingers]] == np.array(self.detect_goal)):
                return 'found'
            choices = np.array([0.0,0.075,-0.075,0.025,-0.025,0.05,-0.05])
            x = np.random.choice(choices)
            y = np.random.choice(choices)
            z = -0.02
            msg = self.create_pose_velocity_msg([x,y,z,0.0,0.0,0.0])
            self.jn.kinematic_control(msg)
            rate.sleep()
        return 'not_found'
