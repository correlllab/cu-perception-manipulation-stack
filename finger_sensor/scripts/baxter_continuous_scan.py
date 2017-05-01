#! /usr/bin/env python

from math import copysign, pi, cos

import numpy as np
from math import sqrt

import rospy
import tf
from std_msgs.msg import Int32MultiArray, Header
from geometry_msgs.msg import Vector3, Vector3Stamped, Pose, PoseStamped, Point, Quaternion
from std_msgs.msg import Header, Int64, Bool
from keyboard.msg import Key

from pap.baxter import Baxter
from pap.manager import PickAndPlaceNode

import random

import time #don't use old finger sensor values! only new
#left gripper pose:
# 0.745, 0.262, 0.122
# 0.172, 0.529, -0.286, 0.779


class FingerSensorBaxter(Baxter):
    def __init__(self, limb_name, topic='/sensor_values'):
        super(FingerSensorBaxter, self).__init__(limb_name)
        self.listen = tf.TransformListener()
        self.inside = np.zeros(14)
        self.tip = np.zeros(2)
        self.inside_offset = np.zeros_like(self.inside)
        self.tip_offset = np.zeros_like(self.tip)
        # Picking level
        self.level = None
        self.last_sensor_update = 0
        self.sensor_sub = rospy.Subscriber(topic,
                                           Int32MultiArray,
                                           self.update_sensor_values,
                                           queue_size=1)
        self.object_frame = ""
        self.perc_centroids = np.array([])

        self.touch_centroids = np.array([])

        self.update_transform = rospy.Publisher("/update_camera_alignment",
                                                Pose,
                                                queue_size = 1)
        self.handle_found = False

        self.handles_found = 0
        self.cups_scanned = 0
        #self.zero_sensor()
        #import ipdb;ipdb.set_trace()

    def getOffsetPoses(self, translation, quaternion, requrd_rot, requrd_trans):
        # print ('we are in the fucntion')
        # return 0
        matrix1 = self.listen.fromTranslationRotation(translation, quaternion)
        requrd_quat = tf.transformations.quaternion_from_euler(requrd_rot[0], requrd_rot[1], requrd_rot[2])
        # print (requrd_quat)
        matrix2 = self.listen.fromTranslationRotation(requrd_trans, requrd_quat) #identity matrix
        # print (matrix2)
        matrix3 =  np.zeros((4,4))
        matrix3 = np.dot(matrix1,matrix2)
        #get the euler angles from 4X4 T martix
        scale, shear, rpy_angles, trans_1, perps = tf.transformations.decompose_matrix(matrix3)
        # covert quaternion to euler
        quat_1 = tf.transformations.quaternion_from_euler(rpy_angles[0], rpy_angles[1], rpy_angles[2])

        #converting numpy.ndarray into tuple
        trans_1 = list(trans_1.tolist())
        quat_1 = list(quat_1)
        pose = trans_1 + quat_1
        return pose

    def move_cup(self):
        self.gripper.close()
        
        rotate = long(random.randrange(20, 180))*(3.14156)/(180) 
        print("rotate: " + str(rotate))
        t = self.tl.getLatestCommonTime("/root", "/right_gripper")
        cur_position, quaternion = self.tl.lookupTransform("/root",
                                                           "/right_gripper",
                                                           t)
        quaternion = [1,0,0,0]
        requrd_rot = (0,0,rotate) # in radians
        requrd_trans = (0,0,0)                
        rotated_pose = self.getOffsetPoses(cur_position, quaternion, requrd_rot, requrd_trans)
        trans_5= tuple(rotated_pose[:3])
        quat_5= tuple(rotated_pose[3:])
        br = tf.TransformBroadcaster()
        br.sendTransform(trans_5,
                                  quat_5,
                                  rospy.Time.now(),
                                  "pick_pose",
                                  "/root")  
        pick_pose = PoseStamped(Header(0, rospy.Time(0), n.robot.base),
                                Pose(Point(*trans_5),
                                Quaternion(*quat_5)))
        self.move_ik(pick_pose)
        self.gripper.open()

        t = self.tl.getLatestCommonTime("/root", "/right_gripper")
        cur_position, quaternion = self.tl.lookupTransform("/root",
                                                        "/right_gripper",
                                                           t)
        cur_position = list(cur_position)
        cur_position[2] = cur_position[2] + 0.08
        pose = Pose(Point(*cur_position),
                        Quaternion(*quaternion))
        while True:
            try:
                stamped_pose = PoseStamped( Header(0, rospy.Time(0), n.robot.base), pose)
                self.move_ik(stamped_pose)
                break
            except AttributeError:
                print("can't find valid pose for gripper cause I'm dumb")
                return False

        rospy.sleep(2)
            
        home = PoseStamped(
                    Header(0, rospy.Time(0), n.robot.base),
                    Pose(Point(0.60558, -0.24686, 0.093535),
                    Quaternion(0.99897, -0.034828, 0.027217, 0.010557)))
        self.move_ik(home) 
        rospy.sleep(2)



    def scan_cup(self, offset):
        if self.tl.frameExists("/cup_position"):
            frame_name = "/cup_position"
        else:
            frame_name = "/cup__position"
        self.object_frame = frame_name
        if self.tl.frameExists("/root") and self.tl.frameExists(frame_name):
            t = self.tl.getLatestCommonTime("/root", frame_name)
            position, quaternion = self.tl.lookupTransform("/root",
                                                           frame_name,
                                                           t)


            #new_position = list(position)
            #new_position[0] = new_position[0] + 0.01
            #new_position[1] = new_position[1] - 0.03
            br = tf.TransformBroadcaster()
            br.sendTransform(position,
                                  [1, 0, 0, 0],
                                  rospy.Time.now(),
                                  "pick_pose",
                                  "/root")
            pose = Pose(Point(*position),
                        Quaternion(1, 0, 0, 0))
            h = Header()
            h.stamp = rospy.Time(0)
            h.frame_id = "/root"
            stamped_pose = PoseStamped(h, pose)
            self.pick(stamped_pose)
            self.state = 'postpick'
            

            #x_index = 0
            #y_index = 0
            #x_offsets = (0.0, -1.39, -2.645,  -3.64, -4.28,
            #              -4.5, -4.28, -3.64, -2.645, -1.39, 0.0)
            #y_offsets = (4.5, 4.28, 3.64, 2.645, 1.39, 
            #               0.0, -1.39, -2.645,  -3.64, -4.28, -4.5)
            radian = 0
            rad_increase = (3.14156/2)/5
            rospy.sleep(3)
            self.handle_found = False
            rospy.sleep(1)
            
            count = 11
            if(offset):
                y_offset = 0
            else:
                y_offset = 0.01
            while(not self.handle_found):
                if(self.handle_found):
                    break
                t = self.tl.getLatestCommonTime("/root", frame_name)
                position, quaternion = self.tl.lookupTransform("/root",
                                                           frame_name,
                                                           t)
                quaternion = [1,0,0,0]
                requrd_rot = (0,0,radian) # in radians
                #requrd_trans = (x_offsets[x_index]/100,y_offsets[y_index]/100,.02)
                requrd_trans = (0,0,0)                
                rotated_pose = self.getOffsetPoses(position, quaternion, requrd_rot, requrd_trans)
                #print requrd_rot
                #print requrd_trans
                #print pick_pose
                pick_pose = self.getOffsetPoses(tuple(rotated_pose[:3]), tuple(rotated_pose[3:]), (0,0,0), (0,y_offset,0.04))
                trans_1= tuple(pick_pose[:3])
                quat_1= tuple(rotated_pose[3:])
            
                
                br.sendTransform(trans_1,
                                  quat_1,
                                  rospy.Time.now(),
                                  "pick_pose",
                                  "/root")  


                pick_pose = PoseStamped(Header(0, rospy.Time(0), n.robot.base),
                                Pose(Point(*trans_1),
                                Quaternion(*quat_1)))
                self.move_ik(pick_pose)
                '''
                requrd_rot = (3.14156,3.14156/2,0) # grippers down: 0,-3.14156/2,0
                requrd_trans = (.07,-.03,-.01) #(.07,-.03,-.01) end want
                grip_pose = self.getOffsetPoses(trans_1, quat_1, requrd_rot, requrd_trans)
                trans_2= tuple(grip_pose[:3])
                quat_2= tuple(grip_pose[3:])
                br.sendTransform(trans_2,
                                  quat_2,
                                  rospy.Time.now(),
                                  "grip_pose",
                                  "/root") 
                '''
                #x_index += 1
                #y_index += 1  
                radian += rad_increase 
                count = count - 1
                if(count == 0):
                    break
                #if(x_index == len(x_offsets) or self.handle_found):
                #    break
                #rospy.sleep(3)
            if(not self.handle_found):
                print("Handle not found :(")
        
            


    def _vector_to(self, vector, to='base'):
        h = Header()
        h.stamp = rospy.Time(0)
        h.frame_id = '{}_gripper'.format(self.limb_name)
        v = Vector3(*vector)
        v_base = self.tl.transformVector3(to,
                                          Vector3Stamped(h, v)).vector
        v_cartesian = [v_base.x, v_base.y, v_base.z, 0, 0, 0]
        return v_cartesian
    

    def update_sensor_values(self, msg):
        values = np.array(msg.data)
        self.inside = np.concatenate((values[:7],
                                      values[8:15])) - self.inside_offset

        self.tip = values[[7, 15]] - self.tip_offset
        error = np.mean(self.inside[7:] - self.inside[:7])
        # Experimentally, we need a deadband of about
        # 314-315. Otherwise it moves left or right with nothing in
        # between fingers.
        if abs(error) < 350:
            self.error = 0
        else:
            self.error = error
        self.last_sensor_update = time.time()

        if((((self.inside[10] > 1000) or (self.inside[11] > 1000) or (self.inside[12] > 1000))  and
             (self.inside[4]  > 400) and (self.inside[5]  > 400) and (self.inside[6]  > 400)) or
           (((self.inside[4] > 1000) or (self.inside[5] > 1000) or (self.inside[6] > 1000)) and
             (self.inside[10] > 400) and (self.inside[11] > 400) and (self.inside[12] > 400))):
            if(not self.handle_found): print "handle found"
            self.handle_found = True

    def sensors_updated(self):
        if((time.time() - self.last_sensor_update) < 2):
            return True
        else:
            print("Finger sensor data outdated. Aborting action")
            return False

    def pick(self, pose, direction=(0, 0, 1), distance=0.1):
        pregrasp_pose = self.translate(pose, direction, distance)
       
        while True:
            try:
                #stamped_pose = PoseStamped( Header(0, rospy.Time(0), n.robot.base), pose)
                self.move_ik(pregrasp_pose)
                break
            except AttributeError:
                print("can't find valid pose for gripper cause I'm dumb")
                return False
        rospy.sleep(0.5)
        # We want to block end effector opening so that the next
        # movement happens with the gripper fully opened.
        #self.gripper.open()

        
        while True:
            #rospy.loginfo("Going down to pick (at {})".format(self.tip.max()))
            if(not self.sensors_updated()):
                return
            if self.tip.max() > 2000:
                break
            else:
                scaled_direction = (di / 100 for di in direction)
                #print("Scaled direction: ", scaled_direction)
                v_cartesian = self._vector_to(scaled_direction)
                v_cartesian[2] = -.01
                #print("cartesian: ", v_cartesian)
                v_joint = self.compute_joint_velocities(v_cartesian)
                #print("joint    : ", v_joint)
                self.limb.set_joint_velocities(v_joint)

        rospy.loginfo('Went down!')

    def place(self, pose, direction=(0, 0, 1), distance=0.1):
   
        preplace_pose = self.translate(pose, direction, distance)
        self.move_ik(preplace_pose)
        rospy.sleep(0.5)
        # Edit this. Idea:
        # 1) Move down (direction) until maybe 1cm above place pose.
        # 2) Figure out whether one tip is closer or the other
        # 3) Move left-right to fix that, until values are similar from both tip sensors
        # 4) When kind of close, stop. Move down the missing cm.
        # 5) Open gripper
        # 6) Go to pregrasp
        while True:
            rospy.loginfo("Going down to pick (at {})".format(self.tip.max()))
            # Comment refers to cubelets:
            # 5900 as max tip value works well to stack two levels
            # high. Value experimentally found. Each block is 42 mm
            # high. Believe 5700 is the right value for two blocks
            # high.
            # Ideally, we'd have a fully calibrated sensor and these
            # would become distances
            # Cubelets limits:
            limit = {1: 5700, 2: 5500}
            # YCB limits:
            # limit = {1: 7450, 2: 7100, 9: 5520 + 50 - 20}  # 9: side stacking (11000 for bottom)
            if self.tip.max() > limit[self.level]:
                break
            else:
                scaled_direction = (di / 100 for di in direction)
                v_cartesian = self._vector_to(scaled_direction)
                v_joint = self.compute_joint_velocities(v_cartesian)
                self.limb.set_joint_velocities(v_joint)
        self.limb.set_joint_velocities(self.compute_joint_velocities([0] * 6))
        rospy.loginfo('Went down!')
        rospy.sleep(0.5)
        # If side stacking, do that

        if self.level == 9:
            done = False
            while not done:
                try:
                    cond = False
                    if cond:
                        done = True
                        break
                    # Hard code going backwards
                    scaled_direction = (di / 100 for di in (-1, 0, 0))
                    v_cartesian = self._vector_to(scaled_direction)
                    v_joint = self.compute_joint_velocities(v_cartesian)
                    self.limb.set_joint_velocities(v_joint)
                except KeyboardInterrupt:
                    break
        else:
            A = 0.0  # Centering amplitude (SI)
            theta = pi  # rad/s, so the centering wave takes 2s
            t_0 = rospy.Time.now().to_sec()
            r = rospy.Rate(50)
            tip = []
            while not rospy.is_shutdown():
                # We could do something more clever and command the
                # midpoint velocity from now until the next command, but
                # not really that important
                t = rospy.Time.now().to_sec()
                if theta * (t - t_0) > (2 * pi):
                    break
                y_v = A * theta * cos(theta * (t - t_0))
                v_cartesian = self._vector_to((0, y_v, 0))
                v_joint = self.compute_joint_velocities(v_cartesian)
                self.limb.set_joint_velocities(v_joint)
                tip.append(sum(self.tip))
            r = rospy.Rate(20)
            done = False
            #import pdb; pdb.set_trace()
            while not done:
                rospy.loginfo("centering")
                delta = - (self.tip[0] - self.tip[1])
                rospy.loginfo("Delta is {} ({})".format(delta, 'positive' if delta >= 0 else 'negative'))
                if abs(delta) < 400:
                    done = True
                else:
                    v_cartesian = self._vector_to((0, copysign(0.005, delta), 0))
                    v_joint = self.compute_joint_velocities(v_cartesian)
                    self.limb.set_joint_velocities(v_joint)
                    r.sleep()
            # We're usually over by now, so let's go back a little bit
            r = rospy.Rate(20)
            for i in range(2):
                v_cartesian = self._vector_to((0, -copysign(0.005, delta), 0))
                v_joint = self.compute_joint_velocities(v_cartesian)
                self.limb.set_joint_velocities(v_joint)
                r.sleep()
            self.limb.set_joint_velocities(self.compute_joint_velocities([0] * 6))
            rospy.loginfo("centered")
        rospy.sleep(0.5)
        #self.move_ik(pose)
        #rospy.sleep(0.5)
        self.gripper.open()
        rospy.sleep(0.5)
        self.move_ik(preplace_pose)

if __name__ == '__main__':
    n = PickAndPlaceNode(FingerSensorBaxter, 'right')
    home = PoseStamped(
            Header(0, rospy.Time(0), n.robot.base),
            Pose(Point(0.60558, -0.24686, 0.093535),
                 Quaternion(0.99897, -0.034828, 0.027217, 0.010557)))
    n.robot.move_ik(home) 

    raw_input("Press Enter to continue...")
    #n.robot.gripper.command_position( position = 10)
    #raw_input("Press Enter to continue...")
    perception_pub = rospy.Publisher("/perception/enabled", Bool, queue_size=1)
    tests = 0
    vision_c = 0
    touch_c = 0
    atleast_1_c = 0
    while(True):
        #command perception to true
        perception_pub.publish(Bool(True))
        stop_loop = False
        
        while(not stop_loop or good == "n"):
            loop_wait = 0
            while(loop_wait < 300):
                perception_pub.publish(Bool(True))
                rospy.sleep(0.01)
                loop_wait += 1
            stop_loop = n.robot.tl.frameExists("cup_position")
            stop_loop = n.robot.tl.frameExists("cup__position") or stop_loop
            perception_pub.publish(Bool(False))
            good = raw_input("Perception successful? y/n:")
            
        #command perception to false
        
        vision = raw_input("Is the vision handle correct? y/n:")
        rospy.sleep(3)
        n.robot.gripper.open()
        n.robot.scan_cup(vision == "y")
        sensors = raw_input("Am I correct? y/n:")
        count = raw_input("Count this experiment? y/n:")
        if(count == "y"):
            tests += 1
            if(sensors == "y"): touch_c += 1
            if(vision == "y"): vision_c += 1
            if(vision  == "y" or sensors == "y"): atleast_1_c += 1
            print("vision correct:" + str(vision_c))
            print("touch correct:" + str(touch_c))
            print("atleast one correct:" + str(atleast_1_c)) 
            print("number of tests:" + str(tests))
           
        n.robot.move_cup()
            
                
    

