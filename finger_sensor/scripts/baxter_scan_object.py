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

import thread
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

    def publish_transform(self, name, pos, quat):
        broadcast = tf.TransformBroadcaster()
        while(True):
            broadcast.sendTransform(tuple(pos), quat,
                                    rospy.Time.now(),
                                    name,
                                    "root")
            rospy.sleep(1)

    def scan_cup(self):
        frame_name = "/cup_position"
        self.object_frame = frame_name
        if self.tl.frameExists("/root") and self.tl.frameExists(frame_name):
            t = self.tl.getLatestCommonTime("/root", frame_name)
            position, quaternion = self.tl.lookupTransform("/root",
                                                           frame_name,
                                                           t)

            #perception_pub = rospy.Publisher("/finger_perception/enabled", Bool, 1)
            #perception_pub.publish(Bool(False))
            new_position = list(position)
            new_position[0] = new_position[0]# + 0.01
            new_position[1] = new_position[1]

            cup_position = list(position)
            cup_position[0] = cup_position[0]
            cup_position[1] = cup_position[1]# - 0.01

            br = tf.TransformBroadcaster()
            br.sendTransform(new_position,
                                  [1, 0, 0, 0],
                                  rospy.Time.now(),
                                  "pick_pose",
                                  "/root")
            pose = Pose(Point(*new_position),
                        Quaternion(1, 0, 0, 0))
            h = Header()
            h.stamp = t
            h.frame_id = "/root"
            stamped_pose = PoseStamped(h, pose)
            if( not self.sensors_updated()):
                return
            self.pick(stamped_pose)
            self.state = 'postpick'
            

            radian = 0
            rad_increase = (3.14156/2)/5
            rospy.sleep(3)
            self.handle_found = False
            count = 12
            y_offset = 0.01
            #self.gripper.close()
            raw_input("Press Enter to continue...")
            #perception_pub.publish(Bool(True))
            while(True):#not self.handle_found):
                #t = self.tl.getLatestCommonTime("/root", frame_name)
                #position, quaternion = self.tl.lookupTransform("/root",
                                                           #frame_name,
                                                           #t)
                quaternion = [1,0,0,0]
                requrd_rot = (0,0,radian) # in radians
                #requrd_trans = (x_offsets[x_index]/100,y_offsets[y_index]/100,.02)
                requrd_trans = (0,0,0)                
                rotated_pose = self.getOffsetPoses(cup_position, quaternion, requrd_rot, requrd_trans)
                #print requrd_rot
                #print requrd_trans
                #print pick_pose
                pick_pose = self.getOffsetPoses(tuple(rotated_pose[:3]), tuple(rotated_pose[3:]), (0,0,0), (0,y_offset,0.03))
                trans_1= tuple(pick_pose[:3])
                quat_1= tuple(rotated_pose[3:])
            
                
                br.sendTransform(trans_1,
                                  quat_1,
                                  rospy.Time.now(),
                                  "pick_pose",
                                  "/root")  



                thread.start_new_thread(self.publish_transform, ("pose_" + str(count), trans_1, quat_1))
                pick_pose = PoseStamped(Header(0, rospy.Time(0), n.robot.base),
                                Pose(Point(*trans_1),
                                Quaternion(*quat_1)))
                if( not self.sensors_updated()):
                    return
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
                if(radian >= (3.14156/2)): 
                    y_offset = 0
                    #raw_input("Press Enter to continue...")
                count = count - 1
                rospy.sleep(3)
                if(radian > (3.1)):
                    break
                
                #if(x_index == len(x_offsets) or self.handle_found):
                #    break
                #rospy.sleep(3)
            handle = self.handle_found
            #perception_pub.publish(Bool(False))
            print("Moving up...maybe?")
            raw_input("Press Enter to continue...")
            t = self.tl.getLatestCommonTime("/root", "/right_gripper")
            cur_position, quaternion = self.tl.lookupTransform("/root",
                                                           "/right_gripper",
                                                           t)
            cur_position = list(cur_position)
            cur_position[2] = cur_position[2] + 0.08
            pose = Pose(Point(*cur_position),
                        Quaternion(*quaternion))
            stamped_pose = PoseStamped( Header(0, rospy.Time(0), n.robot.base), pose)
            self.move_ik(stamped_pose)
            rospy.sleep(2)
            
            home = PoseStamped(
                    Header(0, rospy.Time(0), n.robot.base),
                    Pose(Point(0.60558, -0.24686, 0.093535),
                    Quaternion(0.99897, -0.034828, 0.027217, 0.010557)))
            self.move_ik(home) 
            rospy.sleep(2)





    def update_transformation(self, position, orientation):
        # number of points
        print("original: ")
        print position
        N = len(self.perc_centroids)
        print("N: " + str(N))
        P = (np.mat(self.perc_centroids)).T
        Q = (np.mat(self.touch_centroids)).T
        
        centroid_P = np.mean(P, axis=1)
        centroid_Q = np.mean(Q, axis=1)
        print centroid_P
        print centroid_Q
        # centre the points
        PP = P - np.tile(centroid_P, (1,N))
        QQ = Q - np.tile(centroid_Q, (1,N))

        # dot is matrix multiplication for array
        cov = PP * np.transpose(QQ)

        U, S, Vt = np.linalg.svd(cov)
        refl = np.identity(len(U))
        refl[len(U)-1][len(U)-1] = np.linalg.det(Vt*np.transpose(U))
        R = Vt.T*refl*np.transpose(U)
        # special reflection case
        '''if np.linalg.det(R) < 0:
            print "Reflection detected"
            Vt[2,:] *= -1
            R = Vt.T * U.T'''
        #R2 = Vt.T * U.T
        print R
        t = -R*centroid_P + centroid_Q
        print("translation:")
        print t
        #scale, shear, rpy_angles, translation_vector, perspective = tf.transformations.decompose_matrix(transformation_matrix)
        og_point = np.mat(position)
        print("og_point:")
        print og_point
        new_position = R*og_point.T + t
        print("new position:")
        print new_position
        ret_value = [i for i in new_position.flat]
        print ret_value

        M = np.empty((4, 4))
        M[:3, :3] = R
        M[:3, 3] = t.flat
        M[3, :] = [0, 0, 0, 1]
        print M
        #t = tf.TransformerROS()
        quat = tf.transformations.quaternion_from_matrix(M)
        print("quat")
        print quat
        #mult_quat = tf.transformations.quaternion_multiply(orientation, quat)
        #print mult_quat
        #trans = tf.transformations.quaternion_from_matrix(t, t.quaternion_from_matrix(R))
        #print("transformation matrix:")
        #print trans
        #print t.quaternion_from_matrix(trans) #this is a 4x4 matrix
        #tf_conversions.transformations.quaternion_from_euler

        
        return ret_value, quat
        '''
        A2 = (ret_R*A.T) + np.tile(ret_t, (1, n))
        A2 = A2.T

        # Find the error
        err = A2 - B

        err = multiply(err, err)
        err = sum(err)
        rmse = sqrt(err/n);
        '''

    def _vector_to(self, vector, to='base'):
        h = Header()
        h.stamp = rospy.Time(0)
        h.frame_id = '{}_gripper'.format(self.limb_name)
        v = Vector3(*vector)
        v_base = self.tl.transformVector3(to,
                                          Vector3Stamped(h, v)).vector
        v_cartesian = [v_base.x, v_base.y, v_base.z, 0, 0, 0]
        return v_cartesian
    
    def update_camera_transform(self):
        t = self.tl.getLatestCommonTime("/root", "/camera_link")
        position, quaternion = self.tl.lookupTransform("/root",
                                                       "/camera_link",
                                                       t)
        position, quaternion = self.update_transformation(list(position), list(quaternion))

        #get ICP to find rotation and translation
        #multiply old position and orientation by rotation and translation
        #publish new pose to /update_camera_alignment
        self.update_transform.publish( Pose(Point(*position), Quaternion(*quaternion)))


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
        if((self.inside[10] > 1100) or (self.inside[11] > 1100)or (self.inside[12] > 1100) ):
            #print("value above 1100")
            self.handle_found = True

    def sensors_updated(self):
        if((time.time() - self.last_sensor_update) < 2):
            return True
        else:
            print("Finger sensor data outdated. Aborting action")
            return False

    def pick(self, pose, direction=(0, 0, 1), distance=0.1):
        pregrasp_pose = self.translate(pose, direction, distance)

        self.move_ik(pregrasp_pose)
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
    
    def scan_cube(self, pose, direction=(0, 0, 1), distance=0.1):
        points = list()
        pregrasp_pose = self.translate(pose, (-.3, 0, 1), distance)

        self.move_ik(pregrasp_pose)
        rospy.sleep(0.5)
        # We want to block end effector opening so that the next
        # movement happens with the gripper fully opened.
        #self.gripper.open()

        
        while True:
            #rospy.loginfo("Going down to pick (at {})".format(self.tip.max()))
            if(not self.sensors_updated()):
                return
            if self.tip.max() > 10000:
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
        rospy.sleep(0.5)
        rospy.loginfo('Touching side of block')
        count = 0
        while True:
            if(not self.sensors_updated()):
                return
            #rospy.loginfo(count)
            #rospy.loginfo("Moving to touch (at {})".format(self.inside[6]))
            if self.inside[6] > 10000:
                break
            else:
                scaled_direction = (di / 100 for di in direction)
                #print("Scaled direction: ", scaled_direction)
                v_cartesian = self._vector_to(scaled_direction)
                v_cartesian[1] = .005
                #print("cartesian: ", v_cartesian)
                v_joint = self.compute_joint_velocities(v_cartesian)
                #print("joint    : ", v_joint)
                self.limb.set_joint_velocities(v_joint)
                count=count+1
                rospy.sleep(0.25)
            if(count > 50):
                rospy.loginfo("Searching for block again")
                pose.pose.position.x = pose.pose.position.x + .01
                h = Header()
                h.stamp = self.tl.getLatestCommonTime("/root", self.object_frame)
                h.frame_id = "/root"
                pose.header = h
                self.pick(pose)
                return
        t = self.tl.getLatestCommonTime("/root", "/r_gripper_l_finger_tip")
        position, quaternion = self.tl.lookupTransform("/root",
                                                       "/r_gripper_l_finger_tip",
                                                         t)
        points.append(position)


        #self.side_of_block_1 = 
        count = 0
        while True:
            if(not self.sensors_updated()):
                return
            #rospy.loginfo(count)
            #rospy.loginfo("Moving to touch (at {})".format(self.inside[6]))
            if self.inside[6] > 10000:
                break
            else:
                scaled_direction = (di / 100 for di in direction)
                #print("Scaled direction: ", scaled_direction)
                v_cartesian = self._vector_to(scaled_direction)
                v_cartesian[1] = -.005
                #print("cartesian: ", v_cartesian)
                v_joint = self.compute_joint_velocities(v_cartesian)
                #print("joint    : ", v_joint)
                self.limb.set_joint_velocities(v_joint)
                count=count+1
                rospy.sleep(0.25)
            if(count > 10):
                break

       

        rospy.loginfo('Touching along the side of the block')
        while True:
            if(not self.sensors_updated()):
                return
            #rospy.loginfo("Moving to touch (at {})".format(self.inside[6]))
            if self.inside[6] < 7000:
                break
            else:
                scaled_direction = (di / 100 for di in direction)
                #print("Scaled direction: ", scaled_direction)
                v_cartesian = self._vector_to(scaled_direction)
                v_cartesian[0] = .005
                #print("cartesian: ", v_cartesian)
                v_joint = self.compute_joint_velocities(v_cartesian)
                #print("joint    : ", v_joint)
                self.limb.set_joint_velocities(v_joint)
                rospy.sleep(0.25)


        t = self.tl.getLatestCommonTime("/root", "/r_gripper_l_finger_tip")
        position, quaternion = self.tl.lookupTransform("/root",
                                                       "/r_gripper_l_finger_tip",
                                                         t)


        points.append(position)
            
        '''
        while True:
            if(not self.sensors_updated()):
                return
            #rospy.loginfo("Moving to touch (at {})".format(self.inside[6]))
            if self.inside[6] < 7000:
                break
            else:
                scaled_direction = (di / 100 for di in direction)
                #print("Scaled direction: ", scaled_direction)
                v_cartesian = self._vector_to(scaled_direction)
                v_cartesian[0] = -.005
                #print("cartesian: ", v_cartesian)
                v_joint = self.compute_joint_velocities(v_cartesian)
                #print("joint    : ", v_joint)
                self.limb.set_joint_velocities(v_joint)
                rospy.sleep(0.25)

        t = self.tl.getLatestCommonTime("/root", "/r_gripper_l_finger_tip")
        position, quaternion = self.tl.lookupTransform("/root",
                                                       "/r_gripper_l_finger_tip",
                                                         t)
        print("position", position)
        print("quaternion", quaternion)
        '''
        #below stuff touches other side of block
        rospy.loginfo('Touching other side of block')
        while True:
            if(not self.sensors_updated()):
                return
            #rospy.loginfo("Moving to touch (at {})".format(self.inside[6]))
            if self.inside[13] > 10000:
                break
            else:
                scaled_direction = (di / 100 for di in direction)
                #print("Scaled direction: ", scaled_direction)
                v_cartesian = self._vector_to(scaled_direction)
                v_cartesian[1] = -.005
                #print("cartesian: ", v_cartesian)
                v_joint = self.compute_joint_velocities(v_cartesian)
                #print("joint    : ", v_joint)
                self.limb.set_joint_velocities(v_joint)
                rospy.sleep(0.25)

        t = self.tl.getLatestCommonTime("/root", "/r_gripper_r_finger_tip")
        position, quaternion = self.tl.lookupTransform("/root",
                                                       "/r_gripper_r_finger_tip",
                                                         t)

        points.append(position)
        rospy.loginfo('Touching along the side of the block')
        while True:
            if(not self.sensors_updated()):
                return
            #rospy.loginfo("Moving to touch (at {})".format(self.inside[6]))
            if self.inside[13] < 7000:
                break
            else:
                scaled_direction = (di / 100 for di in direction)
                #print("Scaled direction: ", scaled_direction)
                v_cartesian = self._vector_to(scaled_direction)
                v_cartesian[0] = -.005
                #print("cartesian: ", v_cartesian)
                v_joint = self.compute_joint_velocities(v_cartesian)
                #print("joint    : ", v_joint)
                self.limb.set_joint_velocities(v_joint)
                rospy.sleep(0.25)

        t = self.tl.getLatestCommonTime("/root", "/r_gripper_r_finger_tip")
        position, quaternion = self.tl.lookupTransform("/root",
                                                       "/r_gripper_r_finger_tip",
                                                         t)


        points.append(position)
        x = 0
        y = 0
        z = 0
        rospy.loginfo('Touched both sides of the block')
        self.move_ik(pregrasp_pose)

        for point in points:
            x = x + point[0]
            y = y + point[1]
            z = z + point[2]+0.0125

        print("Object location hypothesis:")
        print(pose.pose.position)
        self.perc_centroids = np.append(self.perc_centroids, np.array([pose.pose.position.x, pose.pose.position.y, pose.pose.position.z]))
        print("Object location observed:")
        print(x/4.0)
        print(y/4.0)
        print(z/4.0)
        self.touch_centroids = np.append(self.touch_centroids, np.array([x,y,z]))
        
        # Let's stop centering when we've seen enough zero errors so
        # far (so that a single spike doesn't stop the process)
        so_far = 0
        ''' while True:
            err = self.error
            rospy.loginfo("err is {}".format(err))
            # This needs to be 0, given the deadband
            if abs(err) == 0:
                so_far += 1
                if so_far == 4:
                    # Not sure if necessary, but command 0 velocity seems
                    # a good idea
                    self.limb.set_joint_velocities(
                        self.compute_joint_velocities([0, 0, 0, 0, 0, 0])
                    )
                    break
            # Controlling the arm: At 1.0 / 30000.0 * err, it's unstable
            # 40000^{-1} -> unstable
            # 50000^{-1} -> unstable
            # 80000^{-1} -> almost stable
            # 10^{-5} -> decent given the system, etc
            y_v = np.clip(1.0 / 100000.0 * err, -0.08, 0.08)
            rospy.loginfo("y_v = {}".format(y_v))
            v_cartesian = self._vector_to((0, y_v, 0))
            v_joint = self.compute_joint_velocities(v_cartesian)
            self.limb.set_joint_velocities(v_joint)
        rospy.loginfo('Centered')'''
        # self.move_ik(pose)

        #rospy.sleep(0.5)
        #self.gripper.close()
        #rospy.sleep(0.5)
        #self.move_ik(pregrasp_pose)

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

    n.robot.scan_cup()
    


