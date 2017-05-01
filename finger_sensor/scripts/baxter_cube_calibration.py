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

start_num = 0
num_cubes = 15
cube_name = "/block_"
cubes = ["/block_5","/block_17","/block_0","/block_6","/block_15"]

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
        self.camera_centroids = np.zeros((num_cubes, 3))
        #self.camera_centroids = np.array([[ 0.676749,-0.116798,-0.100457],[ 0.564606,-0.034142,-0.10861 ],[ 0.645695,-0.291158,-0.093074]])
        self.touch_centroids = np.zeros((num_cubes, 3))
        #self.touch_centroids = np.array([[ 0.684658,-0.097485,-0.09421 ],[ 0.568634,-0.025073,-0.098264],[ 0.660665,-0.273198,-0.093639]])
        self.current_index = 0
        self.update_transform = rospy.Publisher("/update_camera_alignment",
                                                Pose,
                                                queue_size = 1)
        #self.zero_sensor()


    def capture_centroid(self, name):
        t = self.tl.getLatestCommonTime("/root", name)
        position, quaternion = self.tl.lookupTransform("/root",
                                                       name,
                                                         t)
        #print("Object location hypothesis:")
        #print(position)
        self.camera_centroids[self.current_index] = np.array(list(position))

    def update_translation(self):
        t = self.tl.getLatestCommonTime("/root", "/camera_link")
        position, quaternion = self.tl.lookupTransform("/root",
                                                       "/camera_link",
                                                       t)
        print("Cam Pos:")
        print(position)

        translation = self.touch_centroids[0] - self.camera_centroids[0]
        print("Translation: ")
        print(translation)
        position = position + translation
        print("New Cam Pos:")
        print(position)
        #print(self.touch_centroids)
        #print(self.camera_centroids)
        
        self.update_transform.publish( Pose(Point(*position), Quaternion(*quaternion)))

    def update_transformation(self, position, orientation):
        # number of points
        print("original: ")
        print position
        N = len(self.perc_centroids)
        print("N: " + str(N))
        P = (np.mat(self.camera_centroids)).T
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

        quat = tf.transformations.quaternion_from_matrix(M)
        print("quat")
        print quat

        return ret_value, quat


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
        #get ICP to find rotation and translation
        #multiply old position and orientation by rotation and translation
        #publish new pose to /update_camera_alignment


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

    def sensors_updated(self):
        if((time.time() - self.last_sensor_update) < 2):
            return True
        else:
            print("Finger sensor data outdated. Aborting action")
            return False

    #moves down until table
    def move_down(self, direction=(0,0,1), max_tip=3500):
        while True:
            #rospy.loginfo("Going down to pick (at {})".format(self.tip.max()))
            if(not self.sensors_updated()):
                return
            if self.tip.max() > max_tip:
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

    #performs horizontal scan until timeout or cube sensed. 
    #   if timeout:
    #       recursively performs scan in the y direction
    def x_horizontal_scan(self, pose, direction=(0,0,1), x_dir=0.005, sensor_index=13, max_inside = 2000, timeout=50):
        rospy.loginfo('Touching side of block')
        count = 0
        while True:
            if(not self.sensors_updated()):
                return
            #rospy.loginfo(count)
            #rospy.loginfo("Moving to touch (at {})".format(self.inside[6]))
            if self.inside[sensor_index] > max_inside:
                return True
            else:
                scaled_direction = (di / 100 for di in direction)
                #print("Scaled direction: ", scaled_direction)
                v_cartesian = self._vector_to(scaled_direction)
                v_cartesian[1] = x_dir
                #print("cartesian: ", v_cartesian)
                v_joint = self.compute_joint_velocities(v_cartesian)
                #print("joint    : ", v_joint)
                self.limb.set_joint_velocities(v_joint)
                count=count+1
                rospy.sleep(0.25)
            if(count > timeout):
                rospy.loginfo("Searching for block again")
                pose.pose.position.x = pose.pose.position.x + .01
                h = Header()
                h.stamp = self.tl.getLatestCommonTime("/root", self.object_frame)
                h.frame_id = "/root"
                pose.header = h
                self.pick(pose)
                return False

    def y_block_side_scan(self, pose, direction=(0,0,1), y_dir=0.005, sensor_index=13, max_inside = 1000, timeout=50):    
        while True:
            if(not self.sensors_updated()):
                return
            #rospy.loginfo("Moving to touch (at {})".format(self.inside[6]))
            if self.inside[sensor_index] < max_inside:
                return
            else:
                scaled_direction = (di / 100 for di in direction)
                #print("Scaled direction: ", scaled_direction)
                v_cartesian = self._vector_to(scaled_direction)
                v_cartesian[0] = y_dir
                #print("cartesian: ", v_cartesian)
                v_joint = self.compute_joint_velocities(v_cartesian)
                #print("joint    : ", v_joint)
                self.limb.set_joint_velocities(v_joint)
                rospy.sleep(0.25)
    
    def pick(self, pose, direction=(0, 0, 1), distance=0.1):
        cube_corners = list()
        pregrasp_pose = self.translate(pose, (-0.1, 0, 1), distance)

        self.move_ik(pregrasp_pose)
        rospy.sleep(0.5)
        # We want to block end effector opening so that the next
        # movement happens with the gripper fully opened.
        self.gripper.open()

        self.move_down(direction, max_tip=3500)
        
        rospy.sleep(0.5)
        
        if(not self.x_horizontal_scan(pose, direction)):
            return
        t = self.tl.getLatestCommonTime("/root", "/r_gripper_l_finger_tip")
        position, quaternion = self.tl.lookupTransform("/root",
                                                       "/r_gripper_l_finger_tip",
                                                         t)
        cube_corners.append(position)

       

        rospy.loginfo('Touching along the side of the block')
        
        self.y_block_side_scan(pose, direction)

        t = self.tl.getLatestCommonTime("/root", "/r_gripper_l_finger_tip")
        position, quaternion = self.tl.lookupTransform("/root",
                                                       "/r_gripper_l_finger_tip",
                                                         t)


        cube_corners.append(position)

        
        #below stuff touches other side of block
        rospy.loginfo('Touching other side of block')
        self.x_horizontal_scan(pose, direction, x_dir=-0.005, sensor_index=6, max_inside = 1500, timeout=5000)

        t = self.tl.getLatestCommonTime("/root", "/r_gripper_r_finger_tip")
        position, quaternion = self.tl.lookupTransform("/root",
                                                       "/r_gripper_r_finger_tip",
                                                         t)

        cube_corners.append(position)
        rospy.loginfo('Touching along the side of the block')
        self.y_block_side_scan(pose, direction, y_dir=-0.005, sensor_index=6)

        t = self.tl.getLatestCommonTime("/root", "/r_gripper_r_finger_tip")
        position, quaternion = self.tl.lookupTransform("/root",
                                                       "/r_gripper_r_finger_tip",
                                                         t)


        cube_corners.append(position)
        x = 0
        y = 0
        z = 0
        rospy.loginfo('Touched both sides of the block')

        cur_position = list(position)
        cur_position[2] = cur_position[2] + 0.1
        new_pose = Pose(Point(*cur_position),
                        Quaternion(*quaternion))
        stamped_pose = PoseStamped( Header(0, rospy.Time(0), n.robot.base), new_pose)
        self.move_ik(stamped_pose)

        for point in cube_corners:
            x = x + point[0]
            y = y + point[1]
            z = z + point[2]

        x = x/4.0
        y = y/4.0
        z = (z+0.0125*4)/4.0    
        
        #print("Object location observed:")

        self.touch_centroids[self.current_index] = np.array([x,y,z])
        self.capture_centroid(self.object_frame)
        self.current_index += 1
        

def publish_transform(name, pos):
    broadcast = tf.TransformBroadcaster()
    while(True):
        broadcast.sendTransform(tuple(pos), [0,0,0,1],
                                    rospy.Time.now(),
                                    name,
                                    "root")
        rospy.sleep(1)

if __name__ == '__main__':
    n = PickAndPlaceNode(FingerSensorBaxter, 'right')
    
    home = PoseStamped(
            Header(0, rospy.Time(0), n.robot.base),
            Pose(Point(0.60558, -0.24686, 0.093535),
                 Quaternion(0.99897, -0.034828, 0.027217, 0.010557)))
    n.robot.move_ik(home) 
    
    #thread.start_new_thread(publish_transform, ("block_0_touch", [.2,.2,.2]))
    raw_input("Press Enter to find cubes...")
    #get block 0 location
    f = open('centroid_storage.txt', 'w')
    cube = 0
    for i in range(start_num,start_num+num_cubes):
        n.robot.object_frame = cube_name + str(i)
        if(n._pickFrame(n.robot.object_frame)):
            
            f.write(n.robot.object_frame)
            f.write('\n')

            thread.start_new_thread(publish_transform, (cube_name + str(i) + "_touch", n.robot.touch_centroids[cube].tolist()))

            f.write('camera: \n')
            f.write(np.array2string(n.robot.camera_centroids[cube], precision=6, separator=','))
            f.write('\ntouch : \n')
            f.write(np.array2string(n.robot.touch_centroids[cube], precision=6, separator=','))
            f.write('\n\n\n')
            cube += 1

    f.close()
    
    #transform using just one cube. find the translation
    raw_input("Press Enter to perform translation...")
    '''
    n.robot.update_translation()

    raw_input("Press Enter for average translation...")
    #get block 1 location

    #find the translation and rotation around the worst axis
    raw_input("Press Enter rigid transformation...")

    n.robot.update_camera_transform()

    raw_input("Press Enter for full transformation...")

    raw_input("Press Enter to continue...")
    '''
