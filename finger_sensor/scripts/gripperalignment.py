#!/usr/bin/env python
from __future__ import division, print_function
from itertools import chain, repeat
from baxter_pykdl import baxter_kinematics
from baxter_interface import CHECK_VERSION
import rospy
import baxter_interface
import baxter_external_devices
from moveit_commander import conversions
from baxter_core_msgs.srv import SolvePositionIK, SolvePositionIKRequest
from baxter_core_msgs.msg import DigitalIOState
import tf
import serial
import re
import numpy as np
import time


# Initialize serial port
ser = serial.Serial('/dev/ttyACM1',115200)

class CuffOKButton(object):
 	def __init__(self, limb_name):
		self.limb_name = limb_name
		self.pressed = False
		node = '/robot/digital_io/%s_lower_button/state' % limb_name
		rospy.Subscriber(node, DigitalIOState, self.callback)

	def callback(self, data):
		self.pressed = data.state == 1


def limb_pose(limb_name):
	button = CuffOKButton(limb_name)
	rate = rospy.Rate(20)
	joint_pose = baxter_interface.Limb(limb_name).joint_angles()
	kinematics = baxter_kinematics(limb_name)
	endpoint_pose = kinematics.forward_position_kinematics(joint_pose)
	return endpoint_pose

class Baxter(object):
	"""Add higher level operations to Baxter."""
	def __init__(self, limb_name, gripper=None):
		self.limb_name = limb_name
		self.limb = baxter_interface.Limb(limb_name)
		if gripper is None:
			self.gripper = baxter_interface.Gripper(limb_name)
		else:
			self.gripper = gripper(limb_name)
		baxter_interface.RobotEnable().enable()


	def move(self, direction, distance):
		pose = limb_pose(self.limb_name)
		pregrasp_pose = self.translate(pose, direction, distance)
		self.move_ik(pregrasp_pose)
	
	def move_ik(self, pose):
		stamped_pose = self.to_stamped_pose(pose)
		joint_pose = self.ik_quaternion(stamped_pose)
		if joint_pose != None:
			return self.limb.move_to_joint_positions(joint_pose)
		else:
			return None

	@staticmethod
	def to_stamped_pose(pose):
		stamped_pose = conversions.list_to_pose_stamped(pose, "base")
		return stamped_pose

	@staticmethod
	def translate(pose, direction, distance):
		scaled_direction = [distance * di for di in direction]
		translated_pose = [pi + di for pi, di in
						   zip(pose,
							   chain(scaled_direction,
									 repeat(0, len(pose) - 3)))]
		return translated_pose
	def ik_quaternion(self, quaternion_pose):
		node = ("ExternalTools/{}/PositionKinematicsNode/"
				"IKService".format(self.limb_name))
		ik_service = rospy.ServiceProxy(node, SolvePositionIK)
		ik_request = SolvePositionIKRequest()
		ik_request.pose_stamp.append(quaternion_pose)
		try:
			rospy.loginfo('ik: waiting for IK service...')
			rospy.wait_for_service(node, 5.0)
			ik_response = ik_service(ik_request)
		except (rospy.ServiceException, rospy.ROSException) as err:
			rospy.logerr("ik_move: service request failed: %r" % err)
		else:
			if ik_response.isValid[0]:
				rospy.loginfo("ik_move: valid joint configuration found")
				limb_joints = dict(zip(ik_response.joints[0].name,
									   ik_response.joints[0].position))
				return limb_joints
			else:
				rospy.logerr('ik_move: no valid configuration found')
				return None

	@staticmethod
	def _to_quaternion_pose(pose):
		if len(pose) == 6:
			q = tf.transformations.quaternion_from_euler(*pose[3:]).tolist()
			return list(pose[:3]) + q
		elif len(pose) == 7:
			return pose
		else:
			raise TypeError('Pose needs to be xyzrpy or xyzqxqyqzq')


	def map_keyboardL(self):
		rs = baxter_interface.RobotEnable(CHECK_VERSION)
		init_state = rs.state().enabled
		limb_0 = baxter_interface.Gripper(self.limb_name, CHECK_VERSION)	   
		#baxter_interface.Limb(limb_name).set_joint_position_speed(0.7)	  
		self.move((0,1,0),0.01)

	def map_keyboardR(self):
		rs = baxter_interface.RobotEnable(CHECK_VERSION)
		init_state = rs.state().enabled
		limb_0 = baxter_interface.Gripper(self.limb_name, CHECK_VERSION)	   
		#baxter_interface.Limb(limb_name).set_joint_position_speed(0.7)	  
		self.move((0,-1,0),0.01)


def main(limb_name, reset):
	rospy.init_node("pick_and_place", anonymous=True)
	if reset or not rospy.has_param('~pick_and_place_poses'):
		#rospy.loginfo(
			#'Saving picking pose for %s limb' % limb_name)
		pick_pose = limb_pose(limb_name)
		rospy.sleep(1)
		place_pose = limb_pose(limb_name)
		rospy.set_param('~pick_and_place_poses',
						{'pick': pick_pose.tolist(),
						 'place': place_pose.tolist()})
		#rospy.loginfo('pick_pose is %s' % pick_pose)
		#rospy.loginfo('place_pose is %s' % place_pose)
	else:
		pick_pose = rospy.get_param('~pick_and_place_poses/pick')
		place_pose = rospy.get_param('~pick_and_place_poses/place')

	b = Baxter(limb_name)
	while True:
		ser.write(b'm')
		s=''
		line = ser.readline()
		line = re.sub(' +',',',line)
		line = re.sub('\r\n','',line)
		pre = line.split(",")
		err = [(int(pre[j+8])-int(pre[j])) for j in range(1,8)]
		sum1 = 0
		for i in range(0,7,1):
			sum1 = sum1 + err[i]
		avg = sum1/7
		print(avg)

		endeffvel = baxter_interface.Limb(limb_name).endpoint_velocity()
		print(endeffvel)

		jacobianinv = baxter_kinematics(limb_name).jacobian_pseudo_inverse()
		print(jacobianinv)
		
		if avg>50:
			b.map_keyboardR()
		elif avg<-50:
			b.map_keyboardL()
		else:
			break
   
	
if __name__ == "__main__":
	import argparse

	parser = argparse.ArgumentParser(
		description='Use baxter to pick and place objects')
	parser.add_argument(
		'-l', '--limb', choices=('left', 'right'), default='left',
		help='Choose which limb to use for picking and placing')
	parser.add_argument(
		'-r', '--reset', action='store_true', default=False) #

	args = parser.parse_args(rospy.myargv()[1:])

	main(args.limb, args.reset)
