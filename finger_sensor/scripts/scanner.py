import rospy
import baxter_interface
import numpy as np
import serial
import re

rospy.init_node('object_scanner')
limb = baxter_interface.Limb('left')

angles = limb.joint_angles()

# Initialize the robot to scanning position
angles['left_s0']=-0.86133
angles['left_s1']=-0.4487985051
angles['left_e0']=0.04486
angles['left_e1']=1.12785
angles['left_w0']=-0.09127
angles['left_w1']=0.8444
angles['left_w2']=-3.1414
limb.set_joint_position_speed(0.5)
limb.move_to_joint_positions(angles)

# Initialize serial port
ser = serial.Serial('/dev/ttyACM1',115200)
print(ser.name)

try:
	for _move in np.linspace(-3.1414,3.1414, 36):

		s = ''
		angles['left_w2']=_move
		limb.set_joint_position_speed(1)
		limb.move_to_joint_positions(angles,timeout=4.0)
		angles = limb.joint_angles()
		#print(angles['left_w2'])
		s=repr(angles['left_w2'])
		ser.write(b'm')
		# read sensor strip
		line = ser.readline()
		line = re.sub(' +',',',line)
		line = re.sub('\r\n','',line)

		pre = line.split(",")
		for _i in range(1,17,1):
			#print(pre[_i])
			s=s+' '+pre[_i]
		print s
except KeyboardInterrupt:
	ser.close()
except Exception:
	ser.close()
