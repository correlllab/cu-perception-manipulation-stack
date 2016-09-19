import rospy
import baxter_interface
from sensor import sensor_node
from std_msgs.msg import Int32MultiArray
import math

class baxter(object):
    def __init__(self, limb_name):
        self.limb_name = limb_name
        self.limb = baxter_interface.Limb(limb_name)
        self.gripper = baxter_interface.Gripper(limb_name)
        # robot_enable.py -e
        baxter_interface.RobotEnable().enable()
        # Calibrate gripper
        self.gripper.calibrate()
        self.L = []
        self.subscribe()
        #self.gripper.open()

    def subscribe(self):
        self.sensor_subscriber = rospy.Subscriber('/sensor_values',
                                                 Int32MultiArray,
                                                 self.control,
                                                 queue_size=1)


    def close_gripper(self):
        self.gripper.command_position(0, block=True)
        rospy.sleep(2)

    def disable(self):
        self.sensor_subscriber.unregister()

    def control(self, msg):
        self.values = msg.data

    def calculate_reflectivity(self, a, b):
        print a
        print b
        #c = [(b[i]-a[i]) for i in range(len(a))]
        #print c
        C0plusC1 = math.pow((2/((1/math.sqrt(b[0]))-(1/math.sqrt(a[0])))),2)
        print C0plusC1
        d = math.sqrt(C0plusC1/a[0])
        print ("distance in cm: " + str(d))

    def move_to_position(self):
        angles = self.limb.joint_angles()
        # Initialize the robot to scanning position
        #print angles
        angles['left_s0']=-0.86133
        angles['left_s1']=-0.4487985051
        angles['left_e0']=0.04486
        angles['left_e1']=1.12785
        angles['left_w0']=-0.09127
        angles['left_w1']=0.8444
        angles['left_w2']=-3.1414
        self.limb.set_joint_position_speed(0.5)
        self.limb.move_to_joint_positions(angles)

def main():
    rospy.init_node('phong_model')
    b = baxter('left')
    b.move_to_position()
    b.L.append(b.values)
    b.close_gripper()
    b.L.append(b.values)
    b.calculate_reflectivity(b.L[0], b.L[1])


if __name__ == '__main__':
    main()
