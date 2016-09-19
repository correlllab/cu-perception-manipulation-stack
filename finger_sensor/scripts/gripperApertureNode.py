import baxter_interface
from std_msgs.msg import Int32
import rospy

def gripperApertureNode():
    gripper = baxter_interface.Gripper('left')
    rospy.init_node('gripperApertureNode')
    pub = rospy.Publisher('/gripperAperture_values', Int32, queue_size=1)
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        ap = gripper.position()
        pub.publish(ap)
        rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('gripperApertureNode')
        g = gripperApertureNode()
    except rospy.ROSException:
        pass
