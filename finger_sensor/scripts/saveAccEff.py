import rospy
from sensor_msgs.msg import Imu
from sensor_msgs.msg import JointState


class startPrinting(object):

    def __init__(self):
        self.subscribe()
        #self.acc_x = None

    def subscribe(self):
        self.subscribeAcc = rospy.Subscriber("/robot/accelerometer/left_accelerometer/state",Imu, self.callback)
        self.subscribeEff = rospy.Subscriber("/robot/joint_states", JointState, self.callback2)
        rospy.spin()
        
    def callback(self, data):
        self.acc_x = data.linear_acceleration.x
        print acc_x

    def callback2(self, data):
        if len(data.effort) == 17:
            self.eff = data.effort[8]
            print self.eff

def main():
    rospy.init_node('AccEff_listener')
    a = startPrinting()
    #print a.acc_x
    rospy.spin()

if __name__ == "__main__":
    main()
