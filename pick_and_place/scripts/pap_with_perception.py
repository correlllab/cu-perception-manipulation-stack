#! /usr/bin/env python
import rospy
from pap.manager import PickAndPlaceNode
from perception.msg import identified_object
from pap.jaco import Jaco

if __name__ == '__main__':
    #n = PickAndPlaceNode('left')
    n = PickAndPlaceNode(Jaco)
    rospy.spin()
