#! /usr/bin/env python
import rospy
from pap.manager import PickAndPlaceNode


if __name__ == '__main__':
    n = PickAndPlaceNode('left')
    rospy.spin()
