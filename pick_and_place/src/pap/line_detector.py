#! /usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np

class LineDetector():
    def __init__(self):
        self.rgb_image_sub = rospy.Subscriber('/camera/rgb/image_raw',Image,self.line_detector)
        self.line_image_pub = rospy.Publisher('/camera/rgb/lines',Image,queue_size=1)
        self.bridge = CvBridge()

    def line_detector(self,msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        gray = cv2.cvtColor(cv_image,cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray,50,150)

        lines = cv2.HoughLines(edges,1,np.pi/180,200)
        for rho,theta in lines[0]:
            a = np.cos(theta)
            b = np.sin(theta)
            x0 = a*rho
            y0 = b*rho
            x1 = int(x0 + 1000*(-b))
            y1 = int(y0 + 1000*(a))
            x2 = int(x0 - 1000*(-b))
            y2 = int(y0 - 1000*(a))

            cv2.line(cv_image,(x1,y1),(x2,y2),(0,0,255),2)
        ros_image = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
        self.line_image_pub.publish(ros_image)


if __name__ == '__main__':
    rospy.init_node("line_detector")
    rate = rospy.Rate(100)
    ld = LineDetector()
    rospy.spin()
