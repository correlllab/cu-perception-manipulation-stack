#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from std_msgs.msg import Header
import numpy as np
import struct
import ctypes

import tf
import tf2_ros
import geometry_msgs.msg

class TableRemover:
    def __init__(self):
        self.point_cloud_sub = rospy.Subscriber('/camera/depth_registered/points', PointCloud2, self.point_cloud_callback)
        self.br = tf2_ros.TransformBroadcaster()

        circle_rgb = [110, 70, 90]
        triangle_rgb = [75, 120, 60]
        square_rgb = [30, 90, 155]
        rectangle_rgb = [165, 175, 90]
        hexagon_rgb = [120, 50, 50]
        self.shapes_rgb = [circle_rgb, triangle_rgb, square_rgb, rectangle_rgb, hexagon_rgb]
        self.shape_names = ['circle', 'triangle', 'square', 'rectangle', 'hexagon']


    def point_cloud_callback(self, msg):
        cloud, rgb = self.get_cloud(msg)
        self.publish_shape_tf(cloud, rgb)

    def publish_shape_tf(self, cloud, rgb):
        for i in range(len(self.shape_names)):
            shape_rgb = self.shapes_rgb[i]
            shape_name = self.shape_names[i]

            tol = 200
            if shape_name == 'rectangle':
                tol = 2000
            idx = self.get_color_shape_idx(rgb, shape_rgb,tol=tol)
            shape_cloud = cloud[idx]

            x, y, z = self.get_shape_centroid(shape_cloud)
            self.broadcast_tf(x,y,z,frame_name=shape_name)

    def broadcast_tf(self, x, y, z, frame_name="unknown"):
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "camera_rgb_optical_frame"
        t.child_frame_id = frame_name
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z
        t.transform.rotation.x = 0
        t.transform.rotation.y = 0
        t.transform.rotation.z = 0
        t.transform.rotation.w = 1.0

        self.br.sendTransform(t)

    def get_shape_centroid(self, shape_cloud):
        return np.nanmean(shape_cloud, axis=0)

    def get_color_shape_idx(self, rgb, shape_rgb, tol=200):
        dist = np.sum((rgb - shape_rgb) ** 2, axis=1)
        return np.where(dist < tol)

    def get_cloud(self, pc):
        cloud = []
        rgb = []
        for point in point_cloud2.read_points(pc, skip_nans=False):
            cloud.append([point[0], point[1], point[2]])
            rgb_point = self.get_rgb(point[3])
            rgb.append(rgb_point)
        return np.array(cloud), np.array(rgb)

    def get_rgb(self, data):
        s = struct.pack('>f', data)
        i = struct.unpack('>l', s)[0]
        pack = ctypes.c_uint32(i).value

        r = int((pack & 0x00FF0000)>> 16)
        g = int((pack & 0x0000FF00)>> 8)
        b = int(pack & 0x000000FF)
        return [r, g, b]


if __name__=="__main__":
    rospy.init_node("rgb_cloud")
    tr = TableRemover()
    rospy.spin()
