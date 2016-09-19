from __future__ import division, print_function
from cStringIO import StringIO
import sys
from baxter_pykdl import baxter_kinematics
import numpy as np
import rospy
import threading
import baxter_interface
from std_msgs.msg import Int32MultiArray, Float64
from sensor import sensor_node
import matplotlib.pyplot as plt
from numpy import matlib


class estimate_shape(object):
    def __init__(self):
        #self.i = 0
        self.data = []
        self.valuesLock = threading.Lock()
        self.newLock    = threading.Lock()
        self.newData = False
        self.limb = baxter_interface.Limb('left')
        self.gripper = baxter_interface.Gripper('left')
        self.pressed = False
        self.sensor_subscriber = rospy.Subscriber('/sensor_values', Int32MultiArray, self.control, queue_size=1)

    def control(self,msg):
        self.values = msg.data
        self.pressed = True
        #print (self.values)

    def unsubscribe(self):
        self.sensor_subscriber.unregister()

    def plot(self,data):

        data = data[:,0:7]

        #subtract the base values
        #data = data - np.matlib.repmat(data[0,:],data.shape[0],1)
        ##data = data - np.matlib.repmat(data.max(axis=1),data.shape[0],1)
        #for inversion of another sensorvalues
        y = np.array([1])
        y = np.matlib.repmat(y,data.shape[0],7)
        #removing the fingertip sensor values from the array
        #data = np.concatenate((data[:,0:7], np.multiply(data[:,8:15],y)), axis=0)
        #plotting the readings taken at different distances
        for i in range(data.shape[0]):
            plt.plot(data[i],'--')
        plt.show()

    def getsensorreadings(self):
        #data = subscribe_sensor()
        #for i in range(100,50,-5):
        self.pressed = False
        self.rate = rospy.Rate(20)
        while not self.pressed and not rospy.is_shutdown():
            self.rate.sleep()
        return self.values

if __name__=='__main__':
    rospy.init_node('shape_estimation')
    eshp = estimate_shape()
    eshp.gripper.calibrate()
    eshp.gripper.set_velocity(30)
    
    #discrete number of plots
    for i in range(100,50,-5):
        data = eshp.getsensorreadings()
        eshp.data.append(data)
        eshp.gripper.command_position(i, block=True)
    #print (eshp.data)
    eshp.data = np.array(eshp.data)
    #print (eshp.data.shape)
    eshp.plot(eshp.data)

    #realtime plotting
