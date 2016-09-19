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
        self.counter = 50
        self.i = 0
        self.data = []
        self.dataavg = np.zeros([7]) #change the dimension to inculde the other 7 sensor
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
        #removing the fingertip sensor values from the array
        #data = np.concatenate((data[:,0:7], np.multiplydata[:,8:15],y)), axis=0)
        if (self.i>0):
            data = data[0:7] #change the dimension to plot other sensor
            #calculating the average
            self.i=self.i+1
            plt.plot(data,'b--')
            plt.ion()
            plt.draw()
            plt.clf()
            self.dataavg = np.add(self.dataavg ,data)
            #print (self.dataavg)
            if (self.i==25):
                self.dataavg=np.divide(self.dataavg,25)
                #print ('this is the avegrage value')
                #print (self.dataavg)
        else:
            data = np.subtract(data[0:7],self.dataavg) #look for the dimension
            #subtracting the avegare of values overtime
            #print ('use the calculated avg and subtract from each new value')
            self.i=self.i+1
            plt.plot(data,'b--')
            plt.ion()
            #plt.show()
            plt.draw()
            plt.clf()

    def getsensorreadings(self):
        #data = subscribe_sensor()
        #for i in range(100,50,-5):
        self.pressed = False
        self.rate = rospy.Rate(20)
        while not self.pressed and not rospy.is_shutdown():
            self.rate.sleep()
        return self.values

def main():
    while not rospy.is_shutdown():
        #try:
            data = eshp.getsensorreadings()
            eshp.data = np.array(data)
            #print (eshp.data)
            eshp.plot(eshp.data)
        #except KeyboardInterrupt:
            #eshp.plot.plt.close()
            #print ('keyboard interrupt >>>')
            #break

    #break
    eshp.sensor_subscriber.unregister()

if __name__=='__main__':
    rospy.init_node('shape_estimation')
    eshp = estimate_shape()
    #eshp.gripper.calibrate()
    eshp.gripper.set_velocity(30)
    main()
    rospy.spin()
