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

class estimate_shape(object):

    def __init__(self):
        #self.i = 0
        self.data = []
        self.valuesLock = threading.Lock()
        self.newLock    = threading.Lock()
        self.newData = False
        self.limb = baxter_interface.Limb('left')
        self.gripper = baxter_interface.Gripper('left')
        self.subscribe()

    def subscribe(self):
        self.sensor_subscriber = rospy.Subscriber('/sensor_values',
                                                    Int32MultiArray,
                                                    self.control, queue_size=1)
        #rospy.spin()

    def unsubscribe(self):
        self.sensor_subscriber.unregister()

    def control(self,msg):
        with self.valuesLock:
            self.values = msg.data
        with self.newLock:
            self.newData = True
        #print (self.values)
        #print (hasattr('estimate_shape','self.values'))

    def plot(self,data):
        #removing the fingertip sensor values from the array
        #data = data[:,0:7]
        data = np.concatenate((data[:,0:7], data[:,8:15]), axis=0)
        #plotting the readings taken at different distances
        for i in range(data.shape[0]):
            #y = np.array([i])
            #y = np.matlib.repmat(y,1,16)
            plt.plot(data[i],'--')
        plt.show()

def main():
    while not hasattr(eshp, 'values'):
        print("Waiting for values")
        pass
    #eshp.data.append(eshp.values)
    for i in range(100,30,-5):
        eshp.gripper.command_position(i, block=True)
        while(True):
            with eshp.newLock:
                if(eshp.newData):
                    break
                else:
                    #pass
                    print("Waiting for new data.")
        with eshp.valuesLock:
            eshp.data.append(eshp.values)
        with eshp.newLock:
            eshp.newData = False
    eshp.data = np.array(eshp.data)
    #print (eshp.data)
    eshp.gripper.command_position(100, block=True)
    eshp.plot(eshp.data)
    eshp.unsubscribe()


if __name__=='__main__':
    rospy.init_node('shape_estimation')
    eshp = estimate_shape()
    eshp.gripper.calibrate()
    eshp.gripper.set_velocity(30)
    eshp.subscribe()
    main()
    #rospy.spin()
