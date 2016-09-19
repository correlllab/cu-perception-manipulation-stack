from __future__ import division

import os

import rospy
from std_msgs.msg import Int32MultiArray, Int32
from sensor_msgs.msg import Imu, JointState
import baxter_interface

import numpy as np
from scipy import signal
import matplotlib.pyplot as plt


class FilterValues(object):
    def __init__(self):
        self.gripper = baxter_interface.Gripper('left')
        self.values = []
        self.acc_x = []
        self.acc_y = []
        self.acc_z = []
        self.eff = []
        self.gripperAperture = []
        self.time1 = []
        self.time2 = []
        self.time3 = []

    def start_recording(self):
        self.sub1 = rospy.Subscriber(
            "/robot/accelerometer/left_accelerometer/state",
            Imu, self.callback)  # FAII ~100Hz
        self.sub2 = rospy.Subscriber("/robot/joint_states",
                                     JointState, self.callback2)  # eff ~138Hz
        self.sub3 = rospy.Subscriber('/sensor_values', Int32MultiArray,
                                     self.callback3, queue_size=1)  # FAI ~18Hz
        self.sub4 = rospy.Subscriber("/gripperAperture_values",
                                     Int32, self.callback4)

    def callback(self, data):
        self.time1.append(data.header.stamp.secs)
        self.acc_x.append(data.linear_acceleration.x)
        self.acc_y.append(data.linear_acceleration.y)
        self.acc_z.append(data.linear_acceleration.z)
        # rospy.loginfo("%s X: %s", data.header.stamp.nsecs, data.linear_acceleration.x)
        # rospy.loginfo("%s Y: %s", data.header.stamp.nsecs, data.linear_acceleration.y)
        # rospy.loginfo("%s Z: %s", data.header.stamp.nsecs, data.linear_acceleration.z)

    def callback2(self, data):
        if len(data.effort) == 17:
            self.time2.append(data.header.stamp.secs)
            self.eff.append(data.effort[8])
        # rospy.loginfo("%s E: %s", data.header.stamp.nsecs, data.effort[8])

    def callback3(self, msg):
        now = rospy.get_rostime()
        self.time3.append(now.secs)
        self.values.append(msg.data)

    def callback4(self, msg):
        self.gripperAperture.append(msg.data)

    def stop_recording(self):
        self.sub1.unregister()
        self.sub2.unregister()
        self.sub3.unregister()
        self.sub4.unregister()

    def filter(self):
        path = os.getcwd()+'/trialGraspEventDetection_dataFiles'
        self.Fgr = np.sum(self.values[:, 9:15], axis=1)  # SAI
        self.Fgl = np.sum(self.values[:, 0:7], axis=1)  # SAI

        # can use this to plot in matlab graspeventdetection_plot.m
        np.savetxt(path+'/SAI_Fgr.txt', self.Fgr)
        # can use this to plot in matlab
        np.savetxt(path+'/SAI_Fgl.txt', self.Fgl)

        # 0.55*pi rad/samples
        b1, a1 = signal.butter(1, 0.55, 'high', analog=False)
        self.f_acc_x = signal.lfilter(b1, a1, self.acc_x, axis=-1, zi=None)
        self.f_acc_y = signal.lfilter(b1, a1, self.acc_y, axis=-1, zi=None)
        self.f_acc_z = signal.lfilter(b1, a1, self.acc_z, axis=-1, zi=None)
        # self.f_eff = signal.lfilter(b1, a1, self.eff, axis=-1, zi=None)
        # type(eff)
        self.FAII = np.sqrt(np.square(self.f_acc_x) +
                            np.square(self.f_acc_y) +
                            np.square(self.f_acc_z))
        # can use this to plot in matlab
        np.savetxt(path+'/FAII.txt', self.FAII)

        # subtract base values from the values array
        self.values1 = self.values - self.values.min(axis=0)
        # pass the filter for each sensor
        self.fvalues1 = np.zeros(self.values1.shape)
        # 0.48*pi rad/samples
        b, a = signal.butter(1, 0.48, 'high', analog=False)
        for i in range(16):
            self.fvalues1[:, i] = signal.lfilter(b, a, self.values1[:, i],
                                                 axis=-1, zi=None)
        self.FAI = np.sum(self.fvalues1, axis=1)
        # can use this to plot in matlab
        np.savetxt(path+'/FAI.txt', self.FAI)

    def plot(self):
        plt.figure(1)
        plt.subplot(411)
        plt.ylabel('GripperAperture')
        # print (self.gripperAperture.shape[0])
        # getting the time scale correct.
        xgripper = np.linspace(0, self.gripperAperture.shape[0] / 20,
                               self.gripperAperture.shape[0])
        plt.plot(xgripper, self.gripperAperture, 'c')

        # plt.subplot(512)
        # plt.ylabel('error')
        # xFAII = np.linspace(0, self.centeringerr.shape[0]/18.78, self.centeringerr.shape[0])
        # plt.plot(xFAII, self.centeringerr)

        plt.subplot(412)
        plt.ylabel('SA-I channel')
        xFg = np.linspace(0, self.Fgl.shape[0]/18.78, self.Fgl.shape[0])
        plt.plot(xFg, self.Fgl, 'y', xFg, self.Fgr, 'k')  # SAI

        plt.subplot(413)
        plt.ylabel('FA-I channel')
        xFAI = np.linspace(0, self.FAI.shape[0]/18.78, self.FAI.shape[0])
        plt.plot(xFAI, self.FAI, 'm')

        plt.subplot(414)
        plt.ylabel('FA-II channel')
        xFAII = np.linspace(0, self.FAII.shape[0]/100, self.FAII.shape[0])
        plt.plot(xFAII, self.FAII)

        # plt.subplot(414)
        # plt.ylabel('aX')
        # xFAII = np.linspace(0, self.f_acc_x.shape[0]/100, self.f_acc_x.shape[0])
        # plt.plot(xFAII, self.f_acc_x)

        # plt.subplot(514)
        # plt.ylabel('aY')
        # xFAII = np.linspace(0, self.f_acc_y.shape[0]/100, self.f_acc_y.shape[0])
        # plt.plot(xFAII, self.f_acc_y)

        # plt.subplot(515)
        # plt.ylabel('aZ')
        # xFAII = np.linspace(0, self.f_acc_z.shape[0]/100, self.f_acc_z.shape[0])
        # plt.plot(xFAII, self.f_acc_z)

        # plt.subplot(515)
        # plt.ylabel('EndEffector_Effort')
        # plt.plot(self.eff,'r')
        plt.show()

    def load_data(self):
        # load recorded data
        self.acc_x = np.loadtxt('acc_x.txt')
        # print type(self.acc_x)
        self.acc_y = np.loadtxt('acc_y.txt')
        self.acc_z = np.loadtxt('acc_z.txt')
        self.values = np.loadtxt('values.txt')
        self.gripperAperture = np.loadtxt('gripperAperture.txt')
        self.centeringerr = np.loadtxt('centeringerr.txt')

    def convertandsave(self):
        # convert to numpy array and save
        path = os.getcwd()+'/trialGraspEventDetection_dataFiles'
        self.gripperAperture = np.array(self.gripperAperture)
        np.savetxt(path+'/gripperAperture.txt', self.gripperAperture)

        self.eff = np.array(self.eff)

        self.acc_x = np.array(self.acc_x)
        np.savetxt(path+'/acc_x.txt', self.acc_x)

        self.acc_y = np.array(self.acc_y)
        np.savetxt(path+'/acc_y.txt', self.acc_y)

        self.acc_z = np.array(self.acc_z)
        np.savetxt(path+'/acc_z.txt', self.acc_z)

        self.values = np.array(self.values)
        np.savetxt(path+'/values.txt', self.values)


if __name__ == "__main__":
    rospy.init_node('AccEff_listener')
    f = FilterValues()
    # f.start_recording()
    # rospy.sleep(3)
    # f.stop_recording()
    # print f.values
    # f.convertandsave()
    f.load_data()
    f.filter()
    f.plot()
