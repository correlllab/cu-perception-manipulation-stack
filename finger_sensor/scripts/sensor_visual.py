#!/usr/bin/env python
"""
Run
$ export MPLBACKEND="module://gr.matplotlib.backend_gr"
before running the visualization to use the gr backend
http://gr-framework.org/tutorials/matplotlib.html
It has built in double buffering, and works great. It's
supposed to be fast too, but it maybe feels a tiny bit
more sluggish, maybe because of the lack of flickering.
"""
import rospy
from std_msgs.msg import Int32MultiArray

import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl


def Int32MultiArray2np(msg):
    arr = np.array(msg.data)
    return arr

    # Not working:
    # Ignoring data_offset
    # shape = [i.size for i in msg.layout.dim]
    # strides = [i.stride for i in msg.layout.dim]
    # print(shape, strides, arr)
    # return np.lib.stride_tricks.as_strided(arr, shape, strides)


class FingerSensorVisualizer(object):
    def __init__(self, topic='/sensor_values'):
        self.nh = rospy.init_node('FingerSensorVisualizer', anonymous=True)
        cm = mpl.cm.get_cmap('YlOrRd')
        self.fig, self.ax = plt.subplots()
        self.im = self.ax.imshow(np.log2(2**12 * np.ones((2, 8))),
                                 cmap=cm,
                                 interpolation='none',
                                 vmin=np.log2(2**12),
                                 vmax=np.log2(2**16),
                                 # Doesn't seem to make a difference (?)
                                 animated=True)
        colorbar = self.fig.colorbar(self.im, orientation='horizontal')
        colorbar.set_ticks(np.arange(12, 17))
        # LaTeX powers
        colorbar.set_ticklabels([r'$2^{{{}}}$'.format(i) for i in range(12, 17)])
        # Big numbers
        colorbar.set_ticklabels([str(int(i)) for i in np.logspace(12, 16, 5, base=2)])

        self.ax.set_xticks(range(9))
        self.ax.set_xticklabels(list(range(1, 8)) + ['Tip'])
        self.ax.set_yticks([0, 1])
        self.ax.set_yticklabels(['Left', 'Right'])
        self.ax.set_xlim(-0.5, 7.5)
        self.fig.show()
        self.fig.canvas.draw()
        # Subscribe last, otherwise the callback might be called
        # before setup is finished...
        self.sub = rospy.Subscriber(topic,
                                    Int32MultiArray,
                                    self.callback,
                                    queue_size=1)

    def callback(self, msg):
        nparr = Int32MultiArray2np(msg)
        if nparr.shape != (16,):
            raise ValueError("Need 16 sensor values!")

        data = nparr.reshape(2, 8)
        self.im.set_data(np.log2(data))
        self.ax.set_title(str(data))
        self.fig.canvas.draw()


if __name__ == '__main__':
    vis = FingerSensorVisualizer()
    plt.show()
    rospy.spin()
