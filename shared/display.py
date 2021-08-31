#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry

import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


class OdometryPlot:
    def __init__(self):
        self.fig = plt.figure(figsize=(4,4))
        self.ax = self.fig.add_subplot(111, projection='3d')

    
    def draw(self, X, Y, Z):
        self.ax.scatter(X,Y,Z)
        plt.draw()
    

    def callback(self, data):
        pos = data.pose.pose.position
        self.draw(pos.x, pos.y, pos.z)

    
    def start(self):
        rospy.init_node('listener', anonymous=True)
        rospy.Subscriber('odometry/filtered', Odometry, self.callback)
        plt.show()
        rospy.spin()



if __name__ == '__main__':
    disp = OdometryPlot()
    disp.start()
