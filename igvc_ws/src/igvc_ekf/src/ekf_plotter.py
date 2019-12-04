#!/usr/bin/env python
import numpy as np
from math import degrees
from matplotlib import pyplot as plt
import rospy
from igvc_msgs.msg import ekf_state



def plot_x(msg):
    global counter
    if counter % 10 == 0:
        stamp = msg.header.stamp
        time = stamp.secs + stamp.nsecs * 1e-9
        plt.plot(msg.x_k[3], msg.x_k[4], '*')
        plt.axis("equal")
        plt.draw()
        plt.pause(0.00000000001)

        csv_file = open("export.csv", "a")
        csv_file.write(str(degrees(msg.x_k[0])) + "," + str(degrees(msg.x_k[1])) + "\n")
        csv_file.close()

    counter += 1

if __name__ == '__main__':
    counter = 0

    rospy.init_node("plotter")

    rospy.Subscriber("/igvc_ekf/filter_output", ekf_state, plot_x)

    csv_file = open("export.csv", "w")
    csv_file.write("lat, lon\n")
    csv_file.close()

    plt.ion()
    plt.show()
    rospy.spin()