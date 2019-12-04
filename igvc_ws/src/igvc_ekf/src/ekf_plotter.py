#!/usr/bin/env python
import numpy as np
from math import degrees
from matplotlib import pyplot as plt

import rospy
from igvc_msgs.msg import ekf_state, ekf_convergence
from roslib.packages import get_pkg_dir

pkg_path = get_pkg_dir("igvc_ekf")

def plot_x(msg):
    global counter
    if counter % 10 == 0:
        plt.figure(1)
        stamp = msg.header.stamp
        time = stamp.secs + stamp.nsecs * 1e-9
        plt.plot(msg.x_k[3], msg.x_k[4], '*')
        plt.axis("equal")
        plt.draw()
        plt.pause(0.00000000001)

        csv_file = open(pkg_path + "/data/export.csv", "a")
        state_str = str(counter/10) + "," + str(degrees(msg.x_k[0])) + "," + str(degrees(msg.x_k[1])) + ","
        for i in range(2, 11):
            state_str += str(msg.x_k[i]) + ","
        csv_file.write(state_str + "\n")
        csv_file.close()

    counter += 1

def plot_conv(msg):
    global counter
    if counter % 10 == 0:
        # stamp = msg.header.stamp
        # plt.figure(2)
        # time = stamp.secs + stamp.nsecs * 1e-9
        # plt.plot(time, msg.data, '*')
        # plt.axis("equal")
        # plt.draw()
        # plt.pause(0.00000000001)

        conv_file = open(pkg_path + "/data/convergence.csv", "a")
        state_str = str(msg.data) + "\n"
        conv_file.write(state_str)
        conv_file.close()


if __name__ == '__main__':
    counter = 0

    rospy.init_node("plotter")

    rospy.Subscriber("/igvc_ekf/filter_output", ekf_state, plot_x)
    rospy.Subscriber("/igvc_ekf/ekf_convergence", ekf_convergence, plot_conv)

    csv_file = open(pkg_path + "/data/export.csv", "w")
    csv_file.write("id, lat, lon, theta, x, y, psi, vel, psi_dot, left_v, right_v, accel\n")
    csv_file.close()

    csv_file = open(pkg_path + "/data/convergence.csv", "w")
    csv_file.close()

    plt.ion()
    plt.show()
    rospy.spin()