#!/usr/bin/env python

# receive serial and send signals to motors

import rospy
import serial
from igvc_msgs.msg import serial_motion_msg

def call_back(data):
    # Print data
    rospy.loginfo("Left speed: %f, Right speed: %f, \n", data.left_speed, data.right_speed)

def receive_serial():
    rospy.init_node('serial_receiver_node', anonymous = True)
    rospy.Subscriber("/igvc/motion_serial", serial_motion_msg, call_back)
    rospy.spin()

if __name__ == '__main__':
    try:
        receive_serial()
    except rospy.ROSInterruptException:
        pass
