#!/usr/bin/env python

import rospy
import json

import freenect
import cv2
import numpy as np

from igvc_msgs.msg import kinect_rgb
from igvc_msgs.msg import kinect_depth

def get_rgb(timer_event):
    global kinect_rgb_pub
    
    array,_ = freenect.sync_get_video()
    array = cv2.cvtColor(array, cv2.COLOR_RGB2BGR)

    rgb_msg = kinect_rgb()
    rgb_msg.rgb_image = array

    kinect_rgb_pub.publish(rgb_msg)


def get_depth(timer_event):
    global kinect_depth_pub

    array,_ = freenect.sync_get_depth()
    array = array.astype(np.uint8)
    

    depth_msg = kinect_depth()
    depth_msg.depth_image = array

    kinect_depth_pub.publish(rgb_msg)
    


def init_kinect_node():
    rospy.init_node("kinect_node", anonymous = True)

    kinect_rgb_pub = rospy.Publisher('/igvc/kinect_rbg/image_raw', kinect_rgb, queue_size = 10)
    kinect_depth_pub = rospy.Publisher('/igvc/kinect_depth/image_raw', kinect_depth, queue_size = 10)

    kinect_video_timer = rospy.Timer(rospy.Duration(secs = 0.005), get_rgb)
    kinect_depth_timer = rospy.Timer(rospy.Duration(secs = 0.005), get_depth)

    rospy.spin()


if __name__ == '__main__':
    try:
        init_kinect_node()
    except rospy.ROSInterruptException:
        pass