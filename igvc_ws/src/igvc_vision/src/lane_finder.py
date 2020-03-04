#!/usr/bin/env python

## Lane finder reads in the image topic and pushes out to the lane_deviation topic the current deviation from lane center.

import rospy
import cv2
import math
import numpy as np
from std_msgs.msg import Float32
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from LaneDetection.LineDetection import process_frame

bridge = CvBridge()
image_pub = rospy.Publisher("/igvc/road_vision/raw", Image, queue_size=1)

frame_num = 0
def camera_callback(data):
    global frame_num
    frame_num += 1
    if frame_num % 5 == 0:
        np_arr = np.fromstring(data.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        cv_image = process_frame(cv_image)

        image_message = bridge.cv2_to_imgmsg(cv_image, encoding="passthrough")
        image_pub.publish(image_message)

def lane_finder():
    rospy.init_node('lane_finder', anonymous=True)

    # pub = rospy.Publisher('/igvc/lane_deviation', Float32, queue_size=1)
    rospy.Subscriber("/igvc/camera/compressed", CompressedImage, camera_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        lane_finder()
    except rospy.ROSInterruptException:
        pass