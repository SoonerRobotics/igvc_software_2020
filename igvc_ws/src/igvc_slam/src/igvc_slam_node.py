#!/usr/bin/env python

import copy
import numpy as np
import rospy
import math
import cv2
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image
from nav_msgs.msg import OccupancyGrid, MapMetaData

# Publishers
config_pub = rospy.Publisher("/igvc_slam/local_config_space", OccupancyGrid, queue_size=1)

# Configuration space map
metadata = MapMetaData()

last_lane_map = None
last_lidar = None

# Initializiation
lidar_init = False

# Image to cv2
bridge = CvBridge()

def lane_callback(data):
    global last_lane_map, print_once
    img = bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")

    resized = cv2.resize(img, (200, 130))
    last_lane_map = resized

def lidar_callback(data):
    # use the global vars
    global last_lidar, metadata

    # HACK: eventually set this to be based on the map size and stuff
    metadata = MapMetaData(map_load_time = data.info.map_load_time, resolution=data.info.resolution,
                            width = data.info.width, height = data.info.height, origin = data.info.origin)

    last_lidar = data.data

# TODO: currently this whole thing is set to use the most recent map frames from each perception unit (which is fine for now). In the future the sizing will change as the map grows.
def config_space_callback(event):
    if last_lane_map is None or last_lidar is None:
        return

    time_1 = rospy.get_time()

    result = [0] * (200 * 200)

    # Update the hidden layer before applying the new map to the current configuration space
    for x in range(200):
        for y in range(200):
            if last_lidar[x + y * 200] > 0 or (y < 100 and last_lane_map[y, x]) > 0:
                for x_i in range(-6,7):
                    for y_i in range(-6,7):
                        index = ((x + x_i)) + 200 * (y + y_i)
                        dist = (x_i)**2 + (y_i)**2

                        if 0 <= (x + x_i) < 200 and 0 <= (y + y_i) < 200 and dist <= 49 and result[index] < dist * (-100/49) + 100: 
                            result[index] = int(dist * (-100/64) + 100)
    
    time_2 = rospy.get_time()
    print("time to config_space_callback", (time_2 - time_1) * 1000)

    # Publish the configuration space
    config_msg = OccupancyGrid(info=metadata, data=result)
    config_pub.publish(config_msg)



def igvc_slam_node():
    # Set up node
    rospy.init_node("igvc_slam_node")

    # Subscribe to necessary topics
    map_sub = rospy.Subscriber("/igvc_vision/map", OccupancyGrid, lidar_callback, queue_size=1)
    lane_sub = rospy.Subscriber("/igvc_vision/road_vision/raw", Image, lane_callback, queue_size=1)

    # Make a timer to publish configuration spaces periodically
    timer = rospy.Timer(rospy.Duration(secs=0.1), config_space_callback, oneshot=False)

    # Wait for topic updates
    rospy.spin()


# Main setup
if __name__ == '__main__':
    try:
        igvc_slam_node()
    except rospy.ROSInterruptException:
        pass