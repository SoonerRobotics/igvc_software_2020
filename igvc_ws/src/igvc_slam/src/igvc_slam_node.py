#!/usr/bin/env python

import copy
import numpy as np
import rospy

from nav_msgs.msg import OccupancyGrid, MapMetaData

# Publishers
config_pub = rospy.Publisher("/igvc_slam/config_space", OccupancyGrid, queue_size=1)

# Configuration space map
metadata = MapMetaData()
lidar_config_data = [0] * (200 * 200)
lanes_camera_config_data = [0] * (200 * 200)

# Initializiation
lidar_init = False


def lidar_callback(data):
    # use the global vars
    global lidar_init
    global lidar_config_data
    global metadata

    lidar_config_data = [0] * (200 * 200)

    # HACK: eventually set this to be based on the map size and stuff
    metadata = MapMetaData(map_load_time = data.info.map_load_time, resolution=data.info.resolution,
                            width = data.info.width, height = data.info.height, origin = data.info.origin)
    lidar_init = True

    for x in range(200):
        for y in range(200):
            if data.data[x + y * 200] > 0:
                for x_i in range(-5,6):
                    for y_i in range(-5,6):
                        dist = (x_i)**2 + (y_i)**2
                        new_x = 199 - (x + x_i)
                        new_y = 199 - (y + y_i)
                        if 0 <= new_x < 200 and 0 <= new_y < 200 and dist <= 25:
                            lidar_config_data[(x + x_i) + 200 * (y + y_i)] = 100





def lanes_camera_callback():
    pass


# TODO: currently this whole thing is set to use the most recent map frames from each perception unit (which is fine for now). In the future the sizing will change as the map grows.
def config_space_callback(event):
    # Use the global data
    global lidar_init
    global lidar_config_data
    global metadata

    if lidar_init is True:
        # TODO: Make this add the lidar config space to the camera config space
        config_space = lidar_config_data

        # Publish the configuration space
        config_msg = OccupancyGrid(info=metadata, data=config_space)
        config_pub.publish(config_msg)



def igvc_slam_node():
    # Set up node
    rospy.init_node("igvc_slam_node")

    # Subscribe to necessary topics
    map_sub = rospy.Subscriber("/igvc_vision/map", OccupancyGrid, lidar_callback, queue_size=10)

    # Make a timer to publish configuration spaces periodically
    timer = rospy.Timer(rospy.Duration(secs=1), config_space_callback, oneshot=False)

    # Wait for topic updates
    rospy.spin()


# Main setup
if __name__ == '__main__':
    try:
        igvc_slam_node()
    except rospy.ROSInterruptException:
        pass