#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
from igvc_msgs.msg import motors
import copy
import numpy as np
from mt_dstar_lite import mt_dstar_lite

motor_pub = rospy.Publisher("/igvc/motors_raw", motors, queue_size=10)
path_pub = rospy.Publisher("/igvc_nav/path_map", OccupancyGrid, queue_size=10)

# Moving Target D* Lite
map_init = False
planner = mt_dstar_lite()


def c_space_callback(c_space):
    global planner
    global map_init

    # Get the grid data
    grid_data = c_space.data

    # Find the best position
    best_pos = (0,0)
    best_pos_cost = 1000000

    # only look forward
    for row in range(100):
        for col in range(200):
            if grid_data[(row * 200) +  col] == 0:
                new_cost = abs(row-75) + abs(col-100)
                if new_cost < best_pos_cost:
                    best_pos_cost = new_cost
                    best_pos = (row, col)

    print "target:"
    print best_pos
    print ""

    # Are we somehow on a bad spot?
    if grid_data[(100 * 200) + 100] == 1:
        return

    # Reset the path
    path = None

    # MOVING TARGET D*LITE
    # If this is the first time receiving a map, initialize the path planner and plan the first path
    if True: #map_init is False:
        planner.initialize(200, 200, (100, 100), best_pos, grid_data)
        path = planner.plan()
    # Otherwise, replan the path
    else:
        # Request the planner replan the path
        path = planner.replan(config_space.tolist(), (100, 100), best_pos, grid_data)


    print "path:"
    print path

    if path is not None:
        path_space = [0] * 200 * 200
        itt = 100
        for path_pos in path:
            path_space[(200 * path_pos[0]) + path_pos[1]] = itt
            itt -= 1

        path_msg = copy.deepcopy(c_space)
        path_msg.data = path_space

        path_pub.publish(path_msg)


def mt_dstar_node():
    # Setup node
    rospy.init_node("mt_dstar_node")

    # Subscribe to necessary topics
    rospy.Subscriber("/igvc_slam/config_space", OccupancyGrid, c_space_callback, queue_size=1)

    # Wait for topic updates
    rospy.spin()



# Main setup
if __name__ == '__main__':
    try:
        mt_dstar_node()
    except rospy.ROSInterruptException:
        pass
