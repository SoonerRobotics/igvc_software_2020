#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
from igvc_msgs.msg import motors, ekf_state
import copy
import numpy as np
from mt_dstar_lite import mt_dstar_lite

motor_pub = rospy.Publisher("/igvc/motors_raw", motors, queue_size=10)
path_pub = rospy.Publisher("/igvc_nav/path_map", OccupancyGrid, queue_size=10)

# Moving Target D* Lite
map_init = False
planner = mt_dstar_lite()

# Localization tracking
ekf_offset = (0, 0)
prev_coord = (0, 0)
ekf_init = False
GRID_SIZE = 0.1     # Map block size in meters


# TODO: Do this as a service instead so we can get the EKF offset when the map updates rather than accumulating it ourselves
def ekf_callback(ekf_data):
    global prev_coord, ekf_init, ekf_offset

    # Extract coordinates
    xk = ekf_data.x_k
    x = xk[3]
    y = xk[4]

    # Determine offset (in blocks)
    ekf_offset = ((x - prev_coord[0]) / GRID_SIZE, (y - prev_coord[1]) / GRID_SIZE)

    # Set the EKF to be initialized
    ekf_init = True
# TODO: Do this as a service instead so we can get the EKF offset when the map updates rather than accumulating it ourselves



def c_space_callback(c_space):
    global planner, map_init, ekf_init

    # Get the grid data
    grid_data = c_space.data

    # Make a costmap
    cost_map = [0] * 200 * 200

    # Find the best position
    best_pos = (0,0)
    best_pos_cost = 1000000

    # Only look forward for the goal
    for row in range(101):
        for col in range(200):
            cost_map[(row * 200) + col] = grid_data[(row * 200) + col]
            if grid_data[(row * 200) +  col] == 0:
                new_cost = abs(row-75) + abs(col-100)
                if new_cost < best_pos_cost:
                    best_pos_cost = new_cost
                    best_pos = (row, col)

    # Consider backwards to be an obstacle
    for row in range(101, 200):
        for col in range(200):
            cost_map[(row*200) + col] = 100

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
        planner.initialize(200, 200, (100, 100), best_pos, cost_map)
        path = planner.plan()
    # Otherwise, replan the path
    else:
        # TODO: figure out how to do this
        # Request the planner replan the path
        path = planner.replan(config_space.tolist(), (100, 100), best_pos, cost_map)


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
    else:
        path_msg = copy.deepcopy(c_space)
        data = planner.get_search_space_map()
        data[(best_pos[0]*200) + best_pos[1]] = 100
        print(str(c_space.info.width) + " x " + str(c_space.info.height))
        path_msg.data = data
        path_pub.publish(path_msg)


def mt_dstar_node():
    # Setup node
    rospy.init_node("mt_dstar_node")

    # Subscribe to necessary topics
    rospy.Subscriber("/igvc_slam/config_space", OccupancyGrid, c_space_callback, queue_size=1)  # Mapping
    rospy.Subscriber("/igvc_ekf/filter_output", ekf_state, ekf_callback, queue_size = 1)        # Localization

    # Wait for topic updates
    rospy.spin()



# Main setup
if __name__ == '__main__':
    try:
        mt_dstar_node()
    except rospy.ROSInterruptException:
        pass
