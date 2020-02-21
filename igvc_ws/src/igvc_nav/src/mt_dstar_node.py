#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Pose, Point
from nav_msgs.msg import OccupancyGrid, Path
from igvc_msgs.msg import motors, ekf_state
from igvc_msgs.srv import EKFService
import copy
import numpy as np
from path_planner.mt_dstar_lite import mt_dstar_lite

motor_pub = rospy.Publisher("/igvc/motors_raw", motors, queue_size=10)
#path_pub = rospy.Publisher("/igvc_nav/path_map", OccupancyGrid, queue_size=10)
local_path_pub = rospy.Publisher("/igvc/local_path", Path, queue_size=1)

# Moving Target D* Lite
map_init = False
path_failed = False
planner = mt_dstar_lite()

# Localization tracking
prev_state = (0, 0)  # x, y
GRID_SIZE = 0.1      # Map block size in meters #TODO: the grid size should be 0.1 I think

def pose_stamped_from_position(x, y):
    pose_stamped = PoseStamped()
    pose_stamped.pose = Pose()

    point = Point()
    point.x = x
    point.y = y
    pose_stamped.pose.position = point

    return pose_stamped

def c_space_callback(c_space):
    global planner, map_init, path_failed, prev_state

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
    # If this is the first time receiving a map, or if the path failed to be made last time (for robustness),
    # initialize the path planner and plan the first path
    if map_init == False or path_failed == True:
        planner.initialize(200, 200, (100, 100), best_pos, cost_map)
        path = planner.plan()
        map_init = True
    # Otherwise, replan the path
    else:
        # Get the EKF's robot state estimate
        robot_state = rospy.ServiceProxy('/igvc_ekf/get_robot_state', EKFService)
        xk = robot_state()

        # Update the robot's position on the map
        robot_pos = (100 - int(xk.state.x_k[3] / GRID_SIZE), 100 + int(xk.state.x_k[4] / GRID_SIZE))

        # Transform the map to account for heading changes
        hdg = xk.state.x_k[5]
        #TODO: rotate map to 0 degree heading

        # Calculate the map shift based on the change in EKF state
        map_shift = (int(xk.state.x_k[3] / GRID_SIZE) - prev_state[0], int(xk.state.x_k[4] / GRID_SIZE) - prev_state[1])
        prev_state = (int(xk.state.x_k[3] / GRID_SIZE), int(xk.state.x_k[4] / GRID_SIZE))

        # Request the planner replan the path
        path = planner.replan(robot_pos, best_pos, cost_map) #, map_shift) # TODO: add in shifting


    print "path:"
    print path

    if path is not None:
        # path_space = [0] * 200 * 200
        # itt = 100
        # for path_pos in path:
        #     path_space[(200 * path_pos[0]) + path_pos[1]] = itt
        #     itt -= 1

        # path_msg = copy.deepcopy(c_space)
        # path_msg.data = path_space

        local_path = Path()
        local_path.poses = [pose_stamped_from_position(200 - path_point[1], 200 - path_point[0]) for path_point in path]
        local_path.poses.reverse() # reverse path becuz its backwards lol

        local_path_pub.publish(local_path)
    else:
        # Set the path failed flag so we can fully replan
        path_failed = True

        # path_msg = copy.deepcopy(c_space)
        # data = planner.get_search_space_map()
        # data[(best_pos[0]*200) + best_pos[1]] = 100
        # print(str(c_space.info.width) + " x " + str(c_space.info.height))
        # path_msg.data = data
        # path_pub.publish(path_msg)


def mt_dstar_node():
    # Setup node
    rospy.init_node("mt_dstar_node")

    # Subscribe to necessary topics
    rospy.Subscriber("/igvc_slam/config_space", OccupancyGrid, c_space_callback, queue_size=1)  # Mapping

    # Wait for the EKF to start advertising its service
    rospy.wait_for_service('/igvc_ekf/get_robot_state')

    # Wait for topic updates
    rospy.spin()



# Main setup
if __name__ == '__main__':
    try:
        mt_dstar_node()
    except rospy.ROSInterruptException:
        pass
