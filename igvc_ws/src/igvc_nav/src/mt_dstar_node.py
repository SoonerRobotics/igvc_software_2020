#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import String, Header
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, Transform, TransformStamped, Vector3
from nav_msgs.msg import OccupancyGrid, Path, MapMetaData
from igvc_msgs.msg import motors, ekf_state
from igvc_msgs.srv import EKFService
import copy
import numpy as np
from path_planner.mt_dstar_lite import mt_dstar_lite

motor_pub = rospy.Publisher("/igvc/motors_raw", motors, queue_size=10)
#path_pub = rospy.Publisher("/igvc_nav/path_map", OccupancyGrid, queue_size=10)
global_path_pub = rospy.Publisher("/igvc/global_path", Path, queue_size=1)
local_path_pub = rospy.Publisher("/igvc/local_path", Path, queue_size=1)

# Moving Target D* Lite
map_init = False
path_failed = False
planner = mt_dstar_lite()

grid_data = None

# Location when map was made
map_reference = (0, 0)

# Localization tracking
prev_state = (0, 0)  # x, y
GRID_SIZE = 0.1      # Map block size in meters #TODO: the grid size should be 0.1 I think

# Path tracking
path_seq = 0

cost_map = None

curEKF = None

best_pos = (0,0)

def ekf_callback(data):
    global curEKF
    curEKF = data

def pose_stamped_from_position(x, y):
    pose_stamped = PoseStamped()
    pose_stamped.pose = Pose()

    point = Point()
    point.x = x
    point.y = y
    pose_stamped.pose.position = point

    return pose_stamped

def c_space_callback(c_space):
    global grid_data, cost_map, map_reference, map_init, best_pos

    grid_data = c_space.data

    # get position that we are at when map is made :)
    map_reference = (curEKF.x_k[4], curEKF.x_k[3])
    map_init = False

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

def make_map(c_space):
    global planner, map_init, path_failed, prev_state, path_seq, grid_data

    if grid_data is None or curEKF is None:
        return

    # Are we somehow on a bad spot?
    if grid_data[(100 * 200) + 100] == 1:
        return

    # Reset the path
    path = None

    robot_pos = (100 - int((curEKF.x_k[4] - map_reference[0]) / GRID_SIZE), 100 + int((curEKF.x_k[3] - map_reference[1]) / GRID_SIZE))

    print("robot_pos", robot_pos)

    # MOVING TARGET D*LITE
    # If this is the first time receiving a map, or if the path failed to be made last time (for robustness),
    # initialize the path planner and plan the first path
    if map_init == False or path_failed == True:
        planner.initialize(200, 200, robot_pos, best_pos, cost_map)
        path = planner.plan()
        map_init = True
    # Otherwise, replan the path
    else:
        # Transform the map to account for heading changes
        hdg = curEKF.x_k[5]
        #TODO: rotate map to 0 degree heading

        # Calculate the map shift based on the change in EKF state
        map_shift = (int(curEKF.x_k[3] / GRID_SIZE) - prev_state[0], int(curEKF.x_k[4] / GRID_SIZE) - prev_state[1])
        prev_state = (int(curEKF.x_k[3] / GRID_SIZE), int(curEKF.x_k[4] / GRID_SIZE))

        # Request the planner replan the path
        path = planner.replan(robot_pos, best_pos, cost_map) #, map_shift) # TODO: add in shifting


    # print "path:"
    # print path

    if path is not None:
        global_path = Path()
        local_path = Path()
        
        header = Header()
        header.seq = path_seq
        header.stamp = rospy.Time.now()
        header.frame_id = "base_link"

        global_path.header = header
        local_path.header = header

        path_seq += 1

        def path_point_to_global(pp0, pp1):
            # Local path
            x = (100 - pp1) * GRID_SIZE
            y = (100 - pp0) * GRID_SIZE

            # Translate to global path
            dx = curEKF.x_k[4]
            dy = curEKF.x_k[3]
            psi = curEKF.x_k[5]

            x = x * math.cos(psi) - y * math.sin(psi) + dx
            y = y * math.cos(psi) + x * math.sin(psi) + dy

            return pose_stamped_from_position(x, y)

        global_path.poses = [path_point_to_global(path_point[0], path_point[1]) for path_point in path]
        global_path.poses.reverse() # reverse path becuz its backwards lol

        global_path_pub.publish(global_path)

        def path_point_to_local(pp0, pp1):
            # Local path
            x = pp1 * GRID_SIZE
            y = pp0 * GRID_SIZE

            return pose_stamped_from_position(x, y)

        local_path.poses = [path_point_to_local(path_point[0], path_point[1]) for path_point in path]
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
    rospy.Subscriber("/igvc_slam/local_config_space", OccupancyGrid, c_space_callback, queue_size=1)  # Mapping

    # Wait for the EKF to start advertising its service
    rospy.wait_for_service('/igvc_ekf/get_robot_state')

    rospy.Subscriber("/igvc_ekf/filter_output", ekf_state, ekf_callback)

    # Make a timer to publish cnew paths
    timer = rospy.Timer(rospy.Duration(secs=0.05), make_map, oneshot=False)

    # Wait for topic updates
    rospy.spin()



# Main setup
if __name__ == '__main__':
    try:
        mt_dstar_node()
    except rospy.ROSInterruptException:
        pass
