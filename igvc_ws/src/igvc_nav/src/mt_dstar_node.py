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

#temps
import matplotlib as mpl
from matplotlib import pyplot as plt

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

    if curEKF is None:
        return

    grid_data = c_space.data

    # Make a costmap
    temp_cost_map = [0] * 200 * 200

    # Find the best position
    temp_best_pos = (0,0)
    best_pos_cost = 1000000

    # Only look forward for the goal
    for row in range(101):
        for col in range(200):
            temp_cost_map[(row * 200) + col] = grid_data[(row * 200) + col]
            if grid_data[(row * 200) +  col] == 0:
                new_cost = abs(row-55) + abs(col-100)
                if new_cost < best_pos_cost:
                    best_pos_cost = new_cost
                    temp_best_pos = (row, col)

    # Consider backwards to be an obstacle
    for row in range(101, 200):
        for col in range(200):
            temp_cost_map[(row*200) + col] = 100


    map_reference = (curEKF.x_k[3], curEKF.x_k[4])
    cost_map = temp_cost_map
    best_pos = temp_best_pos
    map_init = False

def make_map(c_space):
    global planner, map_init, path_failed, prev_state, path_seq, grid_data

    if cost_map is None or curEKF is None:
        return

    # Reset the path
    path = None

    robot_pos = (100 + int((curEKF.x_k[3] - map_reference[0]) / GRID_SIZE), 100 + int((curEKF.x_k[4] - map_reference[1]) / GRID_SIZE))

    # MOVING TARGET D*LITE
    # If this is the first time receiving a map, or if the path failed to be made last time (for robustness),
    # initialize the path planner and plan the first path
    if True:
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
            x = (robot_pos[0] - pp0) * GRID_SIZE
            y = (robot_pos[1] - pp1) * GRID_SIZE

            # Translate to global path
            dx = curEKF.x_k[3]
            dy = curEKF.x_k[4]
            psi = curEKF.x_k[5]

            new_x = x * math.cos(psi) - y * math.sin(psi) + dx
            new_y = y * math.cos(psi) + x * math.sin(psi) + dy

            return pose_stamped_from_position(new_x, new_y)

        global_path.poses = [path_point_to_global(path_point[0], path_point[1]) for path_point in path]
        global_path.poses.reverse() # reverse path becuz its backwards lol

        global_path_pub.publish(global_path)

        # make a color map of fixed colors
        cmap = mpl.colors.ListedColormap(['white','pink','red','black','green'])
        bounds=[-1,20,45,55,101]
        norm = mpl.colors.BoundaryNorm(bounds, cmap.N)

        temp_grid = copy.deepcopy(cost_map)

        plt.figure(1)

        for point in path:
            temp_grid[point[1] + 200 * point[0]] = 25

        temp_grid[robot_pos[1] + 200 * robot_pos[0]] = 50
        temp_grid[best_pos[1] + 200 * best_pos[0]] = 50

        # tell imshow about color map so that only set colors are used
        plt.imshow(np.reshape(temp_grid, (200, 200)),interpolation='nearest',
                            cmap = cmap,norm=norm)

        plt.draw()
        plt.pause(0.00000000001)
        
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
    timer = rospy.Timer(rospy.Duration(secs=0.1), make_map, oneshot=False)

    plt.ion()
    plt.show()

    # Wait for topic updates
    rospy.spin()



# Main setup
if __name__ == '__main__':
    try:
        mt_dstar_node()
    except rospy.ROSInterruptException:
        pass
