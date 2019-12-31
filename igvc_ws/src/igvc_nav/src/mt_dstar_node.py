#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
from igvc_msgs.msg import motors
import copy
import numpy as np
from mt_dstar_lite import mt_dstar_lite

motor_pub = rospy.Publisher("/igvc/motors_raw", motors, queue_size=10)
config_pub = rospy.Publisher("/igvc_vision/config_space", OccupancyGrid, queue_size=10)
path_pub = rospy.Publisher("/igvc_vision/path_map", OccupancyGrid, queue_size=10)

# Moving Target D* Lite
# planner = mt_dstar_lite(200, 200)


def lidar_callback(data):
    config_space = np.zeros((200, 200))
    config_data = [0] * (200 * 200)

    for x in range(200):
        for y in range(200):
            new_x = 199 - x
            new_y = 199 - y
            if new_y < 100:
                config_space[new_x, new_y] = 1
            if data.data[x + y * 200] > 0:
                for x_i in range(-5,6):
                    for y_i in range(-5,6):
                        dist = (x_i)**2 + (y_i)**2
                        new_x = 199 - (x + x_i)
                        new_y = 199 - (y + y_i)
                        if 0 <= new_x < 200 and 0 <= new_y < 200 and dist <= 25:
                            config_space[new_x, new_y] = 1
                            config_data[(x + x_i) + 200 * (y + y_i)] = 100

    config_msg = copy.copy(data)
    config_msg.data = config_data
    config_pub.publish(config_msg)

    best_pos = (0,0)
    best_pos_cost = 10000
    for x in range(200):
        for y in range(200):
            if config_space[x, y] == 0:
                new_cost = abs(y-125) + abs(x-100)
                if new_cost < best_pos_cost:
                    best_pos_cost = new_cost
                    best_pos = (x, y)

    print "target:"
    print best_pos

    # Are we somehow on a bad spot
    if config_space[100, 100] == 1:
        return

    #path = planner.plan(config_space.tolist(), (100, 100), best_pos)
    path = None

    print "path:"
    print path

    if path is not None:
        path_space = [0] * 200 * 200
        itt = 100
        for path_pos in path:
            path_space[(199 - path_pos[0]) + 200 * (199 - path_pos[1])] = itt
            itt -= 1

        path_msg = copy.deepcopy(data)
        path_msg.data = path_space

        path_pub.publish(path_msg)


def mt_dstar_node():

    # Setup node
    rospy.init_node("mt_dstar_node")

    # Subscribe to necessary topics
    rospy.Subscriber("/igvc_vision/map", OccupancyGrid, lidar_callback, queue_size=1)

    # Wait for topic updates
    rospy.spin()



# Main setup
if __name__ == '__main__':
    try:
        mt_dstar_node()
    except rospy.ROSInterruptException:
        pass
